"""Host-side driver for the Warp digital twin.

`DigitalTwin` uploads a show's piecewise trajectories once, then `run()`
simulates one disturbance scenario. Per physics step (200 Hz):

    1. eval_reference_kernel   -- p/v/a/LED references from the Phase 2 splines
    2. HashGrid.build           -- spatial hash of current positions
    3. interaction_kernel       -- downwash per motor + nearest-neighbour distance
    4. step_kernel              -- controller, 6-DOF plant, motors, battery, metrics

All kernel times are seconds since the show start (float32-friendly).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import numpy as np
import warp as wp

from .kernels import aerodynamics as aero
from .kernels import battery_discharge as batt
from .kernels import multi_agent_dynamics as dyn
from .loaders.arrow_loader import ShowTrajectories
from .loaders.spline_evaluator import PiecewiseTrajectories, WarpTrajectoryBuffers, build_piecewise, evaluate_numpy
from .profile import DroneProfile

CONTROL_RATE_HZ = 200.0
GNSS_CORRELATION_TIME_S = 60.0
# Closest-approach tracking resolves separations below this; anything farther
# reports as inf. Comfortably above the 1.0 m warning threshold (§3.4).
PROXIMITY_RADIUS_M = 3.0
_NO_NEIGHBOUR = 1.0e9


@dataclass
class Disturbances:
    """One Monte Carlo scenario: environment + per-drone hardware spread."""

    seed: int
    ambient_c: float
    mean_wind: np.ndarray                 # (3,) m/s
    mode_amp: np.ndarray                  # (M, 3) m/s
    mode_k: np.ndarray                    # (M, 3) rad/m
    mode_omega: np.ndarray                # (M,) rad/s
    mode_phase: np.ndarray                # (M,)
    gust_vec: np.ndarray                  # (3,) m/s peak
    gust_dir: np.ndarray                  # (3,) unit
    gust_start: float                     # s since show start
    gust_duration: float                  # s (0 = no gust)
    gust_speed: float                     # m/s
    mass_kg: np.ndarray                   # (N,) per-drone plant values
    max_thrust_n: np.ndarray
    motor_tau_s: np.ndarray
    drag_cd: np.ndarray
    capacity_ah: np.ndarray
    internal_resistance_ohm: np.ndarray
    initial_soc: np.ndarray
    gnss_noise_m: float
    gnss_drift_m_per_sqrt_s: float
    initial_position_error_m: float

    @classmethod
    def nominal(cls, profile: DroneProfile, fleet_size: int, seed: int = 0, ambient_c: float = 25.0) -> "Disturbances":
        """Calm air, every drone exactly at the profile's nominal values, perfect GNSS."""
        ones = np.ones(fleet_size)
        return cls(
            seed=seed,
            ambient_c=ambient_c,
            mean_wind=np.zeros(3),
            mode_amp=np.zeros((0, 3)),
            mode_k=np.zeros((0, 3)),
            mode_omega=np.zeros(0),
            mode_phase=np.zeros(0),
            gust_vec=np.zeros(3),
            gust_dir=np.array([1.0, 0.0, 0.0]),
            gust_start=0.0,
            gust_duration=0.0,
            gust_speed=1.0,
            mass_kg=ones * profile.get("physical.mass_kg"),
            max_thrust_n=ones * profile.get("motor_prop.max_thrust_per_motor_n"),
            motor_tau_s=ones * profile.get("motor_prop.motor_time_constant_ms") / 1000.0,
            drag_cd=ones * profile.get("physical.drag_coefficient_cd"),
            capacity_ah=ones * profile.get("battery.capacity_mah") / 1000.0,
            internal_resistance_ohm=ones * profile.get("battery.internal_resistance_ohm"),
            initial_soc=ones * profile.get("battery.initial_soc"),
            gnss_noise_m=0.0,
            gnss_drift_m_per_sqrt_s=0.0,
            initial_position_error_m=0.0,
        )


@dataclass
class SimConfig:
    dt: float = 1.0 / CONTROL_RATE_HZ
    tail_sec: float = 0.0          # keep simulating after the show ends (settling)
    record_hz: float = 0.0         # >0: record positions at this rate (host copies, slow on GPU)


@dataclass
class SimResult:
    duration_sec: float
    wall_time_sec: float
    steps: int
    min_separation_m: np.ndarray      # (N,) closest approach to any other drone (>= PROXIMITY_RADIUS_M -> inf)
    min_separation_partner: np.ndarray
    min_separation_time: np.ndarray
    max_tracking_error_m: np.ndarray
    final_soc: np.ndarray
    min_voltage_v: np.ndarray
    final_temp_c: np.ndarray
    brownout: np.ndarray              # (N,) bool
    brownout_time: np.ndarray
    final_position: np.ndarray        # (N, 3)
    recorded_times: np.ndarray = field(default_factory=lambda: np.zeros(0))
    recorded_positions: np.ndarray = field(default_factory=lambda: np.zeros((0, 0, 3)))

    @property
    def realtime_factor(self) -> float:
        return self.duration_sec / self.wall_time_sec if self.wall_time_sec > 0 else math.inf


def controller_params(profile: DroneProfile, disturbances: Disturbances) -> dyn.ControllerParams:
    c = dyn.ControllerParams()
    c.mass_nominal = profile.get("physical.mass_kg")
    c.inertia = wp.vec3(*profile.get("physical.inertia_diag_kgm2"))
    c.half_diag = profile.get("physical.arm_length_m") / math.sqrt(2.0)
    c.torque_to_thrust = profile.get("motor_prop.rotor_torque_to_thrust_m")
    c.max_thrust_nominal = profile.get("motor_prop.max_thrust_per_motor_n")
    c.nominal_voltage = profile.get("battery.nominal_voltage_v")
    c.pos_p = wp.vec3(*profile.get("controller_gains.pos_p"))
    c.vel_p = wp.vec3(*profile.get("controller_gains.vel_p"))
    c.vel_i = wp.vec3(*profile.get("controller_gains.vel_i"))
    c.att_p = wp.vec3(*profile.get("controller_gains.att_p"))
    c.rate_p = wp.vec3(*profile.get("controller_gains.rate_p"))
    c.tan_max_tilt = math.tan(math.radians(profile.get("controller_gains.max_tilt_deg")))
    c.drag_area = profile.get("physical.reference_area_m2")
    c.gnss_noise = disturbances.gnss_noise_m
    c.gnss_drift = disturbances.gnss_drift_m_per_sqrt_s
    c.gnss_corr_time = GNSS_CORRELATION_TIME_S
    return c


def open_circuit_voltage(profile: DroneProfile, soc: np.ndarray) -> np.ndarray:
    return profile.get("battery.cells_s") * np.interp(soc, batt.OCV_SOC_TABLE, batt.OCV_CELL_VOLTS)


class DigitalTwin:
    def __init__(self, show: ShowTrajectories | PiecewiseTrajectories, profile: DroneProfile,
                 device: str | None = None):
        wp.init()
        self.device = wp.get_device(device)
        self.profile = profile
        self.pw = show if isinstance(show, PiecewiseTrajectories) else build_piecewise(show)
        self.n = self.pw.fleet_size
        self.refs = WarpTrajectoryBuffers(self.pw, device=self.device)

        # ENU ground plane is z = 0 (Phase 1 origin), lowered only if a launch
        # pad sits below it. Never inferred from the lowest start altitude:
        # a show that starts airborne would otherwise "fly on the ground".
        start_pos, _, _ = evaluate_numpy(self.pw, np.array([self.pw.start_time_sec]))
        self.ground_z = min(0.0, float(start_pos[:, 0, 2].min()))

        arm = profile.get("physical.arm_length_m")
        prop = profile.get("physical.propeller_diameter_m")
        self.r0 = aero.wake_radius_at_rotor(arm, prop)
        self.disk_area = aero.total_disk_area(prop)
        self.half_diag = arm / math.sqrt(2.0)
        self.wake_depth = aero.wake_query_depth(self.r0)
        self.wake_query_radius = aero.wake_query_radius(self.r0, self.half_diag)
        self.grid = wp.HashGrid(128, 128, 128, device=self.device)

    # ------------------------------------------------------------------ #

    def _battery_params(self, ambient_c: float) -> batt.BatteryParams:
        p = self.profile
        b = batt.BatteryParams()
        b.cells = float(p.get("battery.cells_s"))
        b.cutoff_v = p.get("battery.cutoff_voltage_v")
        b.efficiency = p.get("battery.powertrain_efficiency")
        b.peukert = p.get("battery.peukert_exponent")
        b.led_max_power_w = p.get("battery.led_max_power_w")
        b.avionics_a = p.get("battery.avionics_current_a")
        b.rotor_torque_to_thrust_m = p.get("motor_prop.rotor_torque_to_thrust_m")
        b.max_rotor_speed = batt.max_rotor_speed_rad_s(p.get("motor_prop.kv_rating"),
                                                       p.get("battery.nominal_voltage_v"))
        b.nominal_max_thrust = p.get("motor_prop.max_thrust_per_motor_n")
        b.ambient_c = ambient_c
        b.ocv_soc = wp.array(batt.OCV_SOC_TABLE, dtype=float, device=self.device)
        b.ocv_volts = wp.array(batt.OCV_CELL_VOLTS, dtype=float, device=self.device)
        return b

    def _wind_field(self, dist: Disturbances) -> aero.WindField:
        w = aero.WindField()
        f32 = np.float32
        w.mean = wp.vec3(*dist.mean_wind)
        w.n_modes = int(dist.mode_amp.shape[0])
        # zero-length arrays are fine for Warp but keep at least one slot for older runtimes
        m = max(w.n_modes, 1)
        pad = lambda a, shape: np.zeros(shape, f32) if a.size == 0 else a.astype(f32)  # noqa: E731
        w.mode_amp = wp.array(pad(dist.mode_amp, (m, 3)), dtype=wp.vec3, device=self.device)
        w.mode_k = wp.array(pad(dist.mode_k, (m, 3)), dtype=wp.vec3, device=self.device)
        w.mode_omega = wp.array(pad(dist.mode_omega, (m,)), dtype=float, device=self.device)
        w.mode_phase = wp.array(pad(dist.mode_phase, (m,)), dtype=float, device=self.device)
        w.gust_vec = wp.vec3(*dist.gust_vec)
        w.gust_dir = wp.vec3(*dist.gust_dir)
        w.gust_start = float(dist.gust_start)
        w.gust_duration = float(dist.gust_duration)
        w.gust_speed = float(dist.gust_speed)
        return w

    def run(self, dist: Disturbances, config: SimConfig | None = None) -> SimResult:
        config = config or SimConfig()
        n, dev = self.n, self.device
        f32 = np.float32

        def arr(values, dtype=float):
            return wp.array(np.asarray(values).astype(np.int32 if dtype is int else f32), dtype=dtype, device=dev)

        def zeros(dtype):
            return wp.zeros(n, dtype=dtype, device=dev)

        def full(value, dtype=float):
            return arr(np.full(n, value), dtype)

        ctrl = controller_params(self.profile, dist)
        battery = self._battery_params(dist.ambient_c)
        wind = self._wind_field(dist)

        mass = arr(dist.mass_kg)
        max_thrust = arr(dist.max_thrust_n)
        motor_tau = arr(dist.motor_tau_s)
        drag_cd = arr(dist.drag_cd)
        capacity = arr(dist.capacity_ah)
        r_int = arr(dist.internal_resistance_ohm)

        ref_p, ref_v, ref_a = zeros(wp.vec3), zeros(wp.vec3), zeros(wp.vec3)
        led = zeros(float)
        pos, vel, omega = zeros(wp.vec3), zeros(wp.vec3), zeros(wp.vec3)
        quat = zeros(wp.quat)
        thrust = zeros(wp.vec4)
        vel_int, gnss_bias = zeros(wp.vec3), zeros(wp.vec3)
        armed = zeros(int)
        total_thrust, hover_thrust = zeros(float), zeros(float)
        lift = arr(np.ones((n, 4)), wp.vec4)
        downwash = zeros(wp.vec3)
        nearest_dist, nearest_id = zeros(float), zeros(int)

        soc = wp.array(np.asarray(dist.initial_soc, dtype=np.float64), dtype=wp.float64, device=dev)
        v0 = open_circuit_voltage(self.profile, dist.initial_soc)
        voltage = arr(v0)
        temp = full(dist.ambient_c)
        low_timer, brownout = zeros(float), zeros(int)
        max_track, min_volt = zeros(float), arr(v0)
        brown_t = full(-1.0)
        min_sep, min_sep_partner, min_sep_time = full(_NO_NEIGHBOUR), full(-1, int), full(-1.0)

        t0 = self.pw.start_time_sec
        duration = (self.pw.end_time_sec - t0) + config.tail_sec
        steps = int(math.ceil(duration / config.dt - 1e-9))
        seed = int(dist.seed) & 0x3FFFFFFF

        self.refs.launch_reference(t0, ref_p, ref_v, ref_a, led)
        wp.launch(dyn.init_state_kernel, dim=n,
                  inputs=[ref_p, self.ground_z, float(dist.initial_position_error_m), seed, mass],
                  outputs=[pos, vel, quat, omega, thrust, total_thrust, hover_thrust, armed], device=dev)

        record_every = int(round(1.0 / (config.record_hz * config.dt))) if config.record_hz > 0 else 0
        rec_t, rec_p = [], []

        # Record each per-step launch once and replay it, updating only the
        # time/step parameters: re-packing ~40 arguments (incl. structs) per
        # launch otherwise dominates small fleets.
        ref_cmd = self.refs.launch_reference(t0, ref_p, ref_v, ref_a, led, record_cmd=True)
        aero_cmd = wp.launch(aero.interaction_kernel, dim=n,
                             inputs=[self.grid.id, pos, quat, total_thrust, hover_thrust, PROXIMITY_RADIUS_M,
                                     self.wake_depth, self.wake_query_radius, self.r0, self.half_diag,
                                     self.disk_area],
                             outputs=[lift, downwash, nearest_dist, nearest_id], device=dev, record_cmd=True)
        sep_cmd = wp.launch(track_separation_kernel, dim=n, inputs=[0.0, nearest_dist, nearest_id],
                            outputs=[min_sep, min_sep_partner, min_sep_time], device=dev, record_cmd=True)
        step_cmd = wp.launch(dyn.step_kernel, dim=n,
                             inputs=[0.0, float(config.dt), 0, seed, self.ground_z, ctrl, wind, battery,
                                     ref_p, ref_v, ref_a, led, lift, downwash,
                                     mass, max_thrust, motor_tau, drag_cd, capacity, r_int],
                             outputs=[pos, vel, quat, omega, thrust, vel_int, gnss_bias, armed, total_thrust,
                                      soc, temp, voltage, low_timer, brownout, max_track, min_volt, brown_t],
                             device=dev, record_cmd=True)
        wp.synchronize_device(dev)
        wall_start = time.perf_counter()
        origin_offset = t0 - self.refs.time_origin
        for k in range(steps):
            t_rel = k * config.dt
            ref_cmd.set_param_by_name("t", origin_offset + t_rel)
            ref_cmd.launch()
            self.grid.build(pos, PROXIMITY_RADIUS_M)
            aero_cmd.launch()
            sep_cmd.set_param_by_name("t", t_rel)
            sep_cmd.launch()
            step_cmd.set_param_by_name("t", t_rel)
            step_cmd.set_param_by_name("step", k)
            step_cmd.launch()
            if record_every and k % record_every == 0:
                rec_t.append(t_rel + config.dt)
                rec_p.append(pos.numpy().copy())
        wp.synchronize_device(dev)
        wall = time.perf_counter() - wall_start

        sep = min_sep.numpy().astype(np.float64)
        sep[sep >= _NO_NEIGHBOUR * 0.5] = np.inf
        return SimResult(
            duration_sec=steps * config.dt,
            wall_time_sec=wall,
            steps=steps,
            min_separation_m=sep,
            min_separation_partner=min_sep_partner.numpy().astype(np.int64),
            min_separation_time=min_sep_time.numpy().astype(np.float64) + t0,
            max_tracking_error_m=max_track.numpy().astype(np.float64),
            final_soc=soc.numpy().astype(np.float64),
            min_voltage_v=min_volt.numpy().astype(np.float64),
            final_temp_c=temp.numpy().astype(np.float64),
            brownout=brownout.numpy().astype(bool),
            brownout_time=np.where(brownout.numpy() > 0, brown_t.numpy() + t0, np.nan),
            final_position=pos.numpy().astype(np.float64),
            recorded_times=np.asarray(rec_t) + t0,
            recorded_positions=np.asarray(rec_p) if rec_p else np.zeros((0, n, 3)),
        )


@wp.kernel
def track_separation_kernel(t: float, nearest_dist: wp.array(dtype=float), nearest_id: wp.array(dtype=int),
                            min_sep: wp.array(dtype=float), partner: wp.array(dtype=int),
                            when: wp.array(dtype=float)):
    d = wp.tid()
    dist = nearest_dist[d]
    if dist < min_sep[d]:
        min_sep[d] = dist
        partner[d] = nearest_id[d]
        when[d] = t
