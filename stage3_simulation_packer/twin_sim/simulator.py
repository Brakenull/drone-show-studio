"""Host-side driver for the OpenCL digital twin.

`DigitalTwin` uploads a show's piecewise trajectories once; `run()` simulates
one disturbance scenario and `run_batch()` steps several Monte Carlo
scenarios of the same show together (one launch covers runs x drones, so a
GPU is kept busy even for small fleets). Per physics step (200 Hz):

    1. eval_reference        -- p/v/a/LED references from the Phase 2 splines (shared by all runs)
    2. grid_count / grid_scan / grid_scatter / grid_sort_cells -- neighbour grid of current positions
    3. interaction           -- downwash per motor, nearest neighbour, closest approach
    4. step_dynamics         -- controller, 6-DOF plant, motors, battery, metrics

Kernels live in kernels/*.cl (see common.cl for the build layout). All kernel
times are seconds since the show start (float32-friendly). A scenario's
result does not depend on the device's scheduling or on which batch it ran in.
"""

from __future__ import annotations

import math
import time
import warnings
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pyopencl as cl

from . import physics
from .devices import DeviceInfo, select_device
from .loaders.arrow_loader import ShowTrajectories
from .loaders.spline_evaluator import PiecewiseTrajectories, build_piecewise, evaluate_numpy
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
    record_hz: float = 0.0         # >0: record positions at this rate (single scenario only)


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


def open_circuit_voltage(profile: DroneProfile, soc: np.ndarray) -> np.ndarray:
    return physics.open_circuit_voltage(profile.get("battery.cells_s"), soc)


KERNEL_DIR = Path(__file__).with_name("kernels")
KERNEL_FILES = ("common.cl", "references.cl", "grid.cl", "aerodynamics.cl", "battery_discharge.cl",
                "multi_agent_dynamics.cl")
GRID_CELL_M = PROXIMITY_RADIUS_M  # neighbour-grid cell size
GRID_G = 16                       # cells per axis before the grid wraps (power of two)
SCAN_WG = 256                     # work-group size of grid_scan; divides GRID_G ** 3
RP_MODES = 20                     # first turbulence-mode slot in the per-run block (common.cl)
LOCAL_SIZE = 64                   # global sizes are padded to a multiple of this


def _c_float(x: float) -> str:
    return f"{float(x)!r}f"


def _c_vec3(v) -> str:
    return "((float3)(" + ", ".join(_c_float(c) for c in v) + "))"


def _padded(n: int) -> tuple[int]:
    return ((n + LOCAL_SIZE - 1) // LOCAL_SIZE * LOCAL_SIZE,)


class DigitalTwin:
    def __init__(self, show: ShowTrajectories | PiecewiseTrajectories, profile: DroneProfile,
                 device: str | None = None):
        self.device_info, cl_device = select_device(device)
        self.ctx = cl.Context([cl_device])
        self.queue = cl.CommandQueue(self.ctx)
        self.profile = profile
        self.pw = show if isinstance(show, PiecewiseTrajectories) else build_piecewise(show)
        self.n = self.pw.fleet_size

        # ENU ground plane is z = 0 (Phase 1 origin), lowered only if a launch
        # pad sits below it. Never inferred from the lowest start altitude:
        # a show that starts airborne would otherwise "fly on the ground".
        start_pos, _, _ = evaluate_numpy(self.pw, np.array([self.pw.start_time_sec]))
        self.ground_z = min(0.0, float(start_pos[:, 0, 2].min()))

        arm = profile.get("physical.arm_length_m")
        prop = profile.get("physical.propeller_diameter_m")
        self.r0 = physics.wake_radius_at_rotor(arm, prop)
        self.disk_area = physics.total_disk_area(prop)
        self.half_diag = arm / math.sqrt(2.0)
        self.wake_depth = physics.wake_query_depth(self.r0)
        self.wake_query_radius = physics.wake_query_radius(self.r0, self.half_diag)
        # A query box must never wrap onto itself, or a neighbour would be counted twice.
        if 2.0 * max(self.wake_query_radius, PROXIMITY_RADIUS_M) / GRID_CELL_M + 2 > GRID_G:
            raise ValueError("the profile's wake query radius is too large for the neighbour grid")

        self._kernels: dict[tuple, dict[str, cl.Kernel]] = {}
        self._upload_references()

    @property
    def device(self) -> DeviceInfo:
        return self.device_info

    # ------------------------------------------------------------------ #

    def _buffer(self, array: np.ndarray, read_only: bool = False) -> cl.Buffer:
        flags = (cl.mem_flags.READ_ONLY if read_only else cl.mem_flags.READ_WRITE) | cl.mem_flags.COPY_HOST_PTR
        return cl.Buffer(self.ctx, flags, hostbuf=np.ascontiguousarray(array))

    def _upload_references(self) -> None:
        """Piecewise trajectories + LED keyframes, times relative to the show start."""
        pw, f32 = self.pw, np.float32
        origin = pw.start_time_sec

        def vec4(a: np.ndarray) -> np.ndarray:
            out = np.zeros((a.shape[0], 4), f32)
            out[:, :3] = a
            return out

        self.offsets = self._buffer(pw.span_offsets.astype(np.int32), True)
        self.span_t0 = self._buffer((pw.span_t0 - origin).astype(f32), True)
        self.span_t1 = self._buffer((pw.span_t1 - origin).astype(f32), True)
        self.span_coef = self._buffer(vec4(pw.span_coef.reshape(-1, 3)), True)
        self.color_offsets = self._buffer(pw.color_offsets.astype(np.int32), True)
        self.color_t = self._buffer((pw.color_t - origin).astype(f32), True)
        self.color_rgb = self._buffer(vec4(pw.color_rgb), True)

    def _program(self, runs: int, dt: float, max_modes: int) -> dict[str, cl.Kernel]:
        """Kernels for this batch size / dt / wind-mode count, built once and cached."""
        key = (runs, dt, max_modes)
        if key in self._kernels:
            return self._kernels[key]
        p = self.profile
        defines = {name: _c_float(value) for name, value in physics.kernel_defines().items()}
        defines.update({
            "N_DRONES": str(self.n), "N_RUNS": str(runs), "N_COEF": str(self.pw.n_coef),
            "RUN_STRIDE": str(RP_MODES + 8 * max_modes),
            "GRID_G": str(GRID_G), "INV_CELL": _c_float(1.0 / GRID_CELL_M), "SCAN_WG": str(SCAN_WG),
            "DT": _c_float(dt), "GROUND_Z": _c_float(self.ground_z),
            "PROX_R": _c_float(PROXIMITY_RADIUS_M), "WAKE_DEPTH": _c_float(self.wake_depth),
            "WAKE_QR": _c_float(self.wake_query_radius), "R0": _c_float(self.r0),
            "HALF_DIAG": _c_float(self.half_diag), "DISK_AREA": _c_float(self.disk_area),
            # controller (nominal profile values)
            "MASS_NOMINAL": _c_float(p.get("physical.mass_kg")),
            "INERTIA": _c_vec3(p.get("physical.inertia_diag_kgm2")),
            "TORQUE_TO_THRUST": _c_float(p.get("motor_prop.rotor_torque_to_thrust_m")),
            "MAX_THRUST_NOMINAL": _c_float(p.get("motor_prop.max_thrust_per_motor_n")),
            "NOMINAL_VOLTAGE": _c_float(p.get("battery.nominal_voltage_v")),
            "POS_P": _c_vec3(p.get("controller_gains.pos_p")), "VEL_P": _c_vec3(p.get("controller_gains.vel_p")),
            "VEL_I": _c_vec3(p.get("controller_gains.vel_i")), "ATT_P": _c_vec3(p.get("controller_gains.att_p")),
            "RATE_P": _c_vec3(p.get("controller_gains.rate_p")),
            "TAN_MAX_TILT": _c_float(math.tan(math.radians(p.get("controller_gains.max_tilt_deg")))),
            "DRAG_AREA": _c_float(p.get("physical.reference_area_m2")),
            "GNSS_CORR_TIME": _c_float(GNSS_CORRELATION_TIME_S),
            # battery
            "BATT_CELLS": _c_float(p.get("battery.cells_s")),
            "BATT_CUTOFF_V": _c_float(p.get("battery.cutoff_voltage_v")),
            "BATT_EFFICIENCY": _c_float(p.get("battery.powertrain_efficiency")),
            "BATT_PEUKERT": _c_float(p.get("battery.peukert_exponent")),
            "BATT_LED_MAX_POWER": _c_float(p.get("battery.led_max_power_w")),
            "BATT_AVIONICS_A": _c_float(p.get("battery.avionics_current_a")),
            "BATT_TORQUE_TO_THRUST": _c_float(p.get("motor_prop.rotor_torque_to_thrust_m")),
            "BATT_MAX_ROTOR_SPEED": _c_float(physics.max_rotor_speed_rad_s(p.get("motor_prop.kv_rating"),
                                                                           p.get("battery.nominal_voltage_v"))),
            "BATT_NOMINAL_MAX_THRUST": _c_float(p.get("motor_prop.max_thrust_per_motor_n")),
            "OCV_N": str(len(physics.OCV_SOC_TABLE)),
            "OCV_SOC_LIST": ", ".join(_c_float(x) for x in physics.OCV_SOC_TABLE),
            "OCV_VOLTS_LIST": ", ".join(_c_float(x) for x in physics.OCV_CELL_VOLTS),
        })
        source = "".join(f"#define {k} {v}\n" for k, v in defines.items())
        source += "\n".join((KERNEL_DIR / name).read_text(encoding="utf-8") for name in KERNEL_FILES)
        with warnings.catch_warnings():
            # Intel's CPU compiler reports vectorization remarks even on success; build errors still raise.
            warnings.simplefilter("ignore", cl.CompilerWarning)
            program = cl.Program(self.ctx, source).build()
        kernels = {k.function_name: k for k in program.all_kernels()}
        self._kernels[key] = kernels
        return kernels

    @staticmethod
    def _run_params(dists: list[Disturbances], max_modes: int) -> np.ndarray:
        """Per-run parameter blocks, laid out as in common.cl (RP_*)."""
        out = np.zeros((len(dists), RP_MODES + 8 * max_modes), np.float32)
        for row, d in zip(out, dists):
            row[0] = d.ambient_c
            row[1] = d.gnss_noise_m
            row[2] = d.gnss_drift_m_per_sqrt_s
            row[3] = d.initial_position_error_m
            row[4:7] = d.mean_wind
            row[7:10] = d.gust_vec
            row[10:13] = d.gust_dir
            row[13] = d.gust_start
            row[14] = d.gust_duration
            row[15] = d.gust_speed
            m = d.mode_amp.shape[0]
            row[16] = m
            modes = row[RP_MODES:RP_MODES + 8 * m].reshape(m, 8)
            modes[:, 0:3] = d.mode_amp
            modes[:, 3:6] = d.mode_k
            modes[:, 6] = d.mode_omega
            modes[:, 7] = d.mode_phase
        return out.reshape(-1)

    # ------------------------------------------------------------------ #

    def sample_grid(self, times_abs: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Reference position/velocity of every drone at every time, evaluated on the device: (N, T, 3) each."""
        times = (np.asarray(times_abs, dtype=np.float64) - self.pw.start_time_sec).astype(np.float32)
        count = self.n * times.shape[0]
        out_p = cl.Buffer(self.ctx, cl.mem_flags.WRITE_ONLY, max(count, 1) * 16)
        out_v = cl.Buffer(self.ctx, cl.mem_flags.WRITE_ONLY, max(count, 1) * 16)
        times_buf = self._buffer(times, True)  # keep alive until enqueued: set_args does not retain it
        kernel = self._program(1, 1.0 / CONTROL_RATE_HZ, 1)["sample_grid"]
        kernel.set_args(np.int32(times.shape[0]), times_buf, self.offsets, self.span_t0,
                        self.span_t1, self.span_coef, out_p, out_v)
        cl.enqueue_nd_range_kernel(self.queue, kernel, _padded(count), (LOCAL_SIZE,))
        p = np.empty((count, 4), np.float32)
        v = np.empty((count, 4), np.float32)
        cl.enqueue_copy(self.queue, p, out_p)
        cl.enqueue_copy(self.queue, v, out_v)
        shape = (self.n, times.shape[0], 3)
        return p[:, :3].reshape(shape), v[:, :3].reshape(shape)

    def references(self, t_abs: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """What `step_dynamics` tracks at `t_abs`, from the device: p, v, a (N, 3) and LED power fraction (N,)."""
        n, f32 = self.n, np.float32
        outs = [cl.Buffer(self.ctx, cl.mem_flags.WRITE_ONLY, n * 16) for _ in range(3)]
        led = cl.Buffer(self.ctx, cl.mem_flags.WRITE_ONLY, n * 4)
        kernel = self._program(1, 1.0 / CONTROL_RATE_HZ, 1)["eval_reference"]
        kernel.set_args(f32(t_abs - self.pw.start_time_sec), self.offsets, self.span_t0, self.span_t1,
                        self.span_coef, self.color_offsets, self.color_t, self.color_rgb, *outs, led)
        cl.enqueue_nd_range_kernel(self.queue, kernel, _padded(n), (LOCAL_SIZE,))
        host = [np.empty((n, 4), f32) for _ in range(3)] + [np.empty(n, f32)]
        for array, buf in zip(host, [*outs, led]):
            cl.enqueue_copy(self.queue, array, buf)
        return host[0][:, :3], host[1][:, :3], host[2][:, :3], host[3]

    def run(self, dist: Disturbances, config: SimConfig | None = None) -> SimResult:
        return self.run_batch([dist], config)[0]

    def run_batch(self, dists: list[Disturbances], config: SimConfig | None = None) -> list[SimResult]:
        """Simulate every scenario in `dists` together; one SimResult per scenario, in order.

        Each result reports the batch's wall time divided by the batch size.
        Recording (`config.record_hz`) needs a single scenario.
        """
        config = config or SimConfig()
        runs, n = len(dists), self.n
        rn = runs * n
        if runs == 0:
            return []
        if config.record_hz > 0 and runs != 1:
            raise ValueError("record_hz needs a single scenario")
        max_modes = max(1, max(d.mode_amp.shape[0] for d in dists))
        kernels = self._program(runs, float(config.dt), max_modes)
        q, f32, i32 = self.queue, np.float32, np.int32

        def per_run(attr: str) -> np.ndarray:
            return np.concatenate([np.asarray(getattr(d, attr), np.float64) for d in dists]).astype(f32)

        def zeros(count: int, dtype=f32, width: int = 1) -> cl.Buffer:
            return self._buffer(np.zeros((count, width) if width > 1 else count, dtype))

        def full(value, dtype=f32) -> cl.Buffer:
            return self._buffer(np.full(rn, value, dtype))

        seeds = self._buffer(np.array([int(d.seed) & 0x3FFFFFFF for d in dists], np.uint32), True)
        runp = self._buffer(self._run_params(dists, max_modes), True)
        mass, max_thrust, motor_tau, drag_cd, capacity, r_int = (
            self._buffer(per_run(a), True) for a in
            ("mass_kg", "max_thrust_n", "motor_tau_s", "drag_cd", "capacity_ah", "internal_resistance_ohm"))

        ref_p, ref_v, ref_a = zeros(n, width=4), zeros(n, width=4), zeros(n, width=4)
        led = zeros(n)
        pos, vel, quat, omega = (zeros(rn, width=4) for _ in range(4))
        thrust, vel_int, gnss_bias = (zeros(rn, width=4) for _ in range(3))
        armed = zeros(rn, i32)
        total_thrust, hover_thrust = zeros(rn), zeros(rn)
        lift = self._buffer(np.ones((rn, 4), f32))
        downwash = zeros(rn)

        soc, soc_comp = self._buffer(per_run("initial_soc")), zeros(rn)
        v0 = open_circuit_voltage(self.profile, np.concatenate([d.initial_soc for d in dists])).astype(f32)
        voltage = self._buffer(v0)
        temp = self._buffer(np.repeat(np.array([d.ambient_c for d in dists], f32), n))
        low_timer, brownout = zeros(rn), zeros(rn, i32)
        max_track, min_volt = zeros(rn), self._buffer(v0)
        brown_t = full(-1.0)
        min_sep, min_sep_partner, min_sep_time = full(_NO_NEIGHBOUR), full(-1, i32), full(-1.0)

        t_cells = GRID_G ** 3
        counts = zeros(runs * t_cells, i32)
        starts = zeros(runs * (t_cells + 1), i32)
        keys, slots, sorted_ids = zeros(rn, i32), zeros(rn, i32), zeros(rn, i32)

        t0 = self.pw.start_time_sec
        duration = (self.pw.end_time_sec - t0) + config.tail_sec
        steps = int(math.ceil(duration / config.dt - 1e-9))

        # Arguments are bound once; the loop only updates the time / step scalars.
        k_ref = kernels["eval_reference"]
        k_ref.set_args(f32(0.0), self.offsets, self.span_t0, self.span_t1, self.span_coef, self.color_offsets,
                       self.color_t, self.color_rgb, ref_p, ref_v, ref_a, led)
        k_count = kernels["grid_count"]
        k_count.set_args(pos, counts, keys, slots)
        k_scan = kernels["grid_scan"]
        k_scan.set_args(counts, starts)
        k_scatter = kernels["grid_scatter"]
        k_scatter.set_args(keys, slots, starts, sorted_ids)
        k_sort = kernels["grid_sort_cells"]
        k_sort.set_args(starts, sorted_ids)
        k_aero = kernels["interaction"]
        k_aero.set_args(f32(0.0), pos, quat, total_thrust, hover_thrust, starts, sorted_ids,
                        lift, downwash, min_sep, min_sep_partner, min_sep_time)
        k_step = kernels["step_dynamics"]
        k_step.set_args(f32(0.0), i32(0), seeds, runp, ref_p, ref_v, ref_a, led, lift, downwash,
                        mass, max_thrust, motor_tau, drag_cd, capacity, r_int,
                        pos, vel, quat, omega, thrust, vel_int, gnss_bias, armed, total_thrust,
                        soc, soc_comp, temp, voltage, low_timer, brownout, max_track, min_volt, brown_t)
        k_init = kernels["init_state"]
        k_init.set_args(seeds, runp, ref_p, mass, pos, vel, quat, omega, thrust, total_thrust,
                        hover_thrust, armed, vel_int, gnss_bias)

        g_n, g_rn, g_cells = _padded(n), _padded(rn), _padded(runs * t_cells)
        local = (LOCAL_SIZE,)
        g_scan, local_scan = (runs * SCAN_WG,), (SCAN_WG,)
        enqueue = cl.enqueue_nd_range_kernel

        enqueue(q, k_ref, g_n, local)
        enqueue(q, k_init, g_rn, local)

        record_every = int(round(1.0 / (config.record_hz * config.dt))) if config.record_hz > 0 else 0
        rec_t, rec_p = [], []
        host_pos = np.empty((rn, 4), f32)

        q.finish()
        wall_start = time.perf_counter()
        for k in range(steps):
            t_rel = f32(k * config.dt)
            k_ref.set_arg(0, t_rel)
            enqueue(q, k_ref, g_n, local)
            enqueue(q, k_count, g_rn, local)
            enqueue(q, k_scan, g_scan, local_scan)
            enqueue(q, k_scatter, g_rn, local)
            enqueue(q, k_sort, g_cells, local)
            k_aero.set_arg(0, t_rel)
            enqueue(q, k_aero, g_rn, local)
            k_step.set_arg(0, t_rel)
            k_step.set_arg(1, i32(k))
            enqueue(q, k_step, g_rn, local)
            if record_every and k % record_every == 0:
                cl.enqueue_copy(q, host_pos, pos)
                rec_t.append(k * config.dt + config.dt)
                rec_p.append(host_pos[:, :3].astype(np.float64))
            elif k % 64 == 63:
                q.flush()
        q.finish()
        wall = time.perf_counter() - wall_start

        def read(buf: cl.Buffer, dtype=f32, width: int = 1) -> np.ndarray:
            out = np.empty((rn, width) if width > 1 else rn, dtype)
            cl.enqueue_copy(q, out, buf)
            return out

        sep = read(min_sep).astype(np.float64)
        sep[sep >= _NO_NEIGHBOUR * 0.5] = np.inf
        partner, sep_t = read(min_sep_partner, i32), read(min_sep_time)
        track, soc_f, vmin, temp_f = read(max_track), read(soc), read(min_volt), read(temp)
        brown, brown_time, final_pos = read(brownout, i32), read(brown_t), read(pos, width=4)

        results = []
        for r in range(runs):
            s = slice(r * n, (r + 1) * n)
            results.append(SimResult(
                duration_sec=steps * config.dt,
                wall_time_sec=wall / runs,
                steps=steps,
                min_separation_m=sep[s],
                min_separation_partner=partner[s].astype(np.int64),
                min_separation_time=sep_t[s].astype(np.float64) + t0,
                max_tracking_error_m=track[s].astype(np.float64),
                final_soc=soc_f[s].astype(np.float64),
                min_voltage_v=vmin[s].astype(np.float64),
                final_temp_c=temp_f[s].astype(np.float64),
                brownout=brown[s].astype(bool),
                brownout_time=np.where(brown[s] > 0, brown_time[s].astype(np.float64) + t0, np.nan),
                final_position=final_pos[s, :3].astype(np.float64),
                recorded_times=np.asarray(rec_t) + t0,
                recorded_positions=np.asarray(rec_p) if rec_p else np.zeros((0, n, 3)),
            ))
        return results
