"""Parallel 6-DOF multi-agent dynamics + onboard tracking controller (docs/3-phase-3.md §3.1).

One Warp thread per drone. Frame conventions: world ENU (z up), body FLU,
unit quaternion `q` rotates body -> world. §3.1 writes the translational
equation in the NED sign convention; in ENU it reads

    m p'' = -m g e3 + f R e3 + F_aero
    J w'  = -w x (J w) + tau

with F_aero = -1/2 rho Cd A |v - v_air| (v - v_air); v_air = wind + downwash,
i.e. §3.1's F_drag and F_wind are the same aerodynamic term evaluated on
relative airspeed, and F_downwash enters twice: as air velocity here and as
per-motor lift loss (aerodynamics.py).

Controller (runs every physics step, 200 Hz):

* position P -> velocity PI (cascade), velocity feedforward v_ref and
  acceleration feedforward a_ref straight from the Phase 2 B-spline
  derivatives (no phase lag from differentiating setpoints);
  the velocity-loop integrator is the standard flight-stack addition that
  trims out mass / thrust tolerances and steady downwash;
* geometric attitude from the desired thrust vector (yaw held at 0),
  attitude P -> body-rate P, torques through the X-quad mixer;
* first-order motor lag, thrust capped by battery voltage sag.

Uses *nominal* profile values for the controller model and the per-drone
Monte Carlo *actual* values for the plant, so tolerances show up as tracking
error exactly as they would in the field.
"""

from __future__ import annotations

import warp as wp

from .aerodynamics import RHO_AIR, WindField, motor_offset, wind_at
from .battery_discharge import BatteryParams, battery_step

GRAVITY = wp.constant(9.80665)
VEL_INTEGRAL_LIMIT = wp.constant(3.0)     # m/s^2 per axis
MAX_RATE_RP = wp.constant(6.0)            # rad/s roll/pitch command limit
MAX_RATE_YAW = wp.constant(3.0)           # rad/s
ARM_HEIGHT_M = wp.constant(0.05)          # reference this far above ground -> arm
ARM_SPEED_MPS = wp.constant(0.05)


@wp.struct
class ControllerParams:
    mass_nominal: float
    inertia: wp.vec3
    half_diag: float              # arm_length / sqrt(2): motor x/y offset
    torque_to_thrust: float       # c_q (m)
    max_thrust_nominal: float     # N per motor
    nominal_voltage: float
    pos_p: wp.vec3
    vel_p: wp.vec3
    vel_i: wp.vec3
    att_p: wp.vec3
    rate_p: wp.vec3
    tan_max_tilt: float
    drag_area: float              # reference_area_m2
    gnss_noise: float             # white RTK noise, m (1 sigma)
    gnss_drift: float             # OU drift diffusion, m/sqrt(s)
    gnss_corr_time: float         # OU correlation time, s


@wp.func
def clamp_vec(v: wp.vec3, limit: wp.vec3) -> wp.vec3:
    return wp.vec3(wp.clamp(v[0], -limit[0], limit[0]), wp.clamp(v[1], -limit[1], limit[1]),
                   wp.clamp(v[2], -limit[2], limit[2]))


@wp.func
def attitude_from_thrust(f_des: wp.vec3) -> wp.quat:
    b3 = wp.normalize(f_des)
    b2 = wp.normalize(wp.cross(b3, wp.vec3(1.0, 0.0, 0.0)))
    b1 = wp.cross(b2, b3)
    return wp.quat_from_matrix(wp.matrix_from_cols(b1, b2, b3))


@wp.func
def mix(f: float, tau: wp.vec3, half_diag: float, c_q: float) -> wp.vec4:
    # Inverse of: f = sum T, tau_x = sum y_m T, tau_y = -sum x_m T, tau_z = c_q sum s_m T,
    # motors (x, y, s): (+d,+d,+1) (-d,+d,-1) (-d,-d,+1) (+d,-d,-1).
    a = f * 0.25
    bx = tau[0] / (4.0 * half_diag)
    by = tau[1] / (4.0 * half_diag)
    bz = tau[2] / (4.0 * c_q)
    return wp.vec4(a + bx - by + bz, a + bx + by - bz, a - bx + by + bz, a - bx - by - bz)


@wp.func
def unmix(t: wp.vec4, half_diag: float, c_q: float) -> wp.vec4:
    """Returns (f, tau_x, tau_y, tau_z) produced by motor thrusts t."""
    d = half_diag
    return wp.vec4(t[0] + t[1] + t[2] + t[3],
                   d * (t[0] + t[1] - t[2] - t[3]),
                   d * (-t[0] + t[1] + t[2] - t[3]),
                   c_q * (t[0] - t[1] + t[2] - t[3]))


@wp.kernel
def init_state_kernel(ref_p: wp.array(dtype=wp.vec3), ground_z: float, init_err: float, seed: int,
                      mass: wp.array(dtype=float),
                      pos: wp.array(dtype=wp.vec3), vel: wp.array(dtype=wp.vec3), quat: wp.array(dtype=wp.quat),
                      omega: wp.array(dtype=wp.vec3), thrust: wp.array(dtype=wp.vec4),
                      total_thrust: wp.array(dtype=float), hover_thrust: wp.array(dtype=float),
                      armed: wp.array(dtype=int)):
    d = wp.tid()
    rng = wp.rand_init(seed, d)
    p = ref_p[d] + wp.vec3(wp.randn(rng), wp.randn(rng), 0.0) * init_err
    airborne = p[2] > ground_z + ARM_HEIGHT_M
    if not airborne:
        p = wp.vec3(p[0], p[1], ground_z)
    pos[d] = p
    vel[d] = wp.vec3(0.0, 0.0, 0.0)
    quat[d] = wp.quat_identity()
    omega[d] = wp.vec3(0.0, 0.0, 0.0)
    hover = mass[d] * GRAVITY
    hover_thrust[d] = hover
    if airborne:
        thrust[d] = wp.vec4(hover, hover, hover, hover) * 0.25
        total_thrust[d] = hover
        armed[d] = 1
    else:
        thrust[d] = wp.vec4(0.0, 0.0, 0.0, 0.0)
        total_thrust[d] = 0.0
        armed[d] = 0


@wp.kernel
def step_kernel(
    t: float, dt: float, step: int, seed: int, ground_z: float,
    ctrl: ControllerParams, wind: WindField, battery: BatteryParams,
    # references (spline_evaluator.eval_reference_kernel)
    ref_p: wp.array(dtype=wp.vec3), ref_v: wp.array(dtype=wp.vec3), ref_a: wp.array(dtype=wp.vec3),
    led_frac: wp.array(dtype=float),
    # interactions (aerodynamics.interaction_kernel)
    lift_factor: wp.array(dtype=wp.vec4), downwash_air: wp.array(dtype=wp.vec3),
    # per-drone Monte Carlo plant parameters
    mass: wp.array(dtype=float), max_thrust: wp.array(dtype=float), motor_tau: wp.array(dtype=float),
    drag_cd: wp.array(dtype=float), capacity_ah: wp.array(dtype=float), r_int: wp.array(dtype=float),
    # state (in/out)
    pos: wp.array(dtype=wp.vec3), vel: wp.array(dtype=wp.vec3), quat: wp.array(dtype=wp.quat),
    omega: wp.array(dtype=wp.vec3), thrust: wp.array(dtype=wp.vec4), vel_int: wp.array(dtype=wp.vec3),
    gnss_bias: wp.array(dtype=wp.vec3), armed: wp.array(dtype=int), total_thrust: wp.array(dtype=float),
    soc: wp.array(dtype=wp.float64), temp_c: wp.array(dtype=float), voltage: wp.array(dtype=float),
    low_timer: wp.array(dtype=float), brownout: wp.array(dtype=int),
    # running metrics (in/out)
    max_track_err: wp.array(dtype=float), min_voltage: wp.array(dtype=float),
    brownout_time: wp.array(dtype=float),
):
    d = wp.tid()
    p = pos[d]
    v = vel[d]
    q = quat[d]
    w = omega[d]
    rp = ref_p[d]
    rv = ref_v[d]

    # ---- sensing: RTK GNSS with Ornstein-Uhlenbeck drift + white noise ----
    rng = wp.rand_init(seed + step, d)
    bias = gnss_bias[d]
    decay = dt / ctrl.gnss_corr_time
    diffusion = ctrl.gnss_drift * wp.sqrt(dt)
    bias = bias * (1.0 - decay) + wp.vec3(wp.randn(rng), wp.randn(rng), wp.randn(rng)) * diffusion
    gnss_bias[d] = bias
    p_meas = p + bias + wp.vec3(wp.randn(rng), wp.randn(rng), wp.randn(rng)) * ctrl.gnss_noise

    # ---- arming: motors spin up once the reference leaves the pad ----
    is_armed = armed[d]
    if is_armed == 0 and (rp[2] > ground_z + ARM_HEIGHT_M or wp.length(rv) > ARM_SPEED_MPS):
        is_armed = 1
    on_ground = p[2] <= ground_z + 1.0e-3
    if is_armed == 1 and on_ground and rp[2] <= ground_z + 0.02 and wp.length(rv) < 0.01:
        is_armed = 0  # landed, reference parked on the pad
    armed[d] = is_armed

    # ---- battery-limited motor ceiling ----
    v_ratio = wp.clamp(voltage[d] / ctrl.nominal_voltage, 0.5, 1.2)
    t_max = max_thrust[d] * v_ratio * v_ratio

    t_cmd = wp.vec4(0.0, 0.0, 0.0, 0.0)
    if is_armed == 1:
        # ---- position -> velocity PI -> desired force ----
        v_cmd = rv + wp.cw_mul(ctrl.pos_p, rp - p_meas)
        e_v = v_cmd - v
        vi = clamp_vec(vel_int[d] + wp.cw_mul(ctrl.vel_i, e_v) * dt,
                       wp.vec3(VEL_INTEGRAL_LIMIT, VEL_INTEGRAL_LIMIT, VEL_INTEGRAL_LIMIT))
        vel_int[d] = vi
        a_cmd = ref_a[d] + wp.cw_mul(ctrl.vel_p, e_v) + vi
        f_des = (a_cmd + wp.vec3(0.0, 0.0, GRAVITY)) * ctrl.mass_nominal
        fz = wp.max(f_des[2], 0.1 * ctrl.mass_nominal * GRAVITY)
        h = wp.vec3(f_des[0], f_des[1], 0.0)
        h_len = wp.length(h)
        h_max = fz * ctrl.tan_max_tilt
        if h_len > h_max:
            h = h * (h_max / h_len)
        f_des = wp.vec3(h[0], h[1], fz)

        body_z = wp.quat_rotate(q, wp.vec3(0.0, 0.0, 1.0))
        f_coll = wp.clamp(wp.dot(f_des, body_z), 0.0, 4.0 * ctrl.max_thrust_nominal)

        # ---- attitude P -> rate P -> torque ----
        q_des = attitude_from_thrust(f_des)
        q_err = wp.mul(wp.quat_inverse(q), q_des)
        if q_err[3] < 0.0:
            q_err = q_err * -1.0
        e_att = wp.vec3(q_err[0], q_err[1], q_err[2]) * 2.0
        w_cmd = clamp_vec(wp.cw_mul(ctrl.att_p, e_att), wp.vec3(MAX_RATE_RP, MAX_RATE_RP, MAX_RATE_YAW))
        jw = wp.cw_mul(ctrl.inertia, w)
        tau_cmd = wp.cw_mul(ctrl.inertia, wp.cw_mul(ctrl.rate_p, w_cmd - w)) + wp.cross(w, jw)
        t_cmd = mix(f_coll, tau_cmd, ctrl.half_diag, ctrl.torque_to_thrust)
        t_cmd = wp.vec4(wp.clamp(t_cmd[0], 0.0, t_max), wp.clamp(t_cmd[1], 0.0, t_max),
                        wp.clamp(t_cmd[2], 0.0, t_max), wp.clamp(t_cmd[3], 0.0, t_max))
    else:
        vel_int[d] = wp.vec3(0.0, 0.0, 0.0)

    # ---- motor lag ----
    alpha = 1.0 - wp.exp(-dt / motor_tau[d])
    t_now = thrust[d] + (t_cmd - thrust[d]) * alpha
    thrust[d] = t_now

    # ---- plant: forces and torques from actual (downwash-derated) thrust ----
    lf = lift_factor[d]
    t_eff = wp.vec4(t_now[0] * lf[0], t_now[1] * lf[1], t_now[2] * lf[2], t_now[3] * lf[3])
    wrench = unmix(t_eff, ctrl.half_diag, ctrl.torque_to_thrust)
    total_thrust[d] = wrench[0]

    m = mass[d]
    v_air = wind_at(wind, p, t) + downwash_air[d]
    v_rel = v - v_air
    f_drag = v_rel * (-0.5 * RHO_AIR * drag_cd[d] * ctrl.drag_area * wp.length(v_rel))
    f_thrust = wp.quat_rotate(q, wp.vec3(0.0, 0.0, wrench[0]))
    acc = (f_thrust + f_drag) / m - wp.vec3(0.0, 0.0, GRAVITY)

    tau = wp.vec3(wrench[1], wrench[2], wrench[3])
    jw = wp.cw_mul(ctrl.inertia, w)
    w_dot = wp.cw_div(tau - wp.cross(w, jw), ctrl.inertia)

    # ---- semi-implicit Euler ----
    v = v + acc * dt
    p = p + v * dt
    w = w + w_dot * dt
    q = wp.normalize(q + wp.mul(q, wp.quat(w[0], w[1], w[2], 0.0)) * (0.5 * dt))

    # ---- ground contact ----
    if p[2] < ground_z:
        p = wp.vec3(p[0], p[1], ground_z)
        v = wp.vec3(0.0, 0.0, wp.max(v[2], 0.0))
        if is_armed == 0 or f_thrust[2] < m * GRAVITY:
            q = wp.quat_identity()
            w = wp.vec3(0.0, 0.0, 0.0)

    pos[d] = p
    vel[d] = v
    quat[d] = q
    omega[d] = w

    # ---- battery ----
    soc_drop, new_temp, new_v, current, timer, flag = battery_step(
        battery, dt, t_now, led_frac[d], capacity_ah[d], r_int[d], float(soc[d]), temp_c[d], voltage[d],
        low_timer[d], brownout[d])
    if flag == 1 and brownout[d] == 0:
        brownout_time[d] = t
    soc[d] = wp.max(soc[d] - wp.float64(soc_drop), wp.float64(0.0))
    temp_c[d] = new_temp
    voltage[d] = new_v
    low_timer[d] = timer
    brownout[d] = flag
    min_voltage[d] = wp.min(min_voltage[d], new_v)

    if is_armed == 1:
        max_track_err[d] = wp.max(max_track_err[d], wp.length(p - rp))
