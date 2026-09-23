"""Wind field + fast downwash proxy (docs/3-phase-3.md §3.2).

Downwash below drone j (semi-analytic wake cone):

    v_dw(r, z) = v0 * (R0 / R(z))^2 * exp(-r^2 / R(z)^2),   R(z) = R0 + k_wake * z

* z  -- depth below j's rotor plane (only z > 0 contributes)
* r  -- radial distance from j's (vertical) wake axis
* R0 -- wake radius at the rotor plane: arm length + propeller radius
* v0 -- j's momentum-theory induced velocity, sqrt(T_j / (2 rho A_disk)), from
        j's *current* total thrust, so a hard-working drone casts a stronger wake

Effect on a drone i underneath, evaluated separately at each of i's four
motor positions (so a wake edge crossing one side of i produces the
asymmetric tipping torque §3.2 describes, with no extra torque model):

* lift loss per motor: T_eff = T * v_h / (v_h + w), v_h = i's own hover
  induced velocity -- the extra axial inflow w eats into the prop's angle of
  attack; 30 % loss for a drone 1 m under another, ~13 % at 3 m
* the wake at i's center also enters i's drag as a downward air velocity

Wind: mean wind + a sum of travelling sinusoidal turbulence modes (spatially
and temporally correlated, so neighbours feel similar gusts) + one discrete
(1 - cos) gust front that sweeps across the formation at the mean-wind speed.

The same neighbour pass records each drone's nearest-neighbour distance for
the §3.4 safety check (d_crash / buffer warning).
"""

from __future__ import annotations

import math

import warp as wp

RHO_AIR = wp.constant(1.225)           # kg/m^3
K_WAKE = wp.constant(0.12)             # §3.2 wake-cone expansion rate
WAKE_CUTOFF_FRACTION = 0.05  # ignore wake once centre-line speed < 5 % of v0
WAKE_RADIAL_CUTOFF = wp.constant(2.5)  # exp(-2.5^2) < 0.2 %: ignore wake beyond 2.5 R(z) off-axis


def wake_radius_at_rotor(arm_length_m: float, propeller_diameter_m: float) -> float:
    return arm_length_m + 0.5 * propeller_diameter_m


def total_disk_area(propeller_diameter_m: float) -> float:
    return 4.0 * math.pi * (0.5 * propeller_diameter_m) ** 2


def wake_query_depth(r0: float) -> float:
    """Depth at which the centre-line wake speed decays to WAKE_CUTOFF_FRACTION of v0."""
    return r0 * (1.0 / math.sqrt(WAKE_CUTOFF_FRACTION) - 1.0) / K_WAKE


def wake_query_radius(r0: float, half_diag: float) -> float:
    """Radius of the sphere centred wake_depth/2 above a drone that encloses its wake-influence cone."""
    depth = wake_query_depth(r0)
    radial = WAKE_RADIAL_CUTOFF * (r0 + K_WAKE * depth + half_diag)
    return math.hypot(0.5 * depth + half_diag, radial)


@wp.struct
class WindField:
    mean: wp.vec3
    n_modes: int
    mode_amp: wp.array(dtype=wp.vec3)     # amplitude vector (m/s) per mode
    mode_k: wp.array(dtype=wp.vec3)       # spatial wave vector (rad/m)
    mode_omega: wp.array(dtype=float)     # temporal frequency (rad/s)
    mode_phase: wp.array(dtype=float)
    gust_vec: wp.vec3                     # peak discrete-gust velocity (m/s)
    gust_dir: wp.vec3                     # unit propagation direction (horizontal)
    gust_start: float                     # s, front passes the origin
    gust_duration: float                  # s
    gust_speed: float                     # m/s front propagation speed


@wp.func
def wind_at(wind: WindField, p: wp.vec3, t: float) -> wp.vec3:
    w = wind.mean
    for m in range(wind.n_modes):
        w = w + wind.mode_amp[m] * wp.sin(wp.dot(wind.mode_k[m], p) - wind.mode_omega[m] * t + wind.mode_phase[m])
    if wind.gust_duration > 0.0:
        local_t = t - wind.gust_start - wp.dot(p, wind.gust_dir) / wp.max(wind.gust_speed, 0.1)
        if local_t > 0.0 and local_t < wind.gust_duration:
            w = w + wind.gust_vec * (0.5 * (1.0 - wp.cos(2.0 * wp.pi * local_t / wind.gust_duration)))
    return w


@wp.func
def downwash_speed(r_sq: float, z: float, v0: float, r0: float) -> float:
    if z <= 0.0:
        return 0.0
    radius = r0 + K_WAKE * z
    ratio = r0 / radius
    return v0 * ratio * ratio * wp.exp(-r_sq / (radius * radius))


@wp.func
def xy_dist_sq(a: wp.vec3, b: wp.vec3) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return dx * dx + dy * dy


@wp.func
def motor_offset(m: int, half_diag: float) -> wp.vec3:
    # X configuration, motor 0 front-left then counter-clockwise (body FLU).
    sx = 1.0
    sy = 1.0
    if m == 1:
        sx = -1.0
    if m == 2:
        sx = -1.0
        sy = -1.0
    if m == 3:
        sy = -1.0
    return wp.vec3(sx * half_diag, sy * half_diag, 0.0)


@wp.kernel
def interaction_kernel(grid: wp.uint64, pos: wp.array(dtype=wp.vec3), quat: wp.array(dtype=wp.quat),
                       total_thrust: wp.array(dtype=float), hover_thrust: wp.array(dtype=float),
                       proximity_radius: float, wake_depth: float, wake_query_radius: float,
                       r0: float, half_diag: float, disk_area: float,
                       lift_factor: wp.array(dtype=wp.vec4), downwash_air: wp.array(dtype=wp.vec3),
                       nearest_dist: wp.array(dtype=float), nearest_id: wp.array(dtype=int)):
    i = wp.tid()
    p_i = pos[i]

    # ---- proximity (safety metric): small sphere around the drone ----
    best = float(1.0e9)
    best_id = int(-1)
    j = int(0)
    near = wp.hash_grid_query(grid, p_i, proximity_radius)
    while wp.hash_grid_query_next(near, j):
        if j != i:
            d = wp.length(pos[j] - p_i)
            if d < best:
                best = d
                best_id = j
    nearest_dist[i] = best
    nearest_id[i] = best_id

    # ---- downwash: only drones above, inside their wake cone ----
    # The query sphere is centred half a wake-depth above the drone, so it
    # covers the cone-shaped region that can shed wake onto it and not the
    # (useless) half-space below.
    q_i = quat[i]
    two_rho_a = 2.0 * RHO_AIR * disk_area
    v_hover_i = wp.sqrt(wp.max(hover_thrust[i], 1e-3) / two_rho_a)
    m0 = p_i + wp.quat_rotate(q_i, motor_offset(0, half_diag))
    m1 = p_i + wp.quat_rotate(q_i, motor_offset(1, half_diag))
    m2 = p_i + wp.quat_rotate(q_i, motor_offset(2, half_diag))
    m3 = p_i + wp.quat_rotate(q_i, motor_offset(3, half_diag))
    w0 = float(0.0)
    w1 = float(0.0)
    w2 = float(0.0)
    w3 = float(0.0)
    w_cg = float(0.0)

    centre = p_i + wp.vec3(0.0, 0.0, 0.5 * wake_depth)
    wake = wp.hash_grid_query(grid, centre, wake_query_radius)
    while wp.hash_grid_query_next(wake, j):
        if j != i:
            p_j = pos[j]
            dz = p_j[2] - p_i[2]
            if dz > -half_diag and dz < wake_depth + half_diag:
                radius = r0 + K_WAKE * wp.max(dz, 0.0) + half_diag
                dx = p_j[0] - p_i[0]
                dy = p_j[1] - p_i[1]
                reach = WAKE_RADIAL_CUTOFF * radius
                if dx * dx + dy * dy < reach * reach:
                    v0 = wp.sqrt(wp.max(total_thrust[j], 0.0) / two_rho_a)
                    w0 += downwash_speed(xy_dist_sq(m0, p_j), p_j[2] - m0[2], v0, r0)
                    w1 += downwash_speed(xy_dist_sq(m1, p_j), p_j[2] - m1[2], v0, r0)
                    w2 += downwash_speed(xy_dist_sq(m2, p_j), p_j[2] - m2[2], v0, r0)
                    w3 += downwash_speed(xy_dist_sq(m3, p_j), p_j[2] - m3[2], v0, r0)
                    w_cg += downwash_speed(dx * dx + dy * dy, dz, v0, r0)

    lift_factor[i] = wp.vec4(v_hover_i / (v_hover_i + w0), v_hover_i / (v_hover_i + w1),
                             v_hover_i / (v_hover_i + w2), v_hover_i / (v_hover_i + w3))
    downwash_air[i] = wp.vec3(0.0, 0.0, -w_cg)
