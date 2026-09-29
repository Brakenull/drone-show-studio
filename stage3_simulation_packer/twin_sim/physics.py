"""Physical model constants and host-side helpers (docs/3-phase-3.md §3.1-§3.3).

The kernels in `kernels/*.cl` implement the models below; every constant they
use is defined here once and injected into the OpenCL program as a #define
(`kernel_defines()`), so Python and the device code cannot drift apart.

6-DOF dynamics + onboard tracking controller (§3.1), `multi_agent_dynamics.cl`
-----------------------------------------------------------------------------
One work-item per drone. Frame conventions: world ENU (z up), body FLU, unit
quaternion `q` rotates body -> world. §3.1 writes the translational equation
in the NED sign convention; in ENU it reads

    m p'' = -m g e3 + f R e3 + F_aero
    J w'  = -w x (J w) + tau

with F_aero = -1/2 rho Cd A |v - v_air| (v - v_air); v_air = wind + downwash,
i.e. §3.1's F_drag and F_wind are the same aerodynamic term evaluated on
relative airspeed, and F_downwash enters twice: as air velocity here and as
per-motor lift loss (below).

Controller (runs every physics step, 200 Hz):

* position P -> velocity PI (cascade), velocity feedforward v_ref and
  acceleration feedforward a_ref straight from the Phase 2 B-spline
  derivatives (no phase lag from differentiating setpoints); the velocity-loop
  integrator is the standard flight-stack addition that trims out mass /
  thrust tolerances and steady downwash;
* geometric attitude from the desired thrust vector (yaw held at 0),
  attitude P -> body-rate P, torques through the X-quad mixer;
* first-order motor lag, thrust capped by battery voltage sag.

Uses *nominal* profile values for the controller model and the per-drone
Monte Carlo *actual* values for the plant, so tolerances show up as tracking
error exactly as they would in the field.

Wind field + fast downwash proxy (§3.2), `aerodynamics.cl`
----------------------------------------------------------
Downwash below drone j (semi-analytic wake cone):

    v_dw(r, z) = v0 * (R0 / R(z))^2 * exp(-r^2 / R(z)^2),   R(z) = R0 + k_wake * z

* z  -- depth below j's rotor plane (only z > 0 contributes)
* r  -- radial distance from j's (vertical) wake axis
* R0 -- wake radius at the rotor plane: arm length + propeller radius
* v0 -- j's momentum-theory induced velocity, sqrt(T_j / (2 rho A_disk)), from
        j's *current* total thrust, so a hard-working drone casts a stronger wake

Effect on a drone i underneath, evaluated separately at each of i's four motor
positions (so a wake edge crossing one side of i produces the asymmetric
tipping torque §3.2 describes, with no extra torque model):

* lift loss per motor: T_eff = T * v_h / (v_h + w), v_h = i's own hover
  induced velocity -- the extra axial inflow w eats into the prop's angle of
  attack; 30 % loss for a drone 1 m under another, ~13 % at 3 m
* the wake at i's center also enters i's drag as a downward air velocity

Wind: mean wind + a sum of travelling sinusoidal turbulence modes (spatially
and temporally correlated, so neighbours feel similar gusts) + one discrete
(1 - cos) gust front that sweeps across the formation at the mean-wind speed.

The same neighbour pass records each drone's nearest-neighbour distance for
the §3.4 safety check (d_crash / buffer warning).

Electrochemical battery model (§3.3), `battery_discharge.cl`
------------------------------------------------------------
Load current:

    I = sum_k(Q_k * w_k) / (eta * V) + P_LED / V + I_avionics

where Q_k * w_k is motor k's shaft power, written in the §3.3 form T_k * w_k
scaled by the rotor torque/thrust ratio c_q (Q_k = c_q * T_k), and the prop
speed follows the quadratic thrust law w_k = w_max * sqrt(T_k / T_max).

Terminal voltage and state:

    V     = S * OCV(SOC) - I * R_int(T)
    dSOC  = -I_eff * dt / (3600 * C_Ah),   I_eff = I * (I / I_1C)^(k_peukert - 1)
    R_int(T) = R_25C * exp(B * (1/T - 1/298.15))        (Arrhenius, B = 2500 K)
    C_th dT/dt = I^2 R_int - h * (T - T_ambient)

SOC is accumulated with Kahan compensation in float32: a plain float32 sum
loses about 20 % of a small constant draw to rounding at 200 Hz, and the
target GPUs (Intel Iris Xe) have no float64.

Brownout Risk (§3.3): V <= cutoff for more than 2.0 s continuously; latched.
"""

from __future__ import annotations

import math

import numpy as np

# ---- environment / dynamics ----
RHO_AIR = 1.225                 # kg/m^3
GRAVITY = 9.80665               # m/s^2

# ---- controller ----
VEL_INTEGRAL_LIMIT = 3.0        # m/s^2 per axis
MAX_RATE_RP = 6.0               # rad/s roll/pitch command limit
MAX_RATE_YAW = 3.0              # rad/s
ARM_HEIGHT_M = 0.05             # reference this far above ground -> arm
ARM_SPEED_MPS = 0.05

# ---- downwash ----
K_WAKE = 0.12                   # §3.2 wake-cone expansion rate
WAKE_CUTOFF_FRACTION = 0.05     # ignore wake once centre-line speed < 5 % of v0
WAKE_RADIAL_CUTOFF = 2.5        # exp(-2.5^2) < 0.2 %: ignore wake beyond 2.5 R(z) off-axis

# ---- battery ----
# Resting LiPo open-circuit voltage per cell vs state of charge.
OCV_SOC_TABLE = np.array([0.00, 0.05, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00], np.float32)
OCV_CELL_VOLTS = np.array([3.27, 3.61, 3.69, 3.73, 3.77, 3.79, 3.82, 3.87, 3.92, 3.98, 4.06, 4.20], np.float32)
ARRHENIUS_B_K = 2500.0
THERMAL_MASS_J_PER_K = 180.0          # ~200 g 4S 2200 mAh pack
THERMAL_CONDUCTANCE_W_PER_K = 0.6     # pack in prop wash
BROWNOUT_HOLD_S = 2.0
KV_LOAD_FACTOR = 0.75                 # loaded prop speed vs no-load kv * V


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


def max_rotor_speed_rad_s(kv_rating: float, nominal_voltage_v: float) -> float:
    return kv_rating * nominal_voltage_v * 2.0 * math.pi / 60.0 * KV_LOAD_FACTOR


def open_circuit_voltage(cells: float, soc: np.ndarray) -> np.ndarray:
    return cells * np.interp(soc, OCV_SOC_TABLE, OCV_CELL_VOLTS)


def kernel_defines() -> dict[str, float]:
    """Model constants shared with kernels/*.cl (injected as #defines)."""
    names = ("RHO_AIR", "GRAVITY", "VEL_INTEGRAL_LIMIT", "MAX_RATE_RP", "MAX_RATE_YAW", "ARM_HEIGHT_M",
             "ARM_SPEED_MPS", "K_WAKE", "WAKE_RADIAL_CUTOFF", "ARRHENIUS_B_K", "THERMAL_MASS_J_PER_K",
             "THERMAL_CONDUCTANCE_W_PER_K", "BROWNOUT_HOLD_S")
    return {name: globals()[name] for name in names}
