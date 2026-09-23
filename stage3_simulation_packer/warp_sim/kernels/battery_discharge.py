"""Electrochemical battery model (docs/3-phase-3.md §3.3).

Load current:

    I = sum_k(Q_k * w_k) / (eta * V) + P_LED / V + I_avionics

where Q_k * w_k is motor k's shaft power, written in the §3.3 form
T_k * w_k scaled by the rotor torque/thrust ratio c_q (Q_k = c_q * T_k), and
the prop speed follows the quadratic thrust law w_k = w_max * sqrt(T_k / T_max).

Terminal voltage and state:

    V     = S * OCV(SOC) - I * R_int(T)
    dSOC  = -I_eff * dt / (3600 * C_Ah),   I_eff = I * (I / I_1C)^(k_peukert - 1)
    R_int(T) = R_25C * exp(B * (1/T - 1/298.15))        (Arrhenius, B = 2500 K)
    C_th dT/dt = I^2 R_int - h * (T - T_ambient)

Brownout Risk (§3.3): V <= cutoff for more than 2.0 s continuously; latched.
"""

from __future__ import annotations

import math

import numpy as np
import warp as wp

# Resting LiPo open-circuit voltage per cell vs state of charge.
OCV_SOC_TABLE = np.array([0.00, 0.05, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00], np.float32)
OCV_CELL_VOLTS = np.array([3.27, 3.61, 3.69, 3.73, 3.77, 3.79, 3.82, 3.87, 3.92, 3.98, 4.06, 4.20], np.float32)

ARRHENIUS_B_K = wp.constant(2500.0)
THERMAL_MASS_J_PER_K = wp.constant(180.0)      # ~200 g 4S 2200 mAh pack
THERMAL_CONDUCTANCE_W_PER_K = wp.constant(0.6)  # pack in prop wash
BROWNOUT_HOLD_S = wp.constant(2.0)
KV_LOAD_FACTOR = 0.75             # loaded prop speed vs no-load kv * V


def max_rotor_speed_rad_s(kv_rating: float, nominal_voltage_v: float) -> float:
    return kv_rating * nominal_voltage_v * 2.0 * math.pi / 60.0 * KV_LOAD_FACTOR


@wp.struct
class BatteryParams:
    cells: float
    cutoff_v: float
    efficiency: float
    peukert: float
    led_max_power_w: float
    avionics_a: float
    rotor_torque_to_thrust_m: float
    max_rotor_speed: float        # rad/s at max thrust
    nominal_max_thrust: float     # N per motor, for w_k = w_max * sqrt(T/T_max)
    ambient_c: float
    ocv_soc: wp.array(dtype=float)
    ocv_volts: wp.array(dtype=float)


@wp.func
def ocv_cell(params: BatteryParams, soc: float) -> float:
    n = params.ocv_soc.shape[0]
    s = wp.clamp(soc, params.ocv_soc[0], params.ocv_soc[n - 1])
    k = int(0)
    while k < n - 2 and s > params.ocv_soc[k + 1]:
        k += 1
    frac = (s - params.ocv_soc[k]) / (params.ocv_soc[k + 1] - params.ocv_soc[k])
    return params.ocv_volts[k] + (params.ocv_volts[k + 1] - params.ocv_volts[k]) * frac


@wp.func
def shaft_power(params: BatteryParams, thrust: wp.vec4) -> float:
    p = float(0.0)
    for k in range(4):
        t_k = wp.max(thrust[k], 0.0)
        omega = params.max_rotor_speed * wp.sqrt(t_k / params.nominal_max_thrust)
        p += params.rotor_torque_to_thrust_m * t_k * omega
    return p


@wp.func
def battery_step(params: BatteryParams, dt: float, thrust: wp.vec4, led_frac: float, capacity_ah: float,
                 r25: float, soc: float, temp_c: float, voltage: float, low_timer: float, brownout: int):
    """Advance one step; returns (soc_drop, temp_c, voltage, current, low_timer, brownout).

    The caller accumulates `soc_drop` in float64: at 200 Hz a small constant
    draw is only a few float32 ulps of SOC per step, and rounding would
    systematically lose a double-digit percentage of the consumed charge.
    """
    v_prev = wp.max(voltage, 1.0)
    current = shaft_power(params, thrust) / (params.efficiency * v_prev) \
        + params.led_max_power_w * led_frac / v_prev + params.avionics_a

    i_1c = capacity_ah
    i_eff = current * wp.pow(wp.max(current, 1e-6) / i_1c, params.peukert - 1.0)
    soc_drop = i_eff * dt / (3600.0 * capacity_ah)
    soc_new = wp.max(soc - soc_drop, 0.0)

    temp_k = temp_c + 273.15
    r_int = r25 * wp.exp(ARRHENIUS_B_K * (1.0 / temp_k - 1.0 / 298.15))
    heat = current * current * r_int - THERMAL_CONDUCTANCE_W_PER_K * (temp_c - params.ambient_c)
    temp_new = temp_c + heat * dt / THERMAL_MASS_J_PER_K

    v_new = params.cells * ocv_cell(params, soc_new) - current * r_int
    timer = float(0.0)
    if v_new <= params.cutoff_v:
        timer = low_timer + dt
    flag = brownout
    if timer > BROWNOUT_HOLD_S:
        flag = 1
    return soc_drop, temp_new, v_new, current, timer, flag
