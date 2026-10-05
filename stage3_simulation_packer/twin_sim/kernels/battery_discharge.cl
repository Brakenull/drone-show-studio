// Electrochemical battery model; model in physics.py.

__constant float OCV_SOC[OCV_N] = { OCV_SOC_LIST };
__constant float OCV_VOLTS[OCV_N] = { OCV_VOLTS_LIST };

inline float ocv_cell(float soc) {
    float s = clamp(soc, OCV_SOC[0], OCV_SOC[OCV_N - 1]);
    int k = 0;
    while (k < OCV_N - 2 && s > OCV_SOC[k + 1]) k++;
    float frac = (s - OCV_SOC[k]) / (OCV_SOC[k + 1] - OCV_SOC[k]);
    return OCV_VOLTS[k] + (OCV_VOLTS[k + 1] - OCV_VOLTS[k]) * frac;
}

inline float shaft_power(float4 thrust) {
    float t4[4] = {thrust.x, thrust.y, thrust.z, thrust.w};
    float p = 0.0f;
    for (int k = 0; k < 4; ++k) {
        float t_k = fmax(t4[k], 0.0f);
        float omega = BATT_MAX_ROTOR_SPEED * sqrt(t_k / BATT_NOMINAL_MAX_THRUST);
        p += BATT_TORQUE_TO_THRUST * t_k * omega;
    }
    return p;
}

// Advance one step. `soc` / `soc_comp` are a Kahan-compensated float32 sum;
// `brownout` latches once V <= cutoff has lasted BROWNOUT_HOLD_S.
inline void battery_step(float4 thrust, float led_frac, float capacity_ah, float r25, float ambient_c,
                         float* soc, float* soc_comp, float* temp_c, float* voltage, float* low_timer,
                         int* brownout) {
    float v_prev = fmax(*voltage, 1.0f);
    float current = shaft_power(thrust) / (BATT_EFFICIENCY * v_prev)
                  + BATT_LED_MAX_POWER * led_frac / v_prev + BATT_AVIONICS_A;

    float i_1c = capacity_ah;
    float i_eff = current * pow(fmax(current, 1e-6f) / i_1c, BATT_PEUKERT - 1.0f);
    float soc_drop = i_eff * DT / (3600.0f * capacity_ah);
    float y = -soc_drop - *soc_comp;
    float s_new = *soc + y;
    *soc_comp = (s_new - *soc) - y;
    *soc = s_new;
    if (*soc < 0.0f) { *soc = 0.0f; *soc_comp = 0.0f; }

    float temp = *temp_c;
    float temp_k = temp + 273.15f;
    float r_int = r25 * exp(ARRHENIUS_B_K * (1.0f / temp_k - 1.0f / 298.15f));
    float heat = current * current * r_int - THERMAL_CONDUCTANCE_W_PER_K * (temp - ambient_c);
    *temp_c = temp + heat * DT / THERMAL_MASS_J_PER_K;

    float v_new = BATT_CELLS * ocv_cell(*soc) - current * r_int;
    *low_timer = v_new <= BATT_CUTOFF_V ? *low_timer + DT : 0.0f;
    if (*low_timer > BROWNOUT_HOLD_S) *brownout = 1;
    *voltage = v_new;
}
