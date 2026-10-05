// Parallel 6-DOF multi-agent dynamics + onboard tracking controller;
// model in physics.py.

inline float4 motor_mix(float f, float3 tau) {
    // Inverse of: f = sum T, tau_x = sum y_m T, tau_y = -sum x_m T, tau_z = c_q sum s_m T,
    // motors (x, y, s): (+d,+d,+1) (-d,+d,-1) (-d,-d,+1) (+d,-d,-1).
    float a = f * 0.25f;
    float bx = tau.x / (4.0f * HALF_DIAG);
    float by = tau.y / (4.0f * HALF_DIAG);
    float bz = tau.z / (4.0f * TORQUE_TO_THRUST);
    return (float4)(a + bx - by + bz, a + bx + by - bz, a - bx + by + bz, a - bx - by - bz);
}

// (f, tau_x, tau_y, tau_z) produced by motor thrusts t.
inline float4 motor_unmix(float4 t) {
    const float d = HALF_DIAG;
    return (float4)(t.x + t.y + t.z + t.w,
                    d * (t.x + t.y - t.z - t.w),
                    d * (-t.x + t.y + t.z - t.w),
                    TORQUE_TO_THRUST * (t.x - t.y + t.z - t.w));
}

inline float4 attitude_from_thrust(float3 f_des) {
    float3 b3 = normalize(f_des);
    float3 b2 = normalize(cross(b3, (float3)(1.0f, 0.0f, 0.0f)));
    float3 b1 = cross(b2, b3);
    return quat_from_cols(b1, b2, b3);
}

__kernel void init_state(__global const uint* seeds, __global const float* runp, __global const float4* ref_p,
                         __global const float* mass,
                         __global float4* pos, __global float4* vel, __global float4* quat, __global float4* omega,
                         __global float4* thrust, __global float* total_thrust, __global float* hover_thrust,
                         __global int* armed, __global float4* vel_int, __global float4* gnss_bias) {
    int gid = get_global_id(0);
    if (gid >= RN) return;
    int r = gid / N_DRONES;
    int d = gid - r * N_DRONES;
    uint rng = rng_init(seeds[r], (uint)d);
    float init_err = runp[r * RUN_STRIDE + RP_INIT_ERR];
    float ex = randn(&rng);
    float ey = randn(&rng);
    float3 p = ref_p[d].xyz + (float3)(ex, ey, 0.0f) * init_err;
    bool airborne = p.z > GROUND_Z + ARM_HEIGHT_M;
    if (!airborne) p.z = GROUND_Z;
    pos[gid] = (float4)(p, 0.0f);
    vel[gid] = (float4)(0.0f);
    quat[gid] = (float4)(0.0f, 0.0f, 0.0f, 1.0f);
    omega[gid] = (float4)(0.0f);
    vel_int[gid] = (float4)(0.0f);
    gnss_bias[gid] = (float4)(0.0f);
    float hover = mass[gid] * GRAVITY;
    hover_thrust[gid] = hover;
    if (airborne) {
        thrust[gid] = (float4)(hover * 0.25f);
        total_thrust[gid] = hover;
        armed[gid] = 1;
    } else {
        thrust[gid] = (float4)(0.0f);
        total_thrust[gid] = 0.0f;
        armed[gid] = 0;
    }
}

__kernel void step_dynamics(
    float t, int step_index, __global const uint* seeds, __global const float* runp, __global const float* weather,
    // references (eval_reference), one per drone, shared by all runs
    __global const float4* ref_p, __global const float4* ref_v, __global const float4* ref_a,
    __global const float* led_frac,
    // interactions (interaction)
    __global const float4* lift_factor, __global const float* downwash_z,
    // per-drone Monte Carlo plant parameters
    __global const float* mass, __global const float* max_thrust, __global const float* motor_tau,
    __global const float* drag_cd, __global const float* capacity_ah, __global const float* r_int25,
    // state (in/out)
    __global float4* pos, __global float4* vel, __global float4* quat, __global float4* omega,
    __global float4* thrust, __global float4* vel_int, __global float4* gnss_bias, __global int* armed,
    __global float* total_thrust, __global float* soc, __global float* soc_comp, __global float* temp_c,
    __global float* voltage, __global float* low_timer, __global int* brownout,
    // running metrics (in/out)
    __global float* max_track_err, __global float* min_voltage, __global float* brownout_time) {
    int gid = get_global_id(0);
    if (gid >= RN) return;
    int r = gid / N_DRONES;
    int d = gid - r * N_DRONES;
    __global const float* rp = runp + r * RUN_STRIDE;

    float3 p = pos[gid].xyz;
    float3 v = vel[gid].xyz;
    float4 q = quat[gid];
    float3 w = omega[gid].xyz;
    float3 rpos = ref_p[d].xyz;
    float3 rv = ref_v[d].xyz;

    // ---- sensing: RTK GNSS with Ornstein-Uhlenbeck drift + white noise ----
    uint rng = rng_init(seeds[r] + (uint)step_index, (uint)d);
    float3 bias = gnss_bias[gid].xyz;
    float decay = DT / GNSS_CORR_TIME;
    float diffusion = gnss_param(rp, weather, t, RP_GNSS_DRIFT, WX_GNSS_DRIFT) * sqrt(DT);
    bias = bias * (1.0f - decay) + randn3(&rng) * diffusion;
    gnss_bias[gid] = (float4)(bias, 0.0f);
    float3 p_meas = p + bias + randn3(&rng) * gnss_param(rp, weather, t, RP_GNSS_NOISE, WX_GNSS_NOISE);

    // ---- arming: motors spin up once the reference leaves the pad ----
    int is_armed = armed[gid];
    if (is_armed == 0 && (rpos.z > GROUND_Z + ARM_HEIGHT_M || length(rv) > ARM_SPEED_MPS)) is_armed = 1;
    bool on_ground = p.z <= GROUND_Z + 1.0e-3f;
    if (is_armed == 1 && on_ground && rpos.z <= GROUND_Z + 0.02f && length(rv) < 0.01f)
        is_armed = 0;  // landed, reference parked on the pad
    armed[gid] = is_armed;

    // ---- battery-limited motor ceiling ----
    float v_ratio = clamp(voltage[gid] / NOMINAL_VOLTAGE, 0.5f, 1.2f);
    float t_max = max_thrust[gid] * v_ratio * v_ratio;

    float4 t_cmd = (float4)(0.0f);
    if (is_armed == 1) {
        // ---- position -> velocity PI -> desired force ----
        float3 v_cmd = rv + POS_P * (rpos - p_meas);
        float3 e_v = v_cmd - v;
        float3 vi = clamp(vel_int[gid].xyz + VEL_I * e_v * DT, -VEL_INTEGRAL_LIMIT, VEL_INTEGRAL_LIMIT);
        vel_int[gid] = (float4)(vi, 0.0f);
        float3 a_cmd = ref_a[d].xyz + VEL_P * e_v + vi;
        float3 f_des = (a_cmd + (float3)(0.0f, 0.0f, GRAVITY)) * MASS_NOMINAL;
        float fz = fmax(f_des.z, 0.1f * MASS_NOMINAL * GRAVITY);
        float2 h = f_des.xy;
        float h_len = length(h);
        float h_max = fz * TAN_MAX_TILT;
        if (h_len > h_max) h = h * (h_max / h_len);
        f_des = (float3)(h, fz);

        float3 body_z = qrot(q, (float3)(0.0f, 0.0f, 1.0f));
        float f_coll = clamp(dot(f_des, body_z), 0.0f, 4.0f * MAX_THRUST_NOMINAL);

        // ---- attitude P -> rate P -> torque ----
        float4 q_des = attitude_from_thrust(f_des);
        float4 q_err = qmul(qconj(q), q_des);
        if (q_err.w < 0.0f) q_err = -q_err;
        float3 e_att = q_err.xyz * 2.0f;
        float3 rate_lim = (float3)(MAX_RATE_RP, MAX_RATE_RP, MAX_RATE_YAW);
        float3 w_cmd = clamp(ATT_P * e_att, -rate_lim, rate_lim);
        float3 jw = INERTIA * w;
        float3 tau_cmd = INERTIA * (RATE_P * (w_cmd - w)) + cross(w, jw);
        t_cmd = clamp(motor_mix(f_coll, tau_cmd), 0.0f, t_max);
    } else {
        vel_int[gid] = (float4)(0.0f);
    }

    // ---- motor lag ----
    float alpha = 1.0f - exp(-DT / motor_tau[gid]);
    float4 t_prev = thrust[gid];
    float4 t_now = t_prev + (t_cmd - t_prev) * alpha;
    thrust[gid] = t_now;

    // ---- plant: forces and torques from actual (downwash-derated) thrust ----
    float4 wrench = motor_unmix(t_now * lift_factor[gid]);
    total_thrust[gid] = wrench.x;

    float m = mass[gid];
    float3 v_air = wind_at(rp, weather, p, t) + (float3)(0.0f, 0.0f, downwash_z[gid]);
    float3 v_rel = v - v_air;
    float3 f_drag = v_rel * (-0.5f * RHO_AIR * drag_cd[gid] * DRAG_AREA * length(v_rel));
    float3 f_thrust = qrot(q, (float3)(0.0f, 0.0f, wrench.x));
    float3 acc = (f_thrust + f_drag) / m - (float3)(0.0f, 0.0f, GRAVITY);

    float3 tau = wrench.yzw;
    float3 w_dot = (tau - cross(w, INERTIA * w)) / INERTIA;

    // ---- semi-implicit Euler ----
    v = v + acc * DT;
    p = p + v * DT;
    w = w + w_dot * DT;
    q = normalize(q + qmul(q, (float4)(w, 0.0f)) * (0.5f * DT));

    // ---- ground contact ----
    if (p.z < GROUND_Z) {
        p.z = GROUND_Z;
        v = (float3)(0.0f, 0.0f, fmax(v.z, 0.0f));
        if (is_armed == 0 || f_thrust.z < m * GRAVITY) {
            q = (float4)(0.0f, 0.0f, 0.0f, 1.0f);
            w = (float3)(0.0f);
        }
    }
    pos[gid] = (float4)(p, 0.0f);
    vel[gid] = (float4)(v, 0.0f);
    quat[gid] = q;
    omega[gid] = (float4)(w, 0.0f);

    // ---- battery ----
    float s = soc[gid], sc = soc_comp[gid], temp = temp_c[gid], volt = voltage[gid], timer = low_timer[gid];
    int was_brownout = brownout[gid];
    int flag = was_brownout;
    battery_step(t_now, led_frac[d], capacity_ah[gid], r_int25[gid], rp[RP_AMBIENT],
                 &s, &sc, &temp, &volt, &timer, &flag);
    if (flag == 1 && was_brownout == 0) brownout_time[gid] = t;
    soc[gid] = s;
    soc_comp[gid] = sc;
    temp_c[gid] = temp;
    voltage[gid] = volt;
    low_timer[gid] = timer;
    brownout[gid] = flag;
    min_voltage[gid] = fmin(min_voltage[gid], volt);

    if (is_armed == 1) max_track_err[gid] = fmax(max_track_err[gid], length(p - rpos));
}
