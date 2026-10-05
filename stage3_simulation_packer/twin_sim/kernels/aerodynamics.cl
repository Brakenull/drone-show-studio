// Wind field + fast downwash proxy; model in physics.py.

// Mean wind + turbulence modes + gust fronts. With a weather table the mean wind, the turbulence
// scale and the distance the modes have been carried come from it; without one (Monte Carlo) they are
// the run's constants, the scale is 1 and the modes move with time (omega already includes the speed).
inline float3 wind_at(__global const float* rp, __global const float* weather, float3 p, float t) {
    float3 w = vload3(0, rp + RP_MEAN);
    float scale = 1.0f;
    float carried = t;
    if (rp[RP_WX_ROWS] > 0.5f) {
        __global const float* a;
        __global const float* b;
        float f = wx_locate(rp, weather, t, &a, &b);
        w = vload3(0, a + WX_WIND) + (vload3(0, b + WX_WIND) - vload3(0, a + WX_WIND)) * f;
        scale = a[WX_TURB] + (b[WX_TURB] - a[WX_TURB]) * f;
        carried = a[WX_ADVECT] + (b[WX_ADVECT] - a[WX_ADVECT]) * f;
    }
    int nm = (int)rp[RP_NMODES];
    for (int m = 0; m < nm; ++m) {
        __global const float* md = rp + RP_MODES + m * 8;
        w += vload3(0, md) * (scale * sin(dot(vload3(0, md + 3), p) - md[6] * carried + md[7]));
    }
    int ng = (int)rp[RP_NGUSTS];
    for (int g = 0; g < ng; ++g) {
        __global const float* gd = rp + RP_GUSTS + g * 9;
        float dur = gd[7];
        if (dur > 0.0f) {
            float local_t = t - gd[6] - dot(p, vload3(0, gd + 3)) / fmax(gd[8], 0.1f);
            if (local_t > 0.0f && local_t < dur)
                w += vload3(0, gd) * (0.5f * (1.0f - cos(2.0f * PI_F * local_t / dur)));
        }
    }
    return w;
}

inline float downwash_speed(float r_sq, float z, float v0) {
    if (z <= 0.0f) return 0.0f;
    float radius = R0 + K_WAKE * z;
    float ratio = R0 / radius;
    return v0 * ratio * ratio * exp(-r_sq / (radius * radius));
}

inline float xy_dist_sq(float3 a, float3 b) {
    float dx = a.x - b.x, dy = a.y - b.y;
    return dx * dx + dy * dy;
}

// X configuration, motor 0 front-left then counter-clockwise (body FLU).
inline float3 motor_offset(int m) {
    float sx = (m == 1 || m == 2) ? -1.0f : 1.0f;
    float sy = (m == 2 || m == 3) ? -1.0f : 1.0f;
    return (float3)(sx * HALF_DIAG, sy * HALF_DIAG, 0.0f);
}

// Per drone: downwash lift factor per motor + downward air speed at the centre,
// nearest neighbour inside PROX_R, and the running closest approach.
__kernel void interaction(float t, __global const float4* pos, __global const float4* quat,
                          __global const float* total_thrust, __global const float* hover_thrust,
                          __global const int* starts, __global const int* sorted,
                          __global float4* lift_factor, __global float* downwash_z,
                          __global float* min_sep, __global int* min_sep_partner, __global float* min_sep_time) {
    int gid = get_global_id(0);
    if (gid >= RN) return;
    int r = gid / N_DRONES;
    int i = gid - r * N_DRONES;
    int base = r * N_DRONES;
    __global const int* st = starts + r * (T_CELLS + 1);
    __global const int* list = sorted + base;
    float3 p_i = pos[gid].xyz;

    // ---- proximity (safety metric): small sphere around the drone ----
    float best = 1.0e9f;
    int best_id = -1;
    {
        int3 lo = (int3)(cell_of(p_i.x - PROX_R), cell_of(p_i.y - PROX_R), cell_of(p_i.z - PROX_R));
        int3 hi = (int3)(cell_of(p_i.x + PROX_R), cell_of(p_i.y + PROX_R), cell_of(p_i.z + PROX_R));
        for (int cx = lo.x; cx <= hi.x; ++cx)
        for (int cy = lo.y; cy <= hi.y; ++cy)
        for (int cz = lo.z; cz <= hi.z; ++cz) {
            int key = cell_key(cx, cy, cz);
            int e = st[key + 1];
            for (int k = st[key]; k < e; ++k) {
                int j = list[k];
                if (j == i) continue;
                float d = length(pos[base + j].xyz - p_i);
                if (d < PROX_R && d < best) { best = d; best_id = j; }
            }
        }
    }
    if (best < min_sep[gid]) {
        min_sep[gid] = best;
        min_sep_partner[gid] = best_id;
        min_sep_time[gid] = t;
    }

    // ---- downwash: only drones above, inside their wake cone ----
    // The query box is centred half a wake-depth above the drone, so it covers
    // the cone-shaped region that can shed wake onto it, not the half-space below.
    float4 q_i = quat[gid];
    const float two_rho_a = 2.0f * RHO_AIR * DISK_AREA;
    float v_hover_i = sqrt(fmax(hover_thrust[gid], 1e-3f) / two_rho_a);
    float3 m0 = p_i + qrot(q_i, motor_offset(0));
    float3 m1 = p_i + qrot(q_i, motor_offset(1));
    float3 m2 = p_i + qrot(q_i, motor_offset(2));
    float3 m3 = p_i + qrot(q_i, motor_offset(3));
    float w0 = 0.0f, w1 = 0.0f, w2 = 0.0f, w3 = 0.0f, w_cg = 0.0f;

    float3 centre = p_i + (float3)(0.0f, 0.0f, 0.5f * WAKE_DEPTH);
    int3 lo = (int3)(cell_of(centre.x - WAKE_QR), cell_of(centre.y - WAKE_QR), cell_of(centre.z - WAKE_QR));
    int3 hi = (int3)(cell_of(centre.x + WAKE_QR), cell_of(centre.y + WAKE_QR), cell_of(centre.z + WAKE_QR));
    for (int cx = lo.x; cx <= hi.x; ++cx)
    for (int cy = lo.y; cy <= hi.y; ++cy)
    for (int cz = lo.z; cz <= hi.z; ++cz) {
        int key = cell_key(cx, cy, cz);
        int e = st[key + 1];
        for (int k = st[key]; k < e; ++k) {
            int j = list[k];
            if (j == i) continue;
            float3 p_j = pos[base + j].xyz;
            float dz = p_j.z - p_i.z;
            if (dz > -HALF_DIAG && dz < WAKE_DEPTH + HALF_DIAG) {
                float radius = R0 + K_WAKE * fmax(dz, 0.0f) + HALF_DIAG;
                float dx = p_j.x - p_i.x, dy = p_j.y - p_i.y;
                float reach = WAKE_RADIAL_CUTOFF * radius;
                if (dx * dx + dy * dy < reach * reach) {
                    float v0 = sqrt(fmax(total_thrust[base + j], 0.0f) / two_rho_a);
                    w0 += downwash_speed(xy_dist_sq(m0, p_j), p_j.z - m0.z, v0);
                    w1 += downwash_speed(xy_dist_sq(m1, p_j), p_j.z - m1.z, v0);
                    w2 += downwash_speed(xy_dist_sq(m2, p_j), p_j.z - m2.z, v0);
                    w3 += downwash_speed(xy_dist_sq(m3, p_j), p_j.z - m3.z, v0);
                    w_cg += downwash_speed(dx * dx + dy * dy, dz, v0);
                }
            }
        }
    }
    lift_factor[gid] = (float4)(v_hover_i / (v_hover_i + w0), v_hover_i / (v_hover_i + w1),
                                v_hover_i / (v_hover_i + w2), v_hover_i / (v_hover_i + w3));
    downwash_z[gid] = -w_cg;
}
