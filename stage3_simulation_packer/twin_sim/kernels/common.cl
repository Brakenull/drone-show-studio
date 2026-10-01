// Shared layout, vector/quaternion math and RNG for the digital-twin kernels
// (docs/3-phase-3.md §3). simulator.py concatenates, in order:
//
//   common.cl, references.cl, grid.cl, aerodynamics.cl, battery_discharge.cl,
//   multi_agent_dynamics.cl
//
// after a header of #defines: the model constants from physics.py, the drone
// profile, the fleet size N_DRONES, the batch size N_RUNS, DT and the grid
// layout. One program is built per profile / batch size / dt / wind-mode count.
//
// Batching: global id = run * N_DRONES + drone, so N_RUNS Monte Carlo runs of
// the same show step together in one launch; the references (one per drone)
// are shared by all runs. Vectors are stored as float4 (w unused),
// quaternions as float4 (x, y, z, w). Everything is float32.

#define RN (N_RUNS * N_DRONES)
#define PI_F 3.14159265358979f

// Per-run parameter block: runp + run * RUN_STRIDE (simulator._run_params).
#define RP_AMBIENT 0
#define RP_GNSS_NOISE 1
#define RP_GNSS_DRIFT 2
#define RP_INIT_ERR 3
#define RP_MEAN 4
#define RP_NMODES 7
#define RP_NGUSTS 8
#define RP_WX_FIRST 9        // first row of this run's weather table in `weather`
#define RP_WX_ROWS 10        // 0: constant weather, the values above
#define RP_MODES 12          // 8 floats per turbulence mode: amp xyz, k xyz, omega, phase
// RP_GUSTS (= RP_MODES + 8 * max modes) is defined by the host: 9 floats per gust front:
// peak vector xyz, sweep direction xyz, start, duration, sweep speed.

// Weather table (twin_sim/weather.py, docs/4-condition_simulator.md B6): rows of WX_STRIDE
// floats at WX_HZ over show time. Wind columns are interpolated, RTK columns step.
#define WX_WIND 0
#define WX_TURB 3
#define WX_ADVECT 4
#define WX_GNSS_NOISE 5
#define WX_GNSS_DRIFT 6

// Rows a and b around time t and the blend fraction between them.
inline float wx_locate(__global const float* rp, __global const float* weather, float t,
                       __global const float** a, __global const float** b) {
    int rows = (int)rp[RP_WX_ROWS];
    __global const float* base = weather + (int)rp[RP_WX_FIRST] * WX_STRIDE;
    float x = fmax(t * WX_HZ, 0.0f);
    int i0 = min((int)floor(x), rows - 1);
    int i1 = min(i0 + 1, rows - 1);
    *a = base + i0 * WX_STRIDE;
    *b = base + i1 * WX_STRIDE;
    return clamp(x - (float)i0, 0.0f, 1.0f);
}

// A GNSS parameter (RP_GNSS_NOISE / RP_GNSS_DRIFT, or its WX_ column) at time t.
inline float gnss_param(__global const float* rp, __global const float* weather, float t, int rp_index, int column) {
    if (rp[RP_WX_ROWS] < 0.5f) return rp[rp_index];
    __global const float* a;
    __global const float* b;
    wx_locate(rp, weather, t, &a, &b);
    return a[column];
}

// ------------------------------------------------------------------ math --

inline float3 qrot(float4 q, float3 v) {
    float3 u = q.xyz;
    return v * (2.0f * q.w * q.w - 1.0f) + cross(u, v) * (2.0f * q.w) + u * (2.0f * dot(u, v));
}

inline float4 qmul(float4 a, float4 b) {
    return (float4)(a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);
}

inline float4 qconj(float4 q) { return (float4)(-q.x, -q.y, -q.z, q.w); }

// Rotation matrix with columns b1, b2, b3 -> unit quaternion (Shepperd's method).
inline float4 quat_from_cols(float3 b1, float3 b2, float3 b3) {
    float m00 = b1.x, m10 = b1.y, m20 = b1.z;
    float m01 = b2.x, m11 = b2.y, m21 = b2.z;
    float m02 = b3.x, m12 = b3.y, m22 = b3.z;
    float tr = m00 + m11 + m22;
    float x, y, z, w, h, f;
    if (tr >= 0.0f) {
        h = sqrt(tr + 1.0f); w = 0.5f * h; f = 0.5f / h;
        x = (m21 - m12) * f; y = (m02 - m20) * f; z = (m10 - m01) * f;
    } else {
        int max_diag = 0;
        if (m11 > m00) max_diag = 1;
        if (max_diag == 0 && m22 > m00) max_diag = 2;
        else if (max_diag == 1 && m22 > m11) max_diag = 2;
        if (max_diag == 0) {
            h = sqrt((m00 - (m11 + m22)) + 1.0f); x = 0.5f * h; f = 0.5f / h;
            y = (m01 + m10) * f; z = (m20 + m02) * f; w = (m21 - m12) * f;
        } else if (max_diag == 1) {
            h = sqrt((m11 - (m22 + m00)) + 1.0f); y = 0.5f * h; f = 0.5f / h;
            z = (m12 + m21) * f; x = (m01 + m10) * f; w = (m02 - m20) * f;
        } else {
            h = sqrt((m22 - (m00 + m11)) + 1.0f); z = 0.5f * h; f = 0.5f / h;
            x = (m20 + m02) * f; y = (m12 + m21) * f; w = (m10 - m01) * f;
        }
    }
    return normalize((float4)(x, y, z, w));
}

// ------------------------------------------------------------------- rng --
// Counter-based: a stream is seeded from (seed, offset), e.g. (run seed + step,
// drone), so results do not depend on scheduling or batch size.

inline uint pcg_hash(uint v) {
    uint state = v * 747796405u + 2891336453u;
    uint word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    return (word >> 22u) ^ word;
}

inline uint rng_init(uint seed, uint offset) { return pcg_hash(seed ^ pcg_hash(offset + 0x9E3779B9u)); }

inline float rand_uniform(uint* state) {     // (0, 1]
    *state = *state * 747796405u + 2891336453u;
    uint word = ((*state >> ((*state >> 28u) + 4u)) ^ *state) * 277803737u;
    word = (word >> 22u) ^ word;
    return ((float)(word >> 8) + 1.0f) * (1.0f / 16777216.0f);
}

inline float randn(uint* state) {             // Box-Muller
    float u1 = rand_uniform(state);
    float u2 = rand_uniform(state);
    return sqrt(-2.0f * log(u1)) * cos(2.0f * PI_F * u2);
}

inline float3 randn3(uint* state) {
    float x = randn(state);
    float y = randn(state);
    float z = randn(state);
    return (float3)(x, y, z);
}
