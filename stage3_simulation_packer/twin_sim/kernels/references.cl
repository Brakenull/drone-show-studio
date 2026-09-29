// Phase 2 references on the device (docs/3-phase-3.md §1.2, §3.1, §4):
// piecewise power-basis polynomials from loaders/spline_evaluator.py, same
// timeline and color rules as its NumPy path (see that module's docstring).
// Times are seconds since the show start, so float32 keeps sub-millisecond
// resolution over long shows.

// Last span index of `drone` with span_t0 <= t, or offsets[drone] - 1 if t precedes all.
inline int find_span(__global const int* offsets, __global const float* span_t0, int drone, float t) {
    int lo = offsets[drone];
    int hi = offsets[drone + 1];
    while (lo < hi) {
        int mid = (lo + hi) / 2;
        if (span_t0[mid] <= t) lo = mid + 1; else hi = mid;
    }
    return lo - 1;
}

// which = 0: position, 1: velocity, 2: acceleration (Horner in tau).
inline float3 eval_span(__global const float4* coef, int span, float tau, int which) {
    int base = span * N_COEF;
    float3 out = (float3)(0.0f);
    for (int k = N_COEF - 1; k >= which; --k) {
        float scale = 1.0f;
        if (which == 1) scale = (float)k;
        if (which == 2) scale = (float)(k * (k - 1));
        out = out * tau + coef[base + k].xyz * scale;
    }
    return out;
}

inline float3 reference_at(__global const int* offsets, __global const float* span_t0,
                           __global const float* span_t1, __global const float4* coef,
                           int drone, float t, int which) {
    int first = offsets[drone];
    int span = find_span(offsets, span_t0, drone, t);
    bool outside = false;
    if (span < first) { span = first; outside = true; }
    float span_len = span_t1[span] - span_t0[span];
    float tau = t - span_t0[span];
    if (tau > span_len) { tau = span_len; outside = true; }
    if (tau < 0.0f) tau = 0.0f;
    if (outside && which > 0) return (float3)(0.0f);
    return eval_span(coef, span, tau, which);
}

// Un-rounded lerped RGB in [0, 255].
inline float3 led_color_at(__global const int* color_offsets, __global const float* color_t,
                           __global const float4* color_rgb, int drone, float t) {
    int lo = color_offsets[drone];
    int hi = color_offsets[drone + 1];
    int first = lo, last = hi - 1;
    while (lo < hi) {
        int mid = (lo + hi) / 2;
        if (color_t[mid] <= t) lo = mid + 1; else hi = mid;
    }
    int j = lo - 1;
    if (j < first) return color_rgb[first].xyz;
    if (j >= last) return color_rgb[last].xyz;
    float frac = (t - color_t[j]) / (color_t[j + 1] - color_t[j]);
    return color_rgb[j].xyz + (color_rgb[j + 1].xyz - color_rgb[j].xyz) * frac;
}

__kernel void eval_reference(float t, __global const int* offsets, __global const float* span_t0,
                             __global const float* span_t1, __global const float4* coef,
                             __global const int* color_offsets, __global const float* color_t,
                             __global const float4* color_rgb,
                             __global float4* ref_p, __global float4* ref_v, __global float4* ref_a,
                             __global float* led_frac) {
    int d = get_global_id(0);
    if (d >= N_DRONES) return;
    ref_p[d] = (float4)(reference_at(offsets, span_t0, span_t1, coef, d, t, 0), 0.0f);
    ref_v[d] = (float4)(reference_at(offsets, span_t0, span_t1, coef, d, t, 1), 0.0f);
    ref_a[d] = (float4)(reference_at(offsets, span_t0, span_t1, coef, d, t, 2), 0.0f);
    float3 rgb = led_color_at(color_offsets, color_t, color_rgb, d, t);
    led_frac[d] = (rgb.x + rgb.y + rgb.z) / 765.0f;
}

// Positions / velocities of every drone at every time (`DigitalTwin.sample_grid`).
__kernel void sample_grid(int n_times, __global const float* times, __global const int* offsets,
                          __global const float* span_t0, __global const float* span_t1,
                          __global const float4* coef, __global float4* out_p, __global float4* out_v) {
    int gid = get_global_id(0);
    if (gid >= N_DRONES * n_times) return;
    int d = gid / n_times;
    float t = times[gid - d * n_times];
    out_p[gid] = (float4)(reference_at(offsets, span_t0, span_t1, coef, d, t, 0), 0.0f);
    out_v[gid] = (float4)(reference_at(offsets, span_t0, span_t1, coef, d, t, 1), 0.0f);
}
