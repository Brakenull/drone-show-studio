"""B-spline trajectory + LED color evaluation (docs/3-phase-3.md §1.2, §3.1, §4).

Every clamped B-spline segment is converted once, on the host, into its exact
piecewise-polynomial (power basis) form: one polynomial per non-degenerate
knot span, in span-local time ``tau = t - span_t0``. Evaluating p, v, a then
reduces to a binary search over a drone's spans plus Horner's rule, which is
cheap and branch-light on the GPU (`eval_reference_kernel`,
`sample_grid_kernel`) and trivially vectorized on the host (`evaluate_numpy`).

Timeline rules (identical in the NumPy and Warp paths):
* before a drone's first span: first span's start position, v = a = 0
* inside a span: exact polynomial
* in a gap between segments, or after the last span: last reached span's end
  position, v = a = 0 (the drone holds position)

Color rule (§1.2): per-channel linear interpolation between adjacent
`time_sec` keyframes, rounded to uint8 as ``floor(x + 0.5)``; two keyframes at
the same instant make a step (the later one wins from that instant on);
clamp-to-edge outside the keyframe range. Keyframes from all of a drone's
segments are merged into one timeline.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .arrow_loader import ShowTrajectories

_SPAN_EPS = 1e-12


# --------------------------------------------------------------------------- #
# Host-side conversion: clamped B-spline -> piecewise power basis
# --------------------------------------------------------------------------- #

def _ders_basis_funs(span: int, u: float, degree: int, knots: np.ndarray, n_ders: int) -> np.ndarray:
    """NURBS Book A2.3: derivatives 0..n_ders of the non-zero basis functions at u."""
    ndu = np.zeros((degree + 1, degree + 1))
    left = np.zeros(degree + 1)
    right = np.zeros(degree + 1)
    ndu[0, 0] = 1.0
    for j in range(1, degree + 1):
        left[j] = u - knots[span + 1 - j]
        right[j] = knots[span + j] - u
        saved = 0.0
        for r in range(j):
            ndu[j, r] = right[r + 1] + left[j - r]
            temp = ndu[r, j - 1] / ndu[j, r]
            ndu[r, j] = saved + right[r + 1] * temp
            saved = left[j - r] * temp
        ndu[j, j] = saved

    ders = np.zeros((n_ders + 1, degree + 1))
    ders[0, :] = ndu[:, degree]
    a = np.zeros((2, degree + 1))
    for r in range(degree + 1):
        s1, s2 = 0, 1
        a[0, 0] = 1.0
        for k in range(1, n_ders + 1):
            d = 0.0
            rk, pk = r - k, degree - k
            if r >= k:
                a[s2, 0] = a[s1, 0] / ndu[pk + 1, rk]
                d = a[s2, 0] * ndu[rk, pk]
            j1 = 1 if rk >= -1 else -rk
            j2 = k - 1 if r - 1 <= pk else degree - r
            for j in range(j1, j2 + 1):
                a[s2, j] = (a[s1, j] - a[s1, j - 1]) / ndu[pk + 1, rk + j]
                d += a[s2, j] * ndu[rk + j, pk]
            if r <= pk:
                a[s2, k] = -a[s1, k - 1] / ndu[pk + 1, r]
                d += a[s2, k] * ndu[r, pk]
            ders[k, r] = d
            s1, s2 = s2, s1
    factor = float(degree)
    for k in range(1, n_ders + 1):
        ders[k, :] *= factor
        factor *= degree - k
    return ders


def bspline_to_power_spans(knots: np.ndarray, control_points: np.ndarray) -> list[tuple[float, float, np.ndarray]]:
    """Split one clamped B-spline into [(u0, u1, coeffs[(degree+1), 3]), ...].

    ``coeffs[k]`` is the k-th power-basis coefficient in ``tau = u - u0``,
    i.e. ``C^(k)(u0) / k!`` -- exact, since each span is a single polynomial.
    """
    n_cp = control_points.shape[0]
    degree = knots.size - n_cp - 1
    spans = []
    for span in range(degree, n_cp):
        u0, u1 = knots[span], knots[span + 1]
        if u1 - u0 <= _SPAN_EPS:
            continue
        ders = _ders_basis_funs(span, u0, degree, knots, degree)
        local_cp = control_points[span - degree: span + 1]
        coeffs = ders @ local_cp
        for k in range(degree + 1):
            coeffs[k] /= math.factorial(k)
        spans.append((float(u0), float(u1), coeffs))
    return spans


@dataclass
class PiecewiseTrajectories:
    """Flat, device-agnostic piecewise-polynomial form of a whole show."""

    fleet_size: int
    n_coef: int                  # degree + 1
    span_offsets: np.ndarray     # (N+1,) int32, drone d owns spans [off[d], off[d+1])
    span_t0: np.ndarray          # (S,) float64, absolute show time
    span_t1: np.ndarray          # (S,) float64
    span_coef: np.ndarray        # (S, n_coef, 3) float64, tau = t - span_t0
    color_offsets: np.ndarray    # (N+1,) int32
    color_t: np.ndarray          # (K,) float64
    color_rgb: np.ndarray        # (K, 3) float64 in [0, 255]
    start_time_sec: float
    end_time_sec: float


def build_piecewise(show: ShowTrajectories) -> PiecewiseTrajectories:
    n_coef = show.degree + 1
    span_offsets, color_offsets = [0], [0]
    t0s, t1s, coefs, color_t, color_rgb = [], [], [], [], []
    for drone in show.drones:
        for seg in drone.segments:
            for u0, u1, coeffs in bspline_to_power_spans(seg.knot_vector, seg.control_points):
                t0s.append(seg.start_time_sec + u0)
                t1s.append(seg.start_time_sec + u1)
                coefs.append(coeffs)
            color_t.extend(seg.color_time_sec.tolist())
            color_rgb.extend(seg.color_rgb.astype(np.float64).tolist())
        span_offsets.append(len(t0s))
        color_offsets.append(len(color_t))

    order_ok = all(
        np.all(np.diff(np.asarray(t0s[a:b])) >= 0) for a, b in zip(span_offsets[:-1], span_offsets[1:])
    )
    if not order_ok:  # pragma: no cover - loader already enforces segment ordering
        raise ValueError("spans are not time-ordered per drone")

    # A drone's color keyframes are merged across segments; a stable sort
    # keeps same-instant keyframes in contract order (the later one wins).
    color_t_arr = np.asarray(color_t, dtype=np.float64)
    color_rgb_arr = np.asarray(color_rgb, dtype=np.float64).reshape(-1, 3)
    for a, b in zip(color_offsets[:-1], color_offsets[1:]):
        order = np.argsort(color_t_arr[a:b], kind="stable")
        color_t_arr[a:b] = color_t_arr[a:b][order]
        color_rgb_arr[a:b] = color_rgb_arr[a:b][order]

    t0_arr = np.asarray(t0s, dtype=np.float64)
    t1_arr = np.asarray(t1s, dtype=np.float64)
    return PiecewiseTrajectories(
        fleet_size=show.fleet_size,
        n_coef=n_coef,
        span_offsets=np.asarray(span_offsets, dtype=np.int32),
        span_t0=t0_arr,
        span_t1=t1_arr,
        span_coef=np.asarray(coefs, dtype=np.float64).reshape(-1, n_coef, 3),
        color_offsets=np.asarray(color_offsets, dtype=np.int32),
        color_t=color_t_arr,
        color_rgb=color_rgb_arr,
        start_time_sec=float(t0_arr.min()) if t0_arr.size else 0.0,
        end_time_sec=float(t1_arr.max()) if t1_arr.size else 0.0,
    )


# --------------------------------------------------------------------------- #
# NumPy reference evaluation (float64) -- used by tests, the packer cross-check
# and the CPU fallback path.
# --------------------------------------------------------------------------- #

def evaluate_numpy(pw: PiecewiseTrajectories, times: np.ndarray,
                   drone_ids: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (pos, vel, acc), each (len(drone_ids), len(times), 3) float64."""
    times = np.asarray(times, dtype=np.float64).reshape(-1)
    ids = np.arange(pw.fleet_size) if drone_ids is None else np.asarray(drone_ids).reshape(-1)
    pos = np.zeros((ids.size, times.size, 3))
    vel = np.zeros_like(pos)
    acc = np.zeros_like(pos)
    for row, d in enumerate(ids):
        a, b = int(pw.span_offsets[d]), int(pw.span_offsets[d + 1])
        t0, t1, coef = pw.span_t0[a:b], pw.span_t1[a:b], pw.span_coef[a:b]
        idx = np.searchsorted(t0, times, side="right") - 1
        before = idx < 0
        idx = np.clip(idx, 0, b - a - 1)
        span_len = t1[idx] - t0[idx]
        tau = times - t0[idx]
        outside = before | (tau > span_len)
        tau = np.clip(tau, 0.0, span_len)
        c = coef[idx]  # (T, n_coef, 3)
        p = np.zeros((times.size, 3))
        v = np.zeros_like(p)
        acc_d = np.zeros_like(p)
        tau_col = tau[:, None]
        for k in range(pw.n_coef - 1, -1, -1):
            p = p * tau_col + c[:, k]
            if k >= 1:
                v = v * tau_col + k * c[:, k]
            if k >= 2:
                acc_d = acc_d * tau_col + k * (k - 1) * c[:, k]
        v[outside] = 0.0
        acc_d[outside] = 0.0
        pos[row], vel[row], acc[row] = p, v, acc_d
    return pos, vel, acc


def evaluate_colors_numpy(pw: PiecewiseTrajectories, times: np.ndarray,
                          drone_ids: np.ndarray | None = None) -> np.ndarray:
    """Return (len(drone_ids), len(times), 3) uint8 LED colors (§1.2 rule)."""
    times = np.asarray(times, dtype=np.float64).reshape(-1)
    ids = np.arange(pw.fleet_size) if drone_ids is None else np.asarray(drone_ids).reshape(-1)
    out = np.zeros((ids.size, times.size, 3), dtype=np.uint8)
    for row, d in enumerate(ids):
        a, b = int(pw.color_offsets[d]), int(pw.color_offsets[d + 1])
        kt, krgb = pw.color_t[a:b], pw.color_rgb[a:b]
        j = np.searchsorted(kt, times, side="right") - 1
        rgb = np.empty((times.size, 3))
        first = j < 0
        last = j >= kt.size - 1
        mid = ~(first | last)
        rgb[first] = krgb[0]
        rgb[last] = krgb[-1]
        if np.any(mid):
            jm = j[mid]
            frac = (times[mid] - kt[jm]) / (kt[jm + 1] - kt[jm])
            rgb[mid] = krgb[jm] + (krgb[jm + 1] - krgb[jm]) * frac[:, None]
        out[row] = np.clip(np.floor(rgb + 0.5), 0, 255).astype(np.uint8)
    return out


# --------------------------------------------------------------------------- #
# Warp (GPU / CPU-LLVM) evaluation
# --------------------------------------------------------------------------- #

try:
    import warp as wp
except ImportError:  # pragma: no cover - the NumPy path still works without Warp
    wp = None


if wp is not None:

    @wp.func
    def find_span(offsets: wp.array(dtype=wp.int32), span_t0: wp.array(dtype=wp.float32), drone: int,
                  t: float) -> int:
        """Last span index of `drone` with span_t0 <= t, or offsets[drone] - 1 if t precedes all."""
        lo = offsets[drone]
        hi = offsets[drone + 1]
        # invariant: answer in [lo - 1, hi - 1]
        while lo < hi:
            mid = (lo + hi) // 2
            if span_t0[mid] <= t:
                lo = mid + 1
            else:
                hi = mid
        return lo - 1

    @wp.func
    def eval_span(span_coef: wp.array(dtype=wp.vec3), n_coef: int, span: int, tau: float, which: int) -> wp.vec3:
        """which = 0: position, 1: velocity, 2: acceleration (Horner in tau)."""
        base = span * n_coef
        out = wp.vec3(0.0, 0.0, 0.0)
        k = n_coef - 1
        while k >= which:
            scale = float(1.0)
            if which == 1:
                scale = float(k)
            if which == 2:
                scale = float(k * (k - 1))
            out = out * tau + span_coef[base + k] * scale
            k -= 1
        return out

    @wp.func
    def reference_at(offsets: wp.array(dtype=wp.int32), span_t0: wp.array(dtype=wp.float32),
                     span_t1: wp.array(dtype=wp.float32), span_coef: wp.array(dtype=wp.vec3), n_coef: int,
                     drone: int, t: float, which: int) -> wp.vec3:
        first = offsets[drone]
        span = find_span(offsets, span_t0, drone, t)
        outside = False
        if span < first:
            span = first
            outside = True
        span_len = span_t1[span] - span_t0[span]
        tau = t - span_t0[span]
        if tau > span_len:
            tau = span_len
            outside = True
        if tau < 0.0:
            tau = 0.0
        if outside and which > 0:
            return wp.vec3(0.0, 0.0, 0.0)
        return eval_span(span_coef, n_coef, span, tau, which)

    @wp.func
    def led_color_at(color_offsets: wp.array(dtype=wp.int32), color_t: wp.array(dtype=wp.float32),
                     color_rgb: wp.array(dtype=wp.vec3), drone: int, t: float) -> wp.vec3:
        """Un-rounded lerped RGB in [0, 255]."""
        lo = color_offsets[drone]
        hi = color_offsets[drone + 1]
        first = lo
        last = hi - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if color_t[mid] <= t:
                lo = mid + 1
            else:
                hi = mid
        j = lo - 1
        if j < first:
            return color_rgb[first]
        if j >= last:
            return color_rgb[last]
        frac = (t - color_t[j]) / (color_t[j + 1] - color_t[j])
        return color_rgb[j] + (color_rgb[j + 1] - color_rgb[j]) * frac

    @wp.kernel
    def eval_reference_kernel(t: float, offsets: wp.array(dtype=wp.int32), span_t0: wp.array(dtype=wp.float32),
                              span_t1: wp.array(dtype=wp.float32), span_coef: wp.array(dtype=wp.vec3),
                              n_coef: int, color_offsets: wp.array(dtype=wp.int32),
                              color_t: wp.array(dtype=wp.float32), color_rgb: wp.array(dtype=wp.vec3),
                              ref_p: wp.array(dtype=wp.vec3), ref_v: wp.array(dtype=wp.vec3),
                              ref_a: wp.array(dtype=wp.vec3), led_frac: wp.array(dtype=float)):
        d = wp.tid()
        ref_p[d] = reference_at(offsets, span_t0, span_t1, span_coef, n_coef, d, t, 0)
        ref_v[d] = reference_at(offsets, span_t0, span_t1, span_coef, n_coef, d, t, 1)
        ref_a[d] = reference_at(offsets, span_t0, span_t1, span_coef, n_coef, d, t, 2)
        rgb = led_color_at(color_offsets, color_t, color_rgb, d, t)
        led_frac[d] = (rgb[0] + rgb[1] + rgb[2]) / 765.0

    @wp.kernel
    def sample_grid_kernel(times: wp.array(dtype=wp.float32), offsets: wp.array(dtype=wp.int32),
                           span_t0: wp.array(dtype=wp.float32), span_t1: wp.array(dtype=wp.float32),
                           span_coef: wp.array(dtype=wp.vec3), n_coef: int,
                           out_p: wp.array2d(dtype=wp.vec3), out_v: wp.array2d(dtype=wp.vec3)):
        d, k = wp.tid()
        out_p[d, k] = reference_at(offsets, span_t0, span_t1, span_coef, n_coef, d, times[k], 0)
        out_v[d, k] = reference_at(offsets, span_t0, span_t1, span_coef, n_coef, d, times[k], 1)


class WarpTrajectoryBuffers:
    """Device-resident piecewise trajectories + LED keyframes.

    Times are stored relative to `time_origin` (the show start) so float32
    keeps sub-millisecond resolution over long shows.
    """

    def __init__(self, pw: PiecewiseTrajectories, device: str | None = None):
        if wp is None:
            raise ImportError("warp-lang is required for WarpTrajectoryBuffers")
        self.pw = pw
        self.device = device
        self.fleet_size = pw.fleet_size
        self.n_coef = pw.n_coef
        self.time_origin = pw.start_time_sec
        f32 = np.float32
        self.offsets = wp.array(pw.span_offsets, dtype=wp.int32, device=device)
        self.span_t0 = wp.array((pw.span_t0 - self.time_origin).astype(f32), dtype=wp.float32, device=device)
        self.span_t1 = wp.array((pw.span_t1 - self.time_origin).astype(f32), dtype=wp.float32, device=device)
        self.span_coef = wp.array(pw.span_coef.reshape(-1, 3).astype(f32), dtype=wp.vec3, device=device)
        self.color_offsets = wp.array(pw.color_offsets, dtype=wp.int32, device=device)
        self.color_t = wp.array((pw.color_t - self.time_origin).astype(f32), dtype=wp.float32, device=device)
        self.color_rgb = wp.array(pw.color_rgb.astype(f32), dtype=wp.vec3, device=device)

    def launch_reference(self, t_abs: float, ref_p, ref_v, ref_a, led_frac, record_cmd: bool = False):
        """Evaluate every drone's p/v/a/LED reference at `t_abs`.

        With record_cmd=True the launch is also returned as a `wp.Launch` that
        a stepping loop can replay after `set_param_by_name("t", t_abs - time_origin)`,
        skipping per-step argument packing.
        """
        return wp.launch(
            eval_reference_kernel,
            dim=self.fleet_size,
            inputs=[float(t_abs - self.time_origin), self.offsets, self.span_t0, self.span_t1, self.span_coef,
                    self.n_coef, self.color_offsets, self.color_t, self.color_rgb],
            outputs=[ref_p, ref_v, ref_a, led_frac],
            device=self.device,
            record_cmd=record_cmd,
        )

    def sample_grid(self, times_abs: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Sample position/velocity for every drone at every time on the device."""
        times = wp.array((np.asarray(times_abs, dtype=np.float64) - self.time_origin).astype(np.float32),
                         dtype=wp.float32, device=self.device)
        shape = (self.fleet_size, times.shape[0])
        out_p = wp.zeros(shape, dtype=wp.vec3, device=self.device)
        out_v = wp.zeros(shape, dtype=wp.vec3, device=self.device)
        wp.launch(sample_grid_kernel, dim=shape,
                  inputs=[times, self.offsets, self.span_t0, self.span_t1, self.span_coef, self.n_coef],
                  outputs=[out_p, out_v], device=self.device)
        return out_p.numpy(), out_v.numpy()
