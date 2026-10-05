"""B-spline trajectory + LED color evaluation.

Every clamped B-spline segment is converted once, on the host, into its exact
piecewise-polynomial (power basis) form: one polynomial per non-degenerate
knot span, in span-local time ``tau = t - span_t0``. Evaluating p, v, a then
reduces to a binary search over a drone's spans plus Horner's rule, which is
cheap and branch-light on the device (`kernels/references.cl`) and trivially
vectorized on the host (`evaluate_numpy`).

Timeline rules (identical in the NumPy and OpenCL paths):
* before a drone's first span: first span's start position, v = a = 0
* inside a span: exact polynomial
* in a gap between segments, or after the last span: last reached span's end
  position, v = a = 0 (the drone holds position)

Color rule: per-channel linear interpolation between adjacent
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
    """Return (len(drone_ids), len(times), 3) uint8 LED colors."""
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
