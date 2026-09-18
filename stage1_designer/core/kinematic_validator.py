"""Real-time kinematic pre-validator (spec section 3.5).

Pure module: no `bpy` dependency, fully unit-testable. Estimates the average
speed a keyframe-to-keyframe transition demands from the nominal timeline
timing an animator authored, flags transitions that exceed the drone fleet's
v_max before they ever reach Phase 2, and can compute a stretched timeline
that brings every transition back under budget.

`D_max` between two keyframes is approximated via same-array-index
correspondence (spec section 3.5: "hoặc xấp xỉ qua phân bố tâm cụm") since
Phase 1 has no visibility into Phase 2's Auction point-matching — the actual
per-drone assignment can only reduce total travel versus this identity
pairing, so this approximation is conservative (it warns at least as often
as the real assignment would, never less).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence

import numpy as np

from ..config import KINEMATIC_SLACK_FRACTION, KINEMATIC_WARNING_FRACTION, QUINTIC_PEAK_VELOCITY_FACTOR

STATUS_OK = "OK"
STATUS_WARNING = "WARNING"
STATUS_ERROR = "ERROR"


@dataclass(frozen=True)
class TransitionKinematics:
    from_index: int  # index of the earlier keyframe in the sequence
    to_index: int
    from_time_sec: float
    to_time_sec: float
    d_max: float
    delta_t: float
    v_req: float
    status: str


def required_velocity(d_max: float, delta_t: float) -> float:
    """v_req = D_max / delta_t (spec section 3.5). `delta_t <= 0` (keyframes
    at the same time, or out of order) is treated as an immediate, infinite
    velocity demand rather than raising, so callers can classify it as an
    unconditional ERROR."""
    if delta_t <= 0.0:
        return float("inf") if d_max > 0.0 else 0.0
    return d_max / delta_t


def classify_status(v_req: float, v_max: float) -> str:
    if v_req > v_max:
        return STATUS_ERROR
    if v_req > KINEMATIC_WARNING_FRACTION * v_max:
        return STATUS_WARNING
    return STATUS_OK


def compute_min_safe_duration(d_max: float, v_max: float, slack_fraction: float = KINEMATIC_SLACK_FRACTION) -> float:
    """Delta_t_safe (spec section 3.5): the minimum keyframe-to-keyframe
    duration that keeps the rest-to-rest quintic peak velocity within v_max,
    plus `slack_fraction` extra headroom for Phase 2's own collision-avoidance
    bending."""
    if d_max <= 0.0 or v_max <= 0.0:
        return 0.0
    return QUINTIC_PEAK_VELOCITY_FACTOR * d_max / v_max * (1.0 + slack_fraction)


def evaluate_transitions(
    times: Sequence[float],
    positions_per_keyframe: Sequence[np.ndarray],
    v_max: float,
) -> List[TransitionKinematics]:
    """One `TransitionKinematics` per consecutive keyframe pair. `times` and
    `positions_per_keyframe` must be the same length and already in timeline
    order; each array in `positions_per_keyframe` is (N, 3)."""
    if len(times) != len(positions_per_keyframe):
        raise ValueError("times and positions_per_keyframe must have the same length")

    results: List[TransitionKinematics] = []
    for k in range(len(times) - 1):
        p_from = np.asarray(positions_per_keyframe[k], dtype=float)
        p_to = np.asarray(positions_per_keyframe[k + 1], dtype=float)
        if p_from.shape != p_to.shape:
            raise ValueError(
                f"keyframe {k} has {p_from.shape[0]} points but keyframe {k + 1} has {p_to.shape[0]}"
            )

        d_max = float(np.linalg.norm(p_to - p_from, axis=1).max()) if len(p_from) else 0.0
        delta_t = times[k + 1] - times[k]
        v_req = required_velocity(d_max, delta_t)
        results.append(
            TransitionKinematics(
                from_index=k,
                to_index=k + 1,
                from_time_sec=times[k],
                to_time_sec=times[k + 1],
                d_max=d_max,
                delta_t=delta_t,
                v_req=v_req,
                status=classify_status(v_req, v_max),
            )
        )
    return results


def has_error(transitions: Sequence[TransitionKinematics]) -> bool:
    return any(t.status == STATUS_ERROR for t in transitions)


def auto_fix_times(
    times: Sequence[float],
    positions_per_keyframe: Sequence[np.ndarray],
    v_max: float,
    slack_fraction: float = KINEMATIC_SLACK_FRACTION,
) -> List[float]:
    """Returns a new timeline where every transition's allocated duration is
    at least its `compute_min_safe_duration`, cascading any inserted extra
    time forward so later keyframes keep their relative spacing (never
    compresses an already-safe transition, only stretches unsafe ones) —
    the same forward-cascading approach Phase 2's T_min auto-scaling uses,
    kept consistent so a fix applied here doesn't get re-stretched there."""
    if not times:
        return []

    fixed = [float(times[0])]
    for k in range(len(times) - 1):
        p_from = np.asarray(positions_per_keyframe[k], dtype=float)
        p_to = np.asarray(positions_per_keyframe[k + 1], dtype=float)
        d_max = float(np.linalg.norm(p_to - p_from, axis=1).max()) if len(p_from) else 0.0
        nominal_delta = times[k + 1] - times[k]
        safe_delta = compute_min_safe_duration(d_max, v_max, slack_fraction)
        fixed.append(fixed[-1] + max(nominal_delta, safe_delta))
    return fixed
