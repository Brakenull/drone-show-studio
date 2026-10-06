"""Real-time kinematic pre-validator.

Pure module: no `bpy` dependency, fully unit-testable. Estimates the average
speed a keyframe-to-keyframe transition demands from the nominal timeline
timing an animator authored, flags transitions that exceed the drone fleet's
v_max before they ever reach Phase 2, and can compute a stretched timeline
that brings every transition back under budget.

`D_max` between two keyframes treats the two kinds of drone separately
(`transition_d_max`), the way Phase 2 flies them: formation points pair by
index, drones joining or leaving the formation fly between their formation
points and the padding slots, and drones parked at both keyframes stay where
they are.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence

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
    """v_req = D_max / delta_t. `delta_t <= 0` (keyframes
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
    """Delta_t_safe: the minimum keyframe-to-keyframe
    duration that keeps the rest-to-rest quintic peak velocity within v_max,
    plus `slack_fraction` extra headroom for Phase 2's own collision-avoidance
    bending."""
    if d_max <= 0.0 or v_max <= 0.0:
        return 0.0
    return QUINTIC_PEAK_VELOCITY_FACTOR * d_max / v_max * (1.0 + slack_fraction)


def longest_assigned_flight(from_points: np.ndarray, to_points: np.ndarray) -> float:
    """Longest flight when every point of the smaller set gets its own point
    of the other set by a minimum-total-distance assignment (a stand-in for
    Phase 2's Auction)."""
    from scipy.optimize import linear_sum_assignment
    from scipy.spatial.distance import cdist

    if len(from_points) == 0 or len(to_points) == 0:
        return 0.0
    cost = cdist(from_points, to_points)
    rows, cols = linear_sum_assignment(cost)
    return float(cost[rows, cols].max())


def transition_d_max(p_from: np.ndarray, p_to: np.ndarray, n_from: int, n_to: int) -> float:
    """Longest flight between two keyframes whose first `n_from` / `n_to`
    rows are the formation and the rest padding slots:
    * formation to formation: by index (exact for a shape that only moves or
      rotates, same seed; an over-estimate up to the shape's size when it
      deforms),
    * drones joining the formation: from the padding slots they are parked on,
    * drones leaving it: to the padding slots,
    * drones parked at both keyframes: none (they keep their pad or slot)."""
    shared = min(n_from, n_to)
    d_max = float(np.linalg.norm(p_to[:shared] - p_from[:shared], axis=1).max(initial=0.0))
    if n_to > n_from:
        d_max = max(d_max, longest_assigned_flight(p_from[n_from:], p_to[n_from:n_to]))
    elif n_from > n_to:
        d_max = max(d_max, longest_assigned_flight(p_from[n_to:n_from], p_to[n_to:]))
    return d_max


def evaluate_transitions(
    times: Sequence[float],
    positions_per_keyframe: Sequence[np.ndarray],
    v_max: float,
    formation_counts: Optional[Sequence[int]] = None,
) -> List[TransitionKinematics]:
    """One `TransitionKinematics` per consecutive keyframe pair. `times` and
    `positions_per_keyframe` must be the same length and already in timeline
    order; each array in `positions_per_keyframe` is (N, 3), its first
    `formation_counts[k]` rows the formation (default: all of them)."""
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

        if formation_counts is None:
            d_max = transition_d_max(p_from, p_to, len(p_from), len(p_to))
        else:
            d_max = transition_d_max(p_from, p_to, formation_counts[k], formation_counts[k + 1])
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
    transitions: Sequence[TransitionKinematics],
    v_max: float,
    slack_fraction: float = KINEMATIC_SLACK_FRACTION,
) -> List[float]:
    """Returns a new timeline where every transition's allocated duration is
    at least its `compute_min_safe_duration` (from the `d_max` that
    `evaluate_transitions` measured), cascading any inserted extra
    time forward so later keyframes keep their relative spacing (never
    compresses an already-safe transition, only stretches unsafe ones) —
    the same forward-cascading approach Phase 2's T_min auto-scaling uses,
    kept consistent so a fix applied here doesn't get re-stretched there."""
    if not times:
        return []

    fixed = [float(times[0])]
    for k in range(len(times) - 1):
        nominal_delta = times[k + 1] - times[k]
        safe_delta = compute_min_safe_duration(transitions[k].d_max, v_max, slack_fraction)
        fixed.append(fixed[-1] + max(nominal_delta, safe_delta))
    return fixed
