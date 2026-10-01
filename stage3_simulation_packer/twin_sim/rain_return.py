"""Rain rule: time to home, coverage and abort flights (docs/4-condition_simulator.md §5.1, §5.2, B7).

Rule: when the rain reaches its alert level, the return to the holding area is commanded `reaction`
seconds later. The drones finish the transition they are in (it is checked and safe) and fly the
planned return path of the formation it ends at; at a formation they start its return at once. From
the last formation the return is the show's own return leg. Every drone must be landed before the rain
reaches its limit level.

Time to home `H(u)`: if the return is commanded at show time u, the time until every drone is
landed, from the planned paths alone (Stage 2 timing, B5, and return durations, B4):

* in the transition into formation k (takeoff included):  H = (arrival_k - u) + D_k
* at formation k (between arrival and the next transition): H = D_k
* on the show's return leg:                                 H = end - u
* when formation k has no return path, the drones fly the rest of the show (its own return leg is
  checked and safe):                                         H = end - u
  (a show without a return leg can't do that; H is then unknown and the moment is not covered)
* after the end: 0 once the return leg has landed the fleet; without a return leg the fleet is still at
  the last formation, so H = D_last (or unknown)

With rain window W (alert to limit), reaction R and margin M, the alert at time t is covered when
R + H(t + R) + M <= W: H is taken when the command reaches the drones. `coverage()` gives the
uncovered intervals exactly; `H` is piecewise of slope 0 or -1.

`abort_plan()` / `compose()` build the reference an abort flight follows (B7): the show until the
drones reach formation k, a hold there until the return starts, the return path shifted to start then,
and the hold on the slots after it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

HOLDING = "holding_area"
RETURN_PATH = "return_path"       # a planned return (B4), or the takeoff flown backwards
RETURN_LEG = "return_leg"         # the show's own return leg, from the last formation
REST_OF_SHOW = "rest_of_show"     # no return path: keep flying the show to its return leg
NONE = "none"                     # no way home known
LANDED = "landed"                 # after the show's return leg: nothing left to do


@dataclass
class ShowTiming:
    """When each formation is reached and left (from the contract's `metadata.transitions`)."""

    names: list[str]
    start: list[float]          # start of the transition into formation k (k = 0: the takeoff)
    arrival: list[float]        # the fleet is at formation k
    leave: list[float]          # the next transition (or the return leg) starts
    end: float                  # every drone landed (or the show's last moment)
    return_leg: tuple[float, float] | None

    @classmethod
    def from_contract(cls, meta: dict[str, Any], names: list[str] | None = None) -> "ShowTiming":
        """`names`: the Phase 1 formations, checked against the transitions (default: taken from them)."""
        transitions = meta.get("transitions") or []
        if not transitions:
            raise ValueError("the show has no transition timing (metadata.transitions); plan Stage 2 again")
        into = [t for t in transitions if t["to_keyframe"] != HOLDING]
        names = [t["to_keyframe"] for t in into] if names is None else names
        if [t["to_keyframe"] for t in into] != list(names):
            raise ValueError("the show's transitions don't match its formations; plan Stage 2 again")
        leg = (meta.get("legs") or {}).get("return")
        end = float(meta["total_duration_sec"])
        leave = [float(into[k + 1]["start_time_sec"]) for k in range(len(into) - 1)]
        leave.append(float(leg["start_time_sec"]) if leg else end)
        return cls(names=list(names), start=[float(t["start_time_sec"]) for t in into],
                   arrival=[float(t["end_time_sec"]) for t in into], leave=leave, end=end,
                   return_leg=(float(leg["start_time_sec"]), float(leg["end_time_sec"])) if leg else None)


@dataclass
class Piece:
    """H(u) on [u0, u1): value0 at u0, slope 0 or -1. `value0` None = no way home."""

    u0: float
    u1: float                    # inf for the last piece (after the show's end)
    value0: float | None
    slope: float
    formation: int
    method: str

    def at(self, u: float) -> float | None:
        return None if self.value0 is None else self.value0 + self.slope * (u - self.u0)


def time_to_home(timing: ShowTiming, durations: dict[int, float]) -> list[Piece]:
    """H(u) over [0, end) as pieces. `durations[k]` = D_k, formation k's return duration (missing = none).
    The last formation's return is the show's return leg when there is one."""
    pieces: list[Piece] = []
    last = len(timing.names) - 1
    for k in range(len(timing.names)):
        d = durations.get(k)
        if k == last and timing.return_leg is not None:
            d, method = timing.return_leg[1] - timing.return_leg[0], RETURN_LEG
        elif d is not None:
            method = RETURN_PATH
        else:
            method = REST_OF_SHOW if timing.return_leg is not None else NONE
        a, b, c = timing.start[k], timing.arrival[k], timing.leave[k]
        if method in (RETURN_PATH, RETURN_LEG):
            pieces.append(Piece(a, b, (b - a) + d, -1.0, k, method))
            pieces.append(Piece(b, c, d, 0.0, k, method))
        elif method == REST_OF_SHOW:
            pieces.append(Piece(a, c, timing.end - a, -1.0, k, method))
        else:
            pieces.append(Piece(a, c, None, 0.0, k, method))
    if timing.return_leg is not None:
        s, e = timing.return_leg
        pieces.append(Piece(s, e, e - s, -1.0, last, RETURN_LEG))
        pieces.append(Piece(timing.end, math.inf, 0.0, 0.0, last, LANDED))
    else:
        d = durations.get(last)
        pieces.append(Piece(timing.end, math.inf, d, 0.0, last, RETURN_PATH if d is not None else NONE))
    return [p for p in pieces if p.u1 > p.u0]


def home_after(pieces: list[Piece], u: float, end: float) -> float | None:
    """H(u); before the show starts, the takeoff's value at its start. (`end` is kept for callers.)"""
    for p in pieces:
        if p.u0 <= u < p.u1:
            return p.at(u)
    return pieces[0].at(max(u, pieces[0].u0)) if pieces else None


@dataclass
class Coverage:
    window_sec: float | None
    reaction_sec: float
    margin_sec: float
    covered_fraction: float | None
    uncovered: list[dict[str, Any]] = field(default_factory=list)   # {start, end, short_by_sec, formation}
    required_window_sec: float | None = None                         # W_req; None: some moment can't be covered
    worst: dict[str, Any] | None = None                              # the moment of the largest R + H + M


def coverage(pieces: list[Piece], end: float, reaction: float, margin: float, window: float | None) -> Coverage:
    """Which alert times t in [0, end) are covered: R + H(t + R) + M <= W (exact)."""
    # The pieces in alert time t = u - R, clipped to [0, end); past `end` H is 0.
    spans: list[tuple[float, float, float | None, float, int]] = []
    for p in pieces:
        a, b = max(p.u0 - reaction, 0.0), min(p.u1 - reaction, end)
        if b > a:
            v = p.at(a + reaction)
            spans.append((a, b, None if v is None else reaction + v + margin, p.slope, p.formation))

    worst_value, worst = -math.inf, None
    impossible = False
    for a, b, c, _, k in spans:
        if c is None:
            impossible = True
        elif c > worst_value:
            worst_value, worst = c, {"time_sec": a, "needed_sec": c, "formation": k}
    required = None if impossible or worst is None else worst_value

    result = Coverage(window_sec=window, reaction_sec=reaction, margin_sec=margin, covered_fraction=None,
                      required_window_sec=required, worst=worst)
    if window is None or end <= 0:
        return result
    uncovered: list[dict[str, Any]] = []
    for a, b, c, slope, k in spans:
        if c is None:
            seg = (a, b, math.inf)
        elif c <= window:
            continue
        elif slope == 0.0:
            seg = (a, b, c - window)
        else:
            seg = (a, min(b, a + (c - window)), c - window)
        if seg[1] <= seg[0]:
            continue
        if uncovered and abs(uncovered[-1]["end"] - seg[0]) < 1e-9:
            uncovered[-1]["end"] = seg[1]
            uncovered[-1]["short_by_sec"] = max(uncovered[-1]["short_by_sec"], seg[2])
        else:
            uncovered.append({"start": seg[0], "end": seg[1], "short_by_sec": seg[2], "formation": k})
    total = sum(u["end"] - u["start"] for u in uncovered)
    result.uncovered = uncovered
    result.covered_fraction = 1.0 - total / end
    return result


# --------------------------------------------------------------------------- #
# Abort flights (B7)
# --------------------------------------------------------------------------- #

@dataclass
class AbortPlan:
    command_sec: float          # when the return reaches the drones
    formation: int              # the formation they return from (-1: nothing changes)
    start_sec: float            # when the flight home starts
    method: str                 # RETURN_PATH, RETURN_LEG (the show itself), REST_OF_SHOW, NONE
    planned_home_sec: float | None


def abort_plan(timing: ShowTiming, durations: dict[int, float], command: float) -> AbortPlan:
    """What the fleet does when the return is commanded at show time `command`."""
    pieces = time_to_home(timing, durations)
    u = max(command, 0.0)
    piece = next((p for p in pieces if p.u0 <= u < p.u1), pieces[0])
    k = piece.formation
    h = piece.at(max(u, piece.u0))
    if piece.method == LANDED:
        return AbortPlan(command, -1, command, LANDED, timing.end)
    if piece.method == RETURN_PATH:
        start = max(u, timing.arrival[k])
        return AbortPlan(command, k, start, RETURN_PATH, start + durations[k])
    if piece.method == RETURN_LEG:
        leg_start = timing.return_leg[0]
        if u >= leg_start:          # already flying home: nothing changes
            return AbortPlan(command, k, leg_start, RETURN_LEG, timing.end)
        start = max(u, timing.arrival[k])
        return AbortPlan(command, k, start, RETURN_LEG, start + (timing.return_leg[1] - leg_start))
    return AbortPlan(command, k, u, piece.method, None if h is None else u + h)


def _shift_segment(seg: dict[str, Any], dt: float) -> dict[str, Any]:
    return {**seg,
            "start_time_sec": seg["start_time_sec"] + dt,
            "end_time_sec": seg["end_time_sec"] + dt,
            "color_keyframes": [{**c, "time_sec": c["time_sec"] + dt} for c in seg.get("color_keyframes", [])]}


def leg_as_return(show: dict[str, Any], leg_start: float) -> dict[str, Any]:
    """The show's own return leg as a return-path contract timed from 0."""
    trajectories = [{"drone_id": t["drone_id"],
                     "segments": [_shift_segment(s, -leg_start) for s in t["segments"]
                                  if s["start_time_sec"] >= leg_start - 1e-9]} for t in show["trajectories"]]
    meta = {**show["metadata"], "total_duration_sec": float(show["metadata"]["total_duration_sec"]) - leg_start}
    return {"metadata": meta, "trajectories": trajectories}


def compose(show: dict[str, Any], ret: dict[str, Any], arrival: float, start: float) -> dict[str, Any]:
    """One contract: the show until `arrival` (the fleet at the formation), a hold there, then `ret` shifted
    to start at `start` (>= arrival). Between segments and after the last one a drone holds position."""
    returns = {t["drone_id"]: t["segments"] for t in ret["trajectories"]}
    trajectories = []
    for traj in show["trajectories"]:
        segments = [dict(s) for s in traj["segments"] if s["end_time_sec"] <= arrival + 1e-9]
        segments += [_shift_segment(s, start) for s in returns[traj["drone_id"]]]
        for i, seg in enumerate(segments):
            seg["segment_index"] = i
        trajectories.append({"drone_id": traj["drone_id"], "segments": segments})
    meta = {**show["metadata"], "total_duration_sec": start + float(ret["metadata"]["total_duration_sec"])}
    return {"metadata": meta, "trajectories": trajectories}


def abort_reference(show: dict[str, Any], timing: ShowTiming, returns: dict[int, dict[str, Any]],
                    plan: AbortPlan) -> dict[str, Any] | None:
    """The contract the fleet flies under `plan`; None when the plan doesn't change the show."""
    if plan.formation < 0 or plan.method in (REST_OF_SHOW, NONE):
        return None
    k = plan.formation
    if plan.method == RETURN_LEG:
        leg_start = timing.return_leg[0]
        if plan.start_sec >= leg_start - 1e-9:
            return None
        return compose(show, leg_as_return(show, leg_start), timing.arrival[k], plan.start_sec)
    return compose(show, returns[k], timing.arrival[k], plan.start_sec)
