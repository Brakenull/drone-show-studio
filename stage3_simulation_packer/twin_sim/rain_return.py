"""Rain rule: time to home, coverage and abort flights.

Rule: when the rain reaches its alert level, the return to the holding area is commanded `reaction`
seconds later. The drones finish the transition they are in (it is checked and safe) and fly the
planned return path of the formation it ends at; at a formation they start its return at once. From
the last formation the return is the show's own return leg. Every drone must be landed before the rain
reaches its limit level. Abort points are return paths planned from
moments inside a transition: the drones keep flying the show to the abort point ahead (or the formation)
whose return gets them home first, and fly that one from there.

Time to home `H(u)`: if the return is commanded at show time u, the time until every drone is
landed, from the planned paths alone (Stage 2 transition timing and return durations):

* in the transition into formation k (takeoff included):  H = (arrival_k - u) + D_k, or, with abort
  points p ahead in that transition, min over them of (time_p - u) + D_p when that is sooner
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

`abort_plan()` / `compose()` build the reference an abort flight follows: the show until the
drones reach formation k, a hold there until the return starts, the return path shifted to start then,
and the hold on the slots after it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

HOLDING = "holding_area"
RETURN_PATH = "return_path"       # a planned return, or the takeoff flown backwards
RETURN_LEG = "return_leg"         # the show's own return leg, from the last formation
REST_OF_SHOW = "rest_of_show"     # no return path: keep flying the show to its return leg
ABORT_POINT = "abort_point"       # a return path planned from a moment inside a transition
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
    point_sec: float | None = None   # ABORT_POINT: the show time its return starts from

    def at(self, u: float) -> float | None:
        return None if self.value0 is None else self.value0 + self.slope * (u - self.u0)


@dataclass(frozen=True)
class AbortPoint:
    """A return path planned from show time `time_sec`, inside the transition into formation `formation`."""

    formation: int
    time_sec: float
    duration_sec: float


def point_id(formation: int, time_sec: float) -> str:
    """The name of an abort point's files: formation and show time in milliseconds (`1-30500`)."""
    return f"{formation}-{round(time_sec * 1000)}"


def _merge(pieces: list[Piece]) -> list[Piece]:
    """Joins neighbours that are one straight piece (same formation, method, point and slope, continuous)."""
    out: list[Piece] = []
    for p in pieces:
        q = out[-1] if out else None
        if (q is not None and (q.formation, q.method, q.point_sec, q.slope) == (p.formation, p.method, p.point_sec,
                                                                                 p.slope)
                and abs(q.u1 - p.u0) < 1e-9 and (q.value0 is None) == (p.value0 is None)
                and (p.value0 is None or abs(q.at(p.u0) - p.value0) < 1e-9)):
            out[-1] = Piece(q.u0, p.u1, q.value0, q.slope, q.formation, q.method, q.point_sec)
        else:
            out.append(p)
    return out


def time_to_home(timing: ShowTiming, durations: dict[int, float],
                 points: list[AbortPoint] | tuple[AbortPoint, ...] = ()) -> list[Piece]:
    """H(u) over [0, end) as pieces. `durations[k]` = D_k, formation k's return duration (missing = none).
    The last formation's return is the show's return leg when there is one. `points`: abort points; one
    outside its transition is ignored."""
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
        # Every way home from the transition into k as (show time it starts from, home time, method, point):
        # an abort point, or going on to the formation. H(u) = home - u for the soonest one ahead of u.
        home = {RETURN_PATH: b + (d or 0.0), RETURN_LEG: b + (d or 0.0), REST_OF_SHOW: timing.end}.get(method)
        ways = sorted((p.time_sec, p.time_sec + p.duration_sec, ABORT_POINT, p.time_sec)
                      for p in points if p.formation == k and a < p.time_sec < b)
        ways.append((b, home, method, None))
        u0 = a
        for i, (t_i, _, _, _) in enumerate(ways):
            if t_i <= u0:
                continue
            best = min((w for w in ways[i:] if w[1] is not None), key=lambda w: w[1], default=None)
            if best is None:
                pieces.append(Piece(u0, t_i, None, 0.0, k, method))
            else:
                pieces.append(Piece(u0, t_i, best[1] - u0, -1.0, k, best[2], best[3]))
            u0 = t_i
        if method in (RETURN_PATH, RETURN_LEG):
            pieces.append(Piece(b, c, d, 0.0, k, method))
        elif method == REST_OF_SHOW:
            pieces.append(Piece(b, c, timing.end - b, -1.0, k, method))
        else:
            pieces.append(Piece(b, c, None, 0.0, k, method))
    if timing.return_leg is not None:
        s, e = timing.return_leg
        pieces.append(Piece(s, e, e - s, -1.0, last, RETURN_LEG))
        pieces.append(Piece(timing.end, math.inf, 0.0, 0.0, last, LANDED))
    else:
        d = durations.get(last)
        pieces.append(Piece(timing.end, math.inf, d, 0.0, last, RETURN_PATH if d is not None else NONE))
    return _merge([p for p in pieces if p.u1 > p.u0])


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
# Abort flights
# --------------------------------------------------------------------------- #

@dataclass
class AbortPlan:
    command_sec: float          # when the return reaches the drones
    formation: int              # the formation they return from, or the one whose transition holds the
                                # abort point (-1: nothing changes)
    start_sec: float            # when the flight home starts (an abort point: its show time)
    method: str                 # RETURN_PATH, ABORT_POINT, RETURN_LEG (the show itself), REST_OF_SHOW, NONE
    planned_home_sec: float | None


def abort_plan(timing: ShowTiming, durations: dict[int, float], command: float,
               points: list[AbortPoint] | tuple[AbortPoint, ...] = ()) -> AbortPlan:
    """What the fleet does when the return is commanded at show time `command`."""
    pieces = time_to_home(timing, durations, points)
    u = max(command, 0.0)
    piece = next((p for p in pieces if p.u0 <= u < p.u1), pieces[0])
    k = piece.formation
    h = piece.at(max(u, piece.u0))
    if piece.method == LANDED:
        return AbortPlan(command, -1, command, LANDED, timing.end)
    if piece.method == ABORT_POINT:
        return AbortPlan(command, k, piece.point_sec, ABORT_POINT, max(u, piece.u0) + h)
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


def _insert_knot(knots: np.ndarray, cps: np.ndarray, degree: int, u: float) -> tuple[np.ndarray, np.ndarray]:
    """Boehm's knot insertion: the same curve with one more knot at u."""
    k = int(np.searchsorted(knots, u, side="right")) - 1
    new = np.empty((cps.shape[0] + 1, cps.shape[1]))
    new[:k - degree + 1] = cps[:k - degree + 1]
    for i in range(k - degree + 1, k + 1):
        alpha = (u - knots[i]) / (knots[i + degree] - knots[i])
        new[i] = alpha * cps[i] + (1.0 - alpha) * cps[i - 1]
    new[k + 1:] = cps[k:]
    return np.insert(knots, k + 1, u), new


def cut_segment(seg: dict[str, Any], t: float) -> dict[str, Any]:
    """The part of a contract segment before show time t (start < t < end), exact: the knot t is inserted
    until it splits the curve."""
    knots = np.asarray(seg["knot_vector"], dtype=float)
    cps = np.asarray(seg["control_points"], dtype=float)
    degree = knots.size - cps.shape[0] - 1
    u = t - seg["start_time_sec"] + knots[0]      # local knots start at 0, absolute ones at the start time
    near = np.abs(knots - u) < 1e-9
    knots = np.where(near, u, knots)              # a knot within 1 ns of the cut is the cut
    while np.count_nonzero(knots == u) < degree:
        knots, cps = _insert_knot(knots, cps, degree, u)
    j = int(np.flatnonzero(knots == u)[0])        # u now fills knots[j : j + degree]
    keys = seg.get("color_keyframes", [])
    colors = [c for c in keys if c["time_sec"] < t - 1e-9]
    after = next((c for c in keys if c["time_sec"] >= t - 1e-9), None)
    if after is not None:
        if colors:
            before = colors[-1]
            span = after["time_sec"] - before["time_sec"]
            f = (t - before["time_sec"]) / span if span > 1e-12 else 1.0
            rgb = [int(round(a + (b - a) * f)) for a, b in zip(before["color_rgb"], after["color_rgb"])]
        else:
            rgb = list(after["color_rgb"])
        colors.append({"time_sec": t, "color_rgb": rgb})
    return {**seg, "end_time_sec": t, "knot_vector": [*knots[:j + degree].tolist(), u],
            "control_points": cps[:j].tolist(), "color_keyframes": colors}


def compose(show: dict[str, Any], ret: dict[str, Any], arrival: float, start: float) -> dict[str, Any]:
    """One contract: the show until `arrival` (the fleet at the formation, or at an abort point inside a
    transition, where a segment is cut), a hold there, then `ret` shifted to start at `start` (>= arrival).
    Between segments and after the last one a drone holds position."""
    returns = {t["drone_id"]: t["segments"] for t in ret["trajectories"]}
    trajectories = []
    for traj in show["trajectories"]:
        segments = []
        for s in traj["segments"]:
            if s["end_time_sec"] <= arrival + 1e-9:
                segments.append(dict(s))
            elif s["start_time_sec"] < arrival - 1e-9:
                segments.append(cut_segment(s, arrival))
        segments += [_shift_segment(s, start) for s in returns[traj["drone_id"]]]
        for i, seg in enumerate(segments):
            seg["segment_index"] = i
        trajectories.append({"drone_id": traj["drone_id"], "segments": segments})
    meta = {**show["metadata"], "total_duration_sec": start + float(ret["metadata"]["total_duration_sec"])}
    return {"metadata": meta, "trajectories": trajectories}


def abort_reference(show: dict[str, Any], timing: ShowTiming, returns: dict[int, dict[str, Any]],
                    plan: AbortPlan, point_returns: dict[str, dict[str, Any]] | None = None) -> dict[str, Any] | None:
    """The contract the fleet flies under `plan`; None when the plan doesn't change the show.
    `point_returns`: the abort points' return contracts by `point_id()`."""
    if plan.formation < 0 or plan.method in (REST_OF_SHOW, NONE):
        return None
    k = plan.formation
    if plan.method == ABORT_POINT:
        return compose(show, (point_returns or {})[point_id(k, plan.start_sec)], plan.start_sec, plan.start_sec)
    if plan.method == RETURN_LEG:
        leg_start = timing.return_leg[0]
        if plan.start_sec >= leg_start - 1e-9:
            return None
        return compose(show, leg_as_return(show, leg_start), timing.arrival[k], plan.start_sec)
    return compose(show, returns[k], timing.arrival[k], plan.start_sec)


# --------------------------------------------------------------------------- #
# Suggestions when a moment is not covered
# --------------------------------------------------------------------------- #

EARLIER_TRIGGER = "earlier_trigger"   # start the return before the rain alert (forecast, or a lower alert level)
ABORT_POINTS = "abort_points"         # plan returns from moments inside a transition
PLAN_RETURN = "plan_return"           # a formation without a return path: plan one
FASTER_RETURN = "faster_return"       # re-plan a return at its Auto duration
DESIGN = "design"                     # the formation whose return drives the gap: a Phase 1 change


@dataclass
class ReturnFacts:
    """What the suggestions know about formation k's flight home."""

    planned_sec: float | None = None      # D_k as planned and flown; None: no return path
    min_sec: float | None = None          # its Auto duration (T_min + final descent), planned or estimated
    target_sec: float | None = None       # the target it was planned with; None: Auto
    attempts: int | None = None
    farthest_drone: int | None = None     # the drone with the longest way home, which sets T_min
    farthest_m: float | None = None
    farthest_area: int | None = None      # the holding area it lands in (0-based); None with one area
    reversed_takeoff: bool = False        # the takeoff flown backwards: nothing to re-plan


def _alert_spans(pieces: list[Piece], end: float, reaction: float) -> list[tuple[float, float, Piece]]:
    """Each piece as the alert times t = u - R it answers, clipped to [0, end)."""
    out = []
    for p in pieces:
        a, b = max(p.u0 - reaction, 0.0), min(p.u1 - reaction, end)
        if b > a:
            out.append((a, b, p))
    return out


def _overlap(cov: Coverage, a: float, b: float) -> float:
    return sum(max(0.0, min(b, u["end"]) - max(a, u["start"])) for u in cov.uncovered)


def suggestions(timing: ShowTiming, durations: dict[int, float], points: list[AbortPoint], reaction: float,
                margin: float, window: float, facts: dict[int, ReturnFacts] | None = None,
                candidates: list[AbortPoint] | None = None,
                rule: dict[str, float] | None = None) -> dict[str, Any]:
    """The options for a rain window, each with its numbers and the uncovered alert time it
    closes on its own, ranked by that.

    `facts`: per formation, its return's planned and Auto durations (or an estimate when it has none).
    `candidates`: abort points not planned yet, each with its estimated duration (the Auto duration of a
    return from there); the "more abort points" option picks those that close uncovered time.
    `rule`: the scenario's alert and limit levels, to express the earlier trigger as a lower alert level.
    Durations that come from estimates make the option an estimate until Stage 2 plans it."""
    facts = facts or {}
    candidates = candidates or []
    end = timing.end

    def evaluate(d: dict[int, float], p: list[AbortPoint], w: float = window) -> tuple[float, Coverage]:
        cov = coverage(time_to_home(timing, d, p), end, reaction, margin, w)
        return sum(u["end"] - u["start"] for u in cov.uncovered), cov

    pieces = time_to_home(timing, durations, points)
    before, base = evaluate(durations, points)
    result: dict[str, Any] = {"window_sec": window, "uncovered_sec": before, "items": []}
    if before <= 1e-9:
        return result

    # Who drives the gap: the pieces with uncovered alert times, by formation and by part (the move into the
    # formation, or holding at it), with the largest excess over the window.
    driving: dict[int, dict[str, Any]] = {}
    for a, b, p in _alert_spans(pieces, end, reaction):
        gap = _overlap(base, a, b)
        if gap <= 1e-9:
            continue
        d = driving.setdefault(p.formation, {"gap": 0.0, "methods": set(), "transition": False, "gap_by": {}})
        d["gap"] += gap
        d["methods"].add(p.method)
        d["gap_by"][p.method] = d["gap_by"].get(p.method, 0.0) + gap
        if p.u0 < timing.arrival[p.formation] - 1e-9:     # in the move into the formation
            d["transition"] = True

    items: list[dict[str, Any]] = []

    def add(kind: str, k: int | None, closes: float, *, estimate: bool, action: dict[str, Any] | None,
            **numbers: Any) -> None:
        items.append({"kind": kind, "formation": k, "formation_name": None if k is None else timing.names[k],
                      "closes_sec": max(0.0, closes), "closes_all": closes >= before - 1e-6,
                      "estimate": estimate, "action": action, "numbers": numbers})

    # Earlier trigger: start the return `lead` seconds before the alert, as if the window were that much
    # longer. With the rain rising steadily from the alert to the limit level in `window` seconds, that is
    # the same as an alert level reached `lead` seconds earlier.
    finite = [u["short_by_sec"] for u in base.uncovered if math.isfinite(u["short_by_sec"])]
    if finite:
        lead = max(finite)
        after, _ = evaluate(durations, points, window + lead)
        level = None
        if rule and window > 0:
            alert, limit = rule["alert_mm_h"], rule["limit_mm_h"]
            level = alert - (limit - alert) * lead / window
            level = round(level, 2) if level >= 0.05 else None
        add(EARLIER_TRIGGER, None, before - after, estimate=False,
            action={"kind": "set_alert", "alert_mm_h": level} if level is not None else None,
            lead_sec=lead, alert_mm_h=level, current_alert_mm_h=rule["alert_mm_h"] if rule else None)

    planned_near = lambda k, t: any(p.formation == k and abs(p.time_sec - t) < 0.5 for p in points)  # noqa: E731
    for k in sorted(driving):
        d = driving[k]
        f = facts.get(k, ReturnFacts())

        # More abort points inside the move into k: add candidates in time order while each closes more,
        # then drop any that a later one made unnecessary.
        if d["transition"]:
            pool = sorted((c for c in candidates if c.formation == k and not planned_near(k, c.time_sec)),
                          key=lambda c: c.time_sec)
            chosen: list[AbortPoint] = []
            current = before
            for c in pool:
                trial, _ = evaluate(durations, points + chosen + [c])
                if trial < current - 1e-6:
                    chosen.append(c)
                    current = trial
            for c in list(chosen):
                rest = [x for x in chosen if x is not c]
                trial, _ = evaluate(durations, points + rest)
                if trial <= current + 1e-6:
                    chosen, current = rest, trial
            if chosen:
                add(ABORT_POINTS, k, before - current, estimate=True,
                    action={"kind": "plan_points", "points": [[c.formation, c.time_sec] for c in chosen]},
                    points=[{"time_sec": c.time_sec, "duration_sec": c.duration_sec} for c in chosen])

        # A formation without a return path: the fleet flies the rest of the show from there.
        if d["methods"] & {REST_OF_SHOW, NONE} and f.planned_sec is None and f.min_sec is not None:
            after, _ = evaluate({**durations, k: f.min_sec}, points)
            add(PLAN_RETURN, k, before - after, estimate=True, action={"kind": "plan_return", "formation": k},
                duration_sec=f.min_sec)

        if RETURN_PATH in d["methods"] and f.planned_sec is not None and not f.reversed_takeoff:
            gain = None if f.min_sec is None else f.planned_sec - f.min_sec
            if gain is not None and gain > 0.5:
                after, _ = evaluate({**durations, k: f.min_sec}, points)
                add(FASTER_RETURN, k, before - after, estimate=True,
                    action={"kind": "replan_return", "formation": k}, possible=True, return_sec=f.planned_sec,
                    new_sec=f.min_sec, target_sec=f.target_sec, attempts=f.attempts)
            elif gain is None and f.target_sec is not None:
                add(FASTER_RETURN, k, 0.0, estimate=True, action={"kind": "replan_return", "formation": k},
                    possible=True, return_sec=f.planned_sec, new_sec=None, target_sec=f.target_sec,
                    attempts=f.attempts)
            else:
                add(FASTER_RETURN, k, 0.0, estimate=False, action=None, possible=False, return_sec=f.planned_sec,
                    farthest_drone=f.farthest_drone, farthest_m=f.farthest_m,
                    farthest_area=f.farthest_area)

        # The formation whose own return drives the gap: how much shorter it must be (a Phase 1 change:
        # bring the formation closer to the holding area, lower it, or move it earlier in the show).
        own = d["gap_by"].get(RETURN_PATH, 0.0) + d["gap_by"].get(RETURN_LEG, 0.0)
        if own > 1e-9:
            leg = k == len(timing.names) - 1 and timing.return_leg is not None
            d_k = (timing.return_leg[1] - timing.return_leg[0]) if leg else durations.get(k)
            worst = max(((reaction + v + margin, a + reaction) for a, b, p in _alert_spans(pieces, end, reaction)
                         if p.formation == k and p.method in (RETURN_PATH, RETURN_LEG)
                         and _overlap(base, a, b) > 1e-9 and (v := p.at(a + reaction)) is not None),
                        default=None)
            if d_k is not None and worst is not None:
                short = worst[0] - window
                u = worst[1]
                add(DESIGN, k, own, estimate=False, action=None, return_sec=d_k, short_sec=short,
                    return_needed_sec=d_k - short, move_sec=max(0.0, timing.arrival[k] - u), return_leg=leg,
                    farthest_drone=f.farthest_drone, farthest_m=f.farthest_m,
                    farthest_area=f.farthest_area)

    order = {EARLIER_TRIGGER: 3, ABORT_POINTS: 0, PLAN_RETURN: 1, FASTER_RETURN: 2, DESIGN: 4}
    items.sort(key=lambda i: (-round(i["closes_sec"], 6), order[i["kind"]]))
    result["items"] = items
    return result
