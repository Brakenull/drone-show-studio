// Rain return coverage (docs/4-condition_simulator.md §5.2) for any rain window, so the window slider
// answers at once. Mirrors twin_sim/rain_return.py `coverage()`; the bridge's `readiness` command and
// tests/test_rain_return.py check the Python side, the Conditions tab shows both on the same pieces.

import type { HomePiece } from "../bridge/types";

export interface Span {
  /** Alert time t (show seconds): [a, b). */
  a: number;
  b: number;
  /** R + H(a + R) + M; null when there is no way home. */
  needed: number | null;
  slope: number;
  formation: number;
  method: HomePiece["method"];
  /** "abort_point": the show time its return starts from. */
  pointSec: number | null;
}

export interface CoverageResult {
  uncovered: { start: number; end: number; shortBy: number; formation: number }[];
  coveredFraction: number;
  /** W_req: the smallest window that covers the whole show; null if some moment has no way home. */
  required: number | null;
  worst: { time: number; needed: number; formation: number } | null;
}

/** The pieces of H(u) moved to alert time t = u - R and clipped to [0, end). */
export function spans(pieces: HomePiece[], end: number, reaction: number, margin: number): Span[] {
  const out: Span[] = [];
  for (const p of pieces) {
    const u1 = p.u1 ?? Infinity;
    const a = Math.max(p.u0 - reaction, 0);
    const b = Math.min(u1 - reaction, end);
    if (b <= a) continue;
    const v = p.value0 === null ? null : p.value0 + p.slope * (a + reaction - p.u0);
    out.push({ a, b, needed: v === null ? null : reaction + v + margin, slope: p.slope, formation: p.formation, method: p.method, pointSec: p.point_sec ?? null });
  }
  return out;
}

export function coverage(pieces: HomePiece[], end: number, reaction: number, margin: number, window: number): CoverageResult {
  const list = spans(pieces, end, reaction, margin);
  let worst: CoverageResult["worst"] = null;
  let impossible = false;
  for (const s of list) {
    if (s.needed === null) impossible = true;
    else if (!worst || s.needed > worst.needed) worst = { time: s.a, needed: s.needed, formation: s.formation };
  }
  const uncovered: CoverageResult["uncovered"] = [];
  for (const s of list) {
    let seg: [number, number, number];
    if (s.needed === null) seg = [s.a, s.b, Infinity];
    else if (s.needed <= window) continue;
    else if (s.slope === 0) seg = [s.a, s.b, s.needed - window];
    else seg = [s.a, Math.min(s.b, s.a + (s.needed - window)), s.needed - window];
    if (seg[1] <= seg[0]) continue;
    const last = uncovered[uncovered.length - 1];
    if (last && Math.abs(last.end - seg[0]) < 1e-9) {
      last.end = seg[1];
      last.shortBy = Math.max(last.shortBy, seg[2]);
    } else uncovered.push({ start: seg[0], end: seg[1], shortBy: seg[2], formation: s.formation });
  }
  const total = uncovered.reduce((sum, u) => sum + (u.end - u.start), 0);
  return {
    uncovered,
    coveredFraction: end > 0 ? 1 - total / end : 1,
    required: impossible || !worst ? null : worst.needed,
    worst,
  };
}
