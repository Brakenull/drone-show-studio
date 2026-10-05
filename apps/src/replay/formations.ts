// Formation marks: where each Phase 1 formation falls in the planned show,
// against its time in the Blender export.

import { formatTime } from "./sampling";
import type { FormationMark } from "./types";

/** How much later than designed the planned show reaches it, e.g. "+58.2 s", or null when unknown. */
export function formationDelay(f: FormationMark): string | null {
  if (f.designed_sec === null || f.designed_sec === undefined) return null;
  const d = f.reached_sec - f.designed_sec;
  if (Math.abs(d) < 0.05) return "as designed";
  return `${d > 0 ? "+" : "−"}${Math.abs(d).toFixed(1)} s`;
}

/** "Shape_679 · reached at 1:26.5 · designed at 0:28.3 in Blender (+58.2 s)" */
export function formationDetail(f: FormationMark): string {
  const parts = [
    f.name,
    f.rejected ? `the rejected attempt ends at ${formatTime(f.reached_sec)}` : `reached at ${formatTime(f.reached_sec)}`,
  ];
  if (f.leaves_sec !== null && f.leaves_sec - f.reached_sec >= 0.05)
    parts.push(`held until ${formatTime(f.leaves_sec)}`);
  if (f.designed_sec !== null && f.designed_sec !== undefined) {
    const delay = formationDelay(f);
    parts.push(`designed at ${formatTime(f.designed_sec)} in Blender${delay && delay !== "as designed" ? ` (${delay})` : ""}`);
  }
  return parts.join(" · ");
}

/** The formation the show is at or last reached at time `t`, or null before the first one. */
export function currentFormation(marks: FormationMark[], t: number): FormationMark | null {
  let out: FormationMark | null = null;
  for (const f of marks) if (f.reached_sec <= t + 1e-3) out = f;
  return out;
}
