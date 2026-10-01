// One-line state of each part of a run, for the stage cards, the Stage 3 sections and the sidebar's
// pipeline bar (docs/5-studio_gui.md §6, "Run header").

import type { RunRecord, Stage3Part as PartRecord } from "../bridge/types";
import { STATUS, duration, type Tone } from "./format";
import { isStage2Running } from "./stage2Jobs";
import { isStage3PartRunning, isStage3Running } from "./stage3Jobs";
import { isSimulateRunning } from "./simulateJobs";
import { metres } from "../replay/sampling";

export interface StageState {
  tone: Tone;
  text: string;
}

const idle = (text: string): StageState => ({ tone: "idle", text });
const plain = (status: RunRecord["stage2"]["status"]): StageState => ({
  tone: STATUS[status].tone,
  text: STATUS[status].label,
});

/** Made from an earlier Stage 2 result than the one the run has now. */
export const isStale = (run: RunRecord, part: { stage2_ended_at?: string | null } | undefined) =>
  !!part?.stage2_ended_at && part.stage2_ended_at !== run.stage2.ended_at;

const stage2Passed = (run: RunRecord) => run.stage2.status === "succeeded" && !isStage2Running(run.run_id);

const EARLIER: StageState = { tone: "warn", text: "From an earlier Stage 2" };

export function inputState(): StageState {
  // A run folder is only created from a file that validated.
  return { tone: "ok", text: "File checked" };
}

export function stage2State(run: RunRecord): StageState {
  if (isStage2Running(run.run_id) || run.stage2.status === "running") return { tone: "busy", text: "Running" };
  const s = run.stage2;
  if (s.status === "succeeded") return { tone: "ok", text: `Passed, ${duration(s.total_duration_sec)} show` };
  if (s.status === "failed_safety")
    return {
      tone: "bad",
      text: s.worst_separation_m != null ? `Rejected, ${metres(s.worst_separation_m, 2)} closest` : "Rejected",
    };
  return plain(s.status);
}

export function stressState(run: RunRecord): StageState {
  if (!stage2Passed(run)) return idle("Needs Stage 2 pass");
  if (isStage3PartRunning(run.run_id, "monte_carlo")) return { tone: "busy", text: "Running" };
  const mc = run.stage3?.monte_carlo;
  if (!mc || mc.status === "not_run" || mc.status === "running") return idle("Not run");
  if (mc.status === "succeeded" || mc.status === "failed_safety") {
    if (isStale(run, mc)) return EARLIER;
    const n = mc.config?.runs;
    return mc.passed
      ? { tone: "ok", text: n ? `All ${n} flights held` : "Passed" }
      : { tone: "bad", text: "Failed" };
  }
  return plain(mc.status);
}

export function conditionsState(run: RunRecord): StageState {
  if (!stage2Passed(run)) return idle("Needs Stage 2 pass");
  if (isSimulateRunning(run.run_id)) return { tone: "busy", text: "Simulating" };
  const done = Object.values(run.conditions?.scenarios ?? {}).filter(
    (s: PartRecord & { passed?: boolean }) => s.passed !== undefined,
  ) as (PartRecord & { passed?: boolean })[];
  if (!done.length) return idle("No scenario flown");
  const failed = done.filter((s) => !s.passed).length;
  if (failed) return { tone: "bad", text: `${failed} of ${done.length} failed` };
  if (done.some((s) => isStale(run, s))) return EARLIER;
  return { tone: "ok", text: done.length === 1 ? "1 scenario held" : `${done.length} scenarios held` };
}

export function packState(run: RunRecord): StageState {
  if (!stage2Passed(run)) return idle("Needs Stage 2 pass");
  if (isStage3PartRunning(run.run_id, "pack")) return { tone: "busy", text: "Packing" };
  const p = run.stage3?.pack;
  if (!p || p.status === "not_run" || p.status === "running") return idle("Not packed");
  if (p.status === "succeeded") return isStale(run, p) ? EARLIER : { tone: "ok", text: `${p.files ?? ""} files packed`.trim() };
  return plain(p.status);
}

/** The Stage 3 card: stress test, weather scenarios and flight files together, worst first. */
export function stage3State(run: RunRecord): StageState {
  if (!stage2Passed(run)) return idle("Needs Stage 2 pass");
  if (isStage3Running(run.run_id) || isSimulateRunning(run.run_id)) return { tone: "busy", text: "Running" };
  const mc = stressState(run);
  const cond = conditionsState(run);
  const pack = packState(run);
  if (mc.tone === "bad") return { tone: "bad", text: "Stress test failed" };
  if (cond.tone === "bad") return { tone: "bad", text: "A scenario failed" };
  if (pack.tone === "bad" || pack.tone === "warn") return { ...pack, text: pack.text === EARLIER.text ? "Out of date" : `Packing: ${pack.text}` };
  if (mc.tone === "warn" || cond.tone === "warn") return { tone: "warn", text: "Out of date" };
  if (mc.tone === "idle" && cond.tone === "idle" && pack.tone === "idle") return idle("Not run");
  return { tone: "ok", text: pack.tone === "ok" ? "Tested and packed" : "Tested, not packed" };
}

export function replayState(run: RunRecord): StageState {
  return run.stage2.status === "succeeded" || run.stage2.status === "failed_safety"
    ? { tone: "ok", text: "Ready" }
    : idle("Not built");
}

export function compareState(run: RunRecord, runs: RunRecord[]): StageState {
  const n = runs.filter(
    (r) => r.run_id !== run.run_id && (r.stage2.status === "succeeded" || r.stage2.status === "failed_safety"),
  ).length;
  return idle(n === 1 ? "1 other run" : `${n} other runs`);
}
