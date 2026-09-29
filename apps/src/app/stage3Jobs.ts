// Live Stage 3 jobs (Monte Carlo, pack) keyed by run id and part. Lives outside React, like
// stage2Jobs.ts, so a long stress test keeps streaming while the user looks at other runs or views.

import { useSyncExternalStore } from "react";
import { cancelJob, startJob } from "../bridge/api";
import type { BridgeEvent, JobExit, McRecord, PackSummary } from "../bridge/types";

export type Stage3Part = "monte_carlo" | "pack";

export interface Stage3Job {
  jobId: number;
  startedAt: number; // ms epoch
  phase: string;
  /** Monte Carlo: the planned run count and the flights finished so far (nominal first, run -1). */
  planned: { runs: number; device: string } | null;
  records: McRecord[];
  /** When the calm-air (nominal) flight landed, ms epoch: the scenario flights start after it. */
  nominalAt: number | null;
  pack: (PackSummary & { ok: boolean }) | null;
  log: string[];
  errors: string[];
  exit: JobExit | null;
  cancelling: boolean;
}

const MAX_LOG = 400;
const key = (runId: string, part: Stage3Part) => `${runId}:${part}`;
let jobs: Record<string, Stage3Job> = {};
const subscribers = new Set<() => void>();

function set(k: string, patch: Partial<Stage3Job>) {
  const current = jobs[k];
  if (!current) return;
  jobs = { ...jobs, [k]: { ...current, ...patch } };
  subscribers.forEach((fn) => fn());
}

function onEvent(k: string, e: BridgeEvent) {
  const job = jobs[k];
  if (!job) return;
  if (e.type === "phase") set(k, { phase: e.name });
  else if (e.type === "mc_start") set(k, { planned: { runs: e.runs, device: e.device } });
  else if (e.type === "mc_run") {
    const { type: _type, ...record } = e;
    set(k, { records: [...job.records, record], ...(record.run < 0 ? { nominalAt: Date.now() } : {}) });
  } else if (e.type === "pack_result") {
    const { type: _type, ...result } = e;
    set(k, { pack: result });
  } else if (e.type === "error") set(k, { errors: [...job.errors, e.message], log: [...job.log, e.message] });
}

/** Starts `monte_carlo` or `pack` on a run; `args` are the command's options after the run folder. */
export async function startStage3(
  runId: string,
  runDir: string,
  part: Stage3Part,
  args: string[],
  onFinished: (exit: JobExit) => void,
): Promise<void> {
  const k = key(runId, part);
  // Create the entry first: the bridge's first events can arrive before startJob resolves.
  jobs = {
    ...jobs,
    [k]: {
      jobId: -1,
      startedAt: Date.now(),
      phase: "starting",
      planned: null,
      records: [],
      nominalAt: null,
      pack: null,
      log: [],
      errors: [],
      exit: null,
      cancelling: false,
    },
  };
  subscribers.forEach((fn) => fn());
  let job;
  try {
    job = await startJob([part, runDir, ...args], {
      run: { dir: runDir, section: part },
      onEvent: (e) => onEvent(k, e),
      onLog: (line) => set(k, { log: [...(jobs[k]?.log ?? []), line].slice(-MAX_LOG) }),
    });
  } catch (err) {
    const exit = { code: null, cancelled: false };
    set(k, { exit, errors: [String(err)] });
    onFinished(exit);
    return;
  }
  set(k, { jobId: job.id });
  const exit = await job.done;
  set(k, { exit, cancelling: false });
  onFinished(exit);
}

export async function cancelStage3(runId: string, part: Stage3Part): Promise<void> {
  const k = key(runId, part);
  const job = jobs[k];
  if (!job || job.exit || job.jobId < 0) return;
  set(k, { cancelling: true });
  try {
    await cancelJob(job.jobId);
  } catch (err) {
    set(k, { cancelling: false });
    throw err;
  }
}

export function isStage3Running(runId: string): boolean {
  return (["monte_carlo", "pack"] as const).some((p) => {
    const job = jobs[key(runId, p)];
    return !!job && !job.exit;
  });
}

export function useStage3Job(runId: string | null, part: Stage3Part): Stage3Job | null {
  return useSyncExternalStore(
    (fn) => {
      subscribers.add(fn);
      return () => subscribers.delete(fn);
    },
    () => (runId ? (jobs[key(runId, part)] ?? null) : null),
  );
}

/** Rough time left from the observed pace since the calm-air flight. Flights are simulated in batches
 *  that land together, so the estimate moves in steps. Null until the first batch lands. */
export function secondsLeft(job: Stage3Job, now: number): number | null {
  const done = job.records.filter((r) => r.run >= 0).length;
  if (!job.planned || !job.nominalAt || done === 0) return null;
  return (((now - job.nominalAt) / 1000) * Math.max(0, job.planned.runs - done)) / done;
}
