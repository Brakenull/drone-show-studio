// Live "Plan return paths" jobs (bridge `stage2-returns`, docs/4-condition_simulator.md B4) keyed by
// run id. Lives outside React, like stage2Jobs, so a job keeps streaming while the user looks elsewhere.

import { useSyncExternalStore } from "react";
import { cancelJob, startJob } from "../bridge/api";
import type { BridgeEvent, JobExit, ReturnEntry } from "../bridge/types";
import { nextSolve, type SolveState } from "./stage2Jobs";

export interface ReturnsJob {
  jobId: number;
  startedAt: number; // ms epoch
  /** The return being planned: position in the job (0-based), job size, formation index. */
  current: { index: number; count: number; keyframe: number } | null;
  solve: SolveState | null;
  /** Returns this job has finished, in order. */
  results: ReturnEntry[];
  errors: string[];
  exit: JobExit | null;
  cancelling: boolean;
}

let jobs: Record<string, ReturnsJob> = {};
const subscribers = new Set<() => void>();

function set(runId: string, patch: Partial<ReturnsJob>) {
  const current = jobs[runId];
  if (!current) return;
  jobs = { ...jobs, [runId]: { ...current, ...patch } };
  subscribers.forEach((fn) => fn());
}

/** `formations` null = the bridge's default (every formation without a return leg). */
export async function startReturns(
  runId: string,
  runDir: string,
  formations: number[] | null,
  onFinished: (exit: JobExit) => void,
): Promise<void> {
  const onEvent = (e: BridgeEvent) => {
    const job = jobs[runId];
    if (!job) return;
    if (e.type === "solve_progress") {
      const fresh = e.return_index !== job.current?.index;
      set(runId, {
        current:
          e.return_index === undefined
            ? job.current
            : { index: e.return_index, count: e.return_count ?? 1, keyframe: e.keyframe_index ?? -1 },
        solve: nextSolve(fresh ? null : job.solve, e),
      });
    } else if (e.type === "return_result") {
      const { type: _type, ...entry } = e;
      set(runId, { results: [...job.results, entry] });
    } else if (e.type === "error") set(runId, { errors: [...job.errors, e.message] });
  };
  jobs = {
    ...jobs,
    [runId]: {
      jobId: -1,
      startedAt: Date.now(),
      current: null,
      solve: null,
      results: [],
      errors: [],
      exit: null,
      cancelling: false,
    },
  };
  subscribers.forEach((fn) => fn());
  let job;
  try {
    const args = ["stage2-returns", runDir, ...(formations ? ["--formations", formations.join(",")] : [])];
    job = await startJob(args, { run: { dir: runDir, section: "stage2_returns" }, onEvent });
  } catch (err) {
    const exit = { code: null, cancelled: false };
    set(runId, { exit, errors: [String(err)] });
    onFinished(exit);
    return;
  }
  set(runId, { jobId: job.id });
  const exit = await job.done;
  set(runId, { exit, cancelling: false });
  onFinished(exit);
}

export async function cancelReturns(runId: string): Promise<void> {
  const job = jobs[runId];
  if (!job || job.exit || job.jobId < 0) return;
  set(runId, { cancelling: true });
  try {
    await cancelJob(job.jobId);
  } catch (err) {
    set(runId, { cancelling: false });
    throw err;
  }
}

export function isReturnsRunning(runId: string): boolean {
  const job = jobs[runId];
  return !!job && !job.exit;
}

export function useReturnsJob(runId: string | null): ReturnsJob | null {
  return useSyncExternalStore(
    (fn) => {
      subscribers.add(fn);
      return () => subscribers.delete(fn);
    },
    () => (runId ? (jobs[runId] ?? null) : null),
  );
}
