// Live Stage 2 jobs keyed by run id. Lives outside React so a solve keeps streaming while the
// user looks at other runs or views.

import { useSyncExternalStore } from "react";
import { cancelJob, startJob } from "../bridge/api";
import type { BridgeEvent, JobExit, SolveProgress } from "../bridge/types";

/** Where the solver is, from drone_core's progress events (docs/5-studio_gui.md §5.2). */
export interface SolveState {
  transition: number;
  transitionCount: number;
  from: string;
  to: string;
  attempt: number;
  maxAttempts: number;
  substage: number;
  substageCount: number;
  iteration: number;
  maxIterations: number;
  /** Attempts the gatekeeper turned down in the current transition (each one is retried). */
  rejected: { attempt: number; worst: number | null; required: number }[];
}

/** Rough share of the solve done, 0..1: whole transitions, then sub-stages, then SCP iterations. */
export function solveFraction(s: SolveState): number {
  const stages = Math.max(s.substageCount, 1);
  const inStage = s.maxIterations ? Math.min(s.iteration / s.maxIterations, 1) : 0;
  const inTransition = s.substage ? (s.substage - 1 + inStage) / stages : 0;
  return Math.min((s.transition + inTransition) / Math.max(s.transitionCount, 1), 1);
}

function nextSolve(prev: SolveState | null, e: SolveProgress): SolveState {
  const base: SolveState =
    prev && prev.transition === e.transition
      ? prev
      : {
          transition: e.transition,
          transitionCount: e.transition_count,
          from: e.from_keyframe,
          to: e.to_keyframe,
          attempt: 0,
          maxAttempts: 0,
          substage: 0,
          substageCount: 0,
          iteration: 0,
          maxIterations: 0,
          rejected: [],
        };
  switch (e.event) {
    case "transition_start":
      return base;
    case "transition_end":
      return { ...base, substage: base.substageCount, iteration: base.maxIterations };
    case "attempt_start":
      return { ...base, attempt: e.attempt, maxAttempts: e.max_attempts, substage: 0, iteration: 0 };
    case "scp_iteration":
      return {
        ...base,
        attempt: e.attempt,
        maxAttempts: e.max_attempts,
        substage: e.substage,
        substageCount: e.substage_count,
        iteration: e.iteration,
        maxIterations: e.max_iterations,
      };
    case "attempt_end":
      return e.passed
        ? base
        : {
            ...base,
            rejected: [
              ...base.rejected,
              { attempt: e.attempt, worst: e.worst_separation_m, required: e.required_separation_m },
            ],
          };
  }
}

export interface Stage2Job {
  jobId: number;
  startedAt: number; // ms epoch
  phase: string;
  detail: string;
  progress: { done: number; total: number } | null;
  solve: SolveState | null;
  log: string[];
  errors: string[];
  exit: JobExit | null;
  cancelling: boolean;
}

const MAX_LOG = 400;
let jobs: Record<string, Stage2Job> = {};
const subscribers = new Set<() => void>();

function set(runId: string, patch: Partial<Stage2Job>) {
  const current = jobs[runId];
  if (!current) return;
  jobs = { ...jobs, [runId]: { ...current, ...patch } };
  subscribers.forEach((fn) => fn());
}

function append(runId: string, line: string) {
  const current = jobs[runId];
  if (current) set(runId, { log: [...current.log, line].slice(-MAX_LOG) });
}

export async function startStage2(
  runId: string,
  runDir: string,
  onFinished: (exit: JobExit) => void,
): Promise<void> {
  const onEvent = (e: BridgeEvent) => {
    if (e.type === "phase") set(runId, { phase: e.name, detail: e.detail, progress: null });
    else if (e.type === "progress") set(runId, { progress: { done: e.done, total: e.total } });
    else if (e.type === "solve_progress") set(runId, { solve: nextSolve(jobs[runId]?.solve ?? null, e) });
    else if (e.type === "error") {
      set(runId, { errors: [...(jobs[runId]?.errors ?? []), e.message] });
      append(runId, e.message);
    }
  };
  // Create the entry first: the bridge's first events can be delivered before startJob resolves.
  jobs = {
    ...jobs,
    [runId]: {
      jobId: -1,
      startedAt: Date.now(),
      phase: "starting",
      detail: "",
      progress: null,
      solve: null,
      log: [],
      errors: [],
      exit: null,
      cancelling: false,
    },
  };
  subscribers.forEach((fn) => fn());
  let job;
  try {
    job = await startJob(["stage2", runDir], { runDir, onEvent, onLog: (line) => append(runId, line) });
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

export async function cancelStage2(runId: string): Promise<void> {
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

export function isStage2Running(runId: string): boolean {
  const job = jobs[runId];
  return !!job && !job.exit;
}

export function useStage2Job(runId: string | null): Stage2Job | null {
  return useSyncExternalStore(
    (fn) => {
      subscribers.add(fn);
      return () => subscribers.delete(fn);
    },
    () => (runId ? (jobs[runId] ?? null) : null),
  );
}
