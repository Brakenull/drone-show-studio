// Live "Simulate this scenario" jobs (bridge `simulate`, docs/4-condition_simulator.md §6) keyed by run id.
// Lives outside React, like stage3Jobs, so a simulation keeps streaming while the user looks elsewhere.

import { useSyncExternalStore } from "react";
import { cancelJob, startJob } from "../bridge/api";
import type { BridgeEvent, JobExit, ScenarioResult } from "../bridge/types";

export interface SimulateJob {
  jobId: number;
  scenarioId: string;
  startedAt: number; // ms epoch
  phase: string;
  /** Simulated seconds so far and in total (the show plus its settling tail). */
  progress: { done: number; total: number } | null;
  result: ScenarioResult | null;
  errors: string[];
  exit: JobExit | null;
  cancelling: boolean;
}

let jobs: Record<string, SimulateJob> = {};
const subscribers = new Set<() => void>();

function set(runId: string, patch: Partial<SimulateJob>) {
  const current = jobs[runId];
  if (!current) return;
  jobs = { ...jobs, [runId]: { ...current, ...patch } };
  subscribers.forEach((fn) => fn());
}

export async function startSimulate(
  runId: string,
  runDir: string,
  scenarioId: string,
  device: string,
  onFinished: (exit: JobExit) => void,
): Promise<void> {
  const onEvent = (e: BridgeEvent) => {
    const job = jobs[runId];
    if (!job) return;
    if (e.type === "phase") set(runId, { phase: e.name });
    else if (e.type === "progress" && e.stage === "simulate") set(runId, { progress: { done: e.done, total: e.total } });
    else if (e.type === "sim_result") set(runId, { result: e.result });
    else if (e.type === "error") set(runId, { errors: [...job.errors, e.message] });
  };
  jobs = {
    ...jobs,
    [runId]: {
      jobId: -1,
      scenarioId,
      startedAt: Date.now(),
      phase: "starting",
      progress: null,
      result: null,
      errors: [],
      exit: null,
      cancelling: false,
    },
  };
  subscribers.forEach((fn) => fn());
  let job;
  try {
    job = await startJob(["simulate", runDir, "--scenario", scenarioId, "--device", device], {
      run: { dir: runDir, section: "simulate" },
      onEvent,
    });
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

export async function cancelSimulate(runId: string): Promise<void> {
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

export function isSimulateRunning(runId: string): boolean {
  const job = jobs[runId];
  return !!job && !job.exit;
}

export function useSimulateJob(runId: string | null): SimulateJob | null {
  return useSyncExternalStore(
    (fn) => {
      subscribers.add(fn);
      return () => subscribers.delete(fn);
    },
    () => (runId ? (jobs[runId] ?? null) : null),
  );
}
