// Live Stage 2 jobs keyed by run id. Lives outside React so a solve keeps streaming while the
// user looks at other runs or views.

import { useSyncExternalStore } from "react";
import { cancelJob, startJob } from "../bridge/api";
import type { BridgeEvent, JobExit } from "../bridge/types";

export interface Stage2Job {
  jobId: number;
  startedAt: number; // ms epoch
  phase: string;
  detail: string;
  progress: { done: number; total: number } | null;
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
