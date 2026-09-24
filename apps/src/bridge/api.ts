// Typed wrappers around the Rust commands (src-tauri/src) and the bridge event stream.

import { invoke } from "@tauri-apps/api/core";
import { listen } from "@tauri-apps/api/event";
import type { BridgeEvent, JobExit, RunRecord, Settings } from "./types";

interface JobHandlers {
  onEvent?: (event: BridgeEvent) => void;
  onLog?: (line: string) => void;
  resolve: (exit: JobExit) => void;
}

type Pending =
  | { kind: "event"; event: BridgeEvent }
  | { kind: "log"; line: string }
  | { kind: "exit"; exit: JobExit };

const handlers = new Map<number, JobHandlers>();
// Events can arrive before start_job's promise resolves with the job id; hold them until then.
const early = new Map<number, Pending[]>();

function deliver(jobId: number, item: Pending) {
  const h = handlers.get(jobId);
  if (!h) {
    early.set(jobId, [...(early.get(jobId) ?? []), item]);
    return;
  }
  if (item.kind === "event") h.onEvent?.(item.event);
  else if (item.kind === "log") h.onLog?.(item.line);
  else {
    handlers.delete(jobId);
    h.resolve(item.exit);
  }
}

let listening: Promise<void> | null = null;
function ensureListening(): Promise<void> {
  listening ??= Promise.all([
    listen<{ job_id: number; event: BridgeEvent }>("bridge-event", (e) =>
      deliver(e.payload.job_id, { kind: "event", event: e.payload.event }),
    ),
    listen<{ job_id: number; line: string }>("bridge-log", (e) =>
      deliver(e.payload.job_id, { kind: "log", line: e.payload.line }),
    ),
    listen<{ job_id: number; code: number | null; cancelled: boolean }>("bridge-exit", (e) =>
      deliver(e.payload.job_id, {
        kind: "exit",
        exit: { code: e.payload.code, cancelled: e.payload.cancelled },
      }),
    ),
  ]).then(() => undefined);
  return listening;
}

export interface Job {
  id: number;
  done: Promise<JobExit>;
}

/** A part of run.json a job owns; it picks the log file and what a cancel marks cancelled. */
export type RunSection = "stage2" | "monte_carlo" | "pack";

/** Start `python -m tools.studio_bridge <args>`; `run` ties it to a run folder section (log + cancel state). */
export async function startJob(
  args: string[],
  opts: {
    run?: { dir: string; section: RunSection };
    onEvent?: (e: BridgeEvent) => void;
    onLog?: (line: string) => void;
  } = {},
): Promise<Job> {
  await ensureListening();
  const id = await invoke<number>("start_job", {
    args,
    runDir: opts.run?.dir ?? null,
    section: opts.run?.section ?? null,
  });
  const done = new Promise<JobExit>((resolve) => {
    handlers.set(id, { onEvent: opts.onEvent, onLog: opts.onLog, resolve });
  });
  for (const item of early.get(id) ?? []) deliver(id, item);
  early.delete(id);
  return { id, done };
}

/** Run a short command and collect its events. */
export async function runJob(args: string[]): Promise<{ events: BridgeEvent[]; exit: JobExit }> {
  const events: BridgeEvent[] = [];
  const job = await startJob(args, { onEvent: (e) => events.push(e) });
  const exit = await job.done;
  return { events, exit };
}

export const cancelJob = (jobId: number) => invoke<void>("cancel_job", { jobId });
export const getSettings = () => invoke<Settings>("get_settings");
export const saveSettings = (settings: Settings) => invoke<Settings>("save_settings", { settings });
export const listRuns = () => invoke<RunRecord[]>("list_runs");
export const openRunFolder = (runId: string, rel?: string) =>
  invoke<void>("open_run_folder", { runId, rel: rel ?? null });
export const stashImport = (name: string, text: string) => invoke<string>("stash_import", { name, text });

export async function readRunJson<T>(runId: string, rel: string): Promise<T | null> {
  const text = await invoke<string | null>("read_run_text", { runId, rel });
  return text === null ? null : (JSON.parse(text) as T);
}

export const readRunText = (runId: string, rel: string) =>
  invoke<string | null>("read_run_text", { runId, rel });

export const readRunBytes = (runId: string, rel: string) =>
  invoke<ArrayBuffer>("read_run_bytes", { runId, rel });
