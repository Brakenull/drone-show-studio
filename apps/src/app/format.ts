import type { RunRecord, RunStatus } from "../bridge/types";

export type Tone = "ok" | "bad" | "warn" | "idle" | "busy";

/** Each tone as antd's Tag colour and Typography.Text type. */
export const TONE_TAG: Record<Tone, string | undefined> = {
  ok: "success",
  bad: "error",
  warn: "warning",
  busy: "processing",
  idle: undefined,
};
export const TONE_TEXT = { ok: "success", bad: "danger", warn: "warning", busy: "secondary", idle: "secondary" } as const;

export const STATUS: Record<RunStatus, { label: string; tone: Tone }> = {
  not_run: { label: "Not run", tone: "idle" },
  running: { label: "Running", tone: "busy" },
  succeeded: { label: "Passed", tone: "ok" },
  failed_safety: { label: "Rejected", tone: "bad" },
  failed_input: { label: "Invalid input", tone: "warn" },
  failed_error: { label: "Error", tone: "warn" },
  cancelled: { label: "Cancelled", tone: "idle" },
};

/** "20260923-213412_intermediate_export-1" -> "intermediate_export-1" */
export function runName(run: Pick<RunRecord, "run_id">): string {
  const i = run.run_id.indexOf("_");
  return i >= 0 ? run.run_id.slice(i + 1) : run.run_id;
}

export function runCreated(run: Pick<RunRecord, "created_at">): string {
  const d = new Date(run.created_at);
  if (Number.isNaN(d.getTime())) return run.created_at;
  return d.toLocaleString(undefined, { day: "numeric", month: "short", hour: "2-digit", minute: "2-digit" });
}

export function duration(seconds: number | null | undefined): string {
  if (seconds === null || seconds === undefined || !Number.isFinite(seconds)) return "n/a";
  const s = Math.round(seconds);
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  const rest = s % 60;
  if (h) return `${h} h ${m} min`;
  if (m) return `${m} min ${rest} s`;
  return `${seconds < 10 ? seconds.toFixed(1) : rest} s`;
}

export function clock(ms: number): string {
  const s = Math.max(0, Math.floor(ms / 1000));
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  const pad = (v: number) => String(v).padStart(2, "0");
  return `${h}:${pad(m)}:${pad(s % 60)}`;
}
