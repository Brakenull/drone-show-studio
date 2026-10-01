// Loads a run's replay files and hands them to the reusable player (docs/5-studio_gui.md §6.4).

import { useEffect, useState } from "react";
import { readRunBytes, readRunJson, runJob } from "../bridge/api";
import type { RunRecord } from "../bridge/types";
import { ReplayPlayer } from "../replay/ReplayPlayer";
import type { ReplayData, ReplayFocus, ReplayHeader, Separation } from "../replay/types";

/** What the Replay tab plays: the show, or a return path (the show up to formation k, then the flight home). */
export type ReplaySource = { kind: "show" } | { kind: "return"; keyframe: number; from: string };

const folder = (source: ReplaySource) =>
  source.kind === "show" ? "stage2/replay" : `stage2/returns/replay_${source.keyframe}`;

const cache = new Map<string, { stamp: string; data: ReplayData }>();

async function loadReplay(runId: string, dir: string): Promise<ReplayData | null> {
  const header = await readRunJson<ReplayHeader>(runId, `${dir}/replay.json`);
  if (!header) return null;
  const [separation, positions, colors] = await Promise.all([
    readRunJson<Separation>(runId, `${dir}/separation.json`),
    readRunBytes(runId, `${dir}/positions.f32`),
    readRunBytes(runId, `${dir}/colors.u8`),
  ]);
  if (!separation) return null;
  const expected = header.frames * header.fleet_size * 3;
  const pos = new Float32Array(positions);
  const col = new Uint8Array(colors);
  if (pos.length !== expected || col.length !== expected) {
    throw new Error(`replay files don't match replay.json (${pos.length} values, expected ${expected}); rebuild it`);
  }
  return { header, separation, positions: pos, colors: col };
}

export function ReplayView({
  run,
  focus,
  source = { kind: "show" },
  onShowPlayback,
}: {
  run: RunRecord;
  focus: ReplayFocus | null;
  source?: ReplaySource;
  /** Back to the show itself from a return path's replay. */
  onShowPlayback?: () => void;
}) {
  const dir = folder(source);
  const key = `${run.run_id}:${dir}`;
  const stamp = `${run.stage2.status}:${run.stage2.ended_at ?? ""}:${run.stage2_returns?.ended_at ?? ""}`;
  const cached = cache.get(key);
  const [data, setData] = useState<ReplayData | null>(cached?.stamp === stamp ? cached.data : null);
  const [state, setState] = useState<"loading" | "ready" | "missing" | "error">(data ? "ready" : "loading");
  const [error, setError] = useState<string | null>(null);
  const [rebuilding, setRebuilding] = useState(false);

  useEffect(() => {
    const hit = cache.get(key);
    if (hit?.stamp === stamp) {
      setData(hit.data);
      setState("ready");
      return;
    }
    let live = true;
    setState("loading");
    loadReplay(run.run_id, dir)
      .then((d) => {
        if (!live) return;
        if (d) cache.set(key, { stamp, data: d });
        setData(d);
        setState(d ? "ready" : "missing");
      })
      .catch((e) => {
        if (!live) return;
        setError(String(e));
        setState("error");
      });
    return () => {
      live = false;
    };
  }, [run.run_id, dir, key, stamp]);

  async function rebuild() {
    setRebuilding(true);
    setError(null);
    try {
      const args = ["replay", run.run_dir, ...(source.kind === "return" ? ["--return", String(source.keyframe)] : [])];
      const { exit, events } = await runJob(args);
      if (exit.code !== 0) {
        const err = events.find((e) => e.type === "error");
        throw new Error(err && err.type === "error" ? err.message : `exit code ${exit.code}`);
      }
      cache.delete(key);
      const d = await loadReplay(run.run_id, dir);
      if (d) cache.set(key, { stamp, data: d });
      setData(d);
      setState(d ? "ready" : "missing");
    } catch (e) {
      setError(String(e));
      setState("error");
    } finally {
      setRebuilding(false);
    }
  }

  const hasOutput =
    source.kind === "return" || run.stage2.status === "succeeded" || run.stage2.status === "failed_safety";
  if (state === "ready" && data) {
    const label =
      source.kind === "return"
        ? `The show until ${source.from}, then the flight home`
        : run.stage2.status === "failed_safety"
          ? "Rejected attempt, after the transitions that passed"
          : "Planned show";
    return (
      <div className="replay-wrap">
        {source.kind === "return" && onShowPlayback && (
          <button className="link replay-back" onClick={onShowPlayback}>
            Back to the planned show
          </button>
        )}
        <ReplayPlayer key={dir} data={data} focus={focus} label={label} />
      </div>
    );
  }
  return (
    <div className="page">
      <header className="page-head">
        <h1>Replay</h1>
      </header>
      {state === "loading" && <p className="status-line">Loading the replay…</p>}
      {(state === "missing" || state === "error") && (
        <>
          {error && <p className="notice notice-bad">{error}</p>}
          {hasOutput ? (
            <>
              <p>
                {source.kind === "return"
                  ? `The return from ${source.from} has no replay files yet.`
                  : "This run has Stage 2 output but no replay files yet."}
              </p>
              <div className="actions">
                <button className="primary" onClick={rebuild} disabled={rebuilding}>
                  {rebuilding ? "Building replay…" : "Build replay"}
                </button>
              </div>
            </>
          ) : (
            <p>There's nothing to replay yet. Run Stage 2 first; the replay is built when it finishes.</p>
          )}
        </>
      )}
    </div>
  );
}
