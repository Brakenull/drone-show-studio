// Loads a run's replay files and hands them to the reusable player (docs/5-studio_gui.md §6.4).

import { useEffect, useState } from "react";
import { readRunBytes, readRunJson, runJob } from "../bridge/api";
import type { RunRecord } from "../bridge/types";
import { ReplayPlayer } from "../replay/ReplayPlayer";
import type { ReplayData, ReplayFocus, ReplayHeader, Separation } from "../replay/types";

const cache = new Map<string, { stamp: string; data: ReplayData }>();

async function loadReplay(runId: string): Promise<ReplayData | null> {
  const header = await readRunJson<ReplayHeader>(runId, "stage2/replay/replay.json");
  if (!header) return null;
  const [separation, positions, colors] = await Promise.all([
    readRunJson<Separation>(runId, "stage2/replay/separation.json"),
    readRunBytes(runId, "stage2/replay/positions.f32"),
    readRunBytes(runId, "stage2/replay/colors.u8"),
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

export function ReplayView({ run, focus }: { run: RunRecord; focus: ReplayFocus | null }) {
  const stamp = `${run.stage2.status}:${run.stage2.ended_at ?? ""}`;
  const cached = cache.get(run.run_id);
  const [data, setData] = useState<ReplayData | null>(cached?.stamp === stamp ? cached.data : null);
  const [state, setState] = useState<"loading" | "ready" | "missing" | "error">(data ? "ready" : "loading");
  const [error, setError] = useState<string | null>(null);
  const [rebuilding, setRebuilding] = useState(false);

  useEffect(() => {
    const hit = cache.get(run.run_id);
    if (hit?.stamp === stamp) {
      setData(hit.data);
      setState("ready");
      return;
    }
    let live = true;
    setState("loading");
    loadReplay(run.run_id)
      .then((d) => {
        if (!live) return;
        if (d) cache.set(run.run_id, { stamp, data: d });
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
  }, [run.run_id, stamp]);

  async function rebuild() {
    setRebuilding(true);
    setError(null);
    try {
      const { exit, events } = await runJob(["replay", run.run_dir]);
      if (exit.code !== 0) {
        const err = events.find((e) => e.type === "error");
        throw new Error(err && err.type === "error" ? err.message : `exit code ${exit.code}`);
      }
      cache.delete(run.run_id);
      const d = await loadReplay(run.run_id);
      if (d) cache.set(run.run_id, { stamp, data: d });
      setData(d);
      setState(d ? "ready" : "missing");
    } catch (e) {
      setError(String(e));
      setState("error");
    } finally {
      setRebuilding(false);
    }
  }

  const hasOutput = run.stage2.status === "succeeded" || run.stage2.status === "failed_safety";
  if (state === "ready" && data) {
    const label =
      run.stage2.status === "failed_safety"
        ? "Rejected attempt, after the transitions that passed"
        : "Planned show";
    return <ReplayPlayer data={data} focus={focus} label={label} />;
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
              <p>This run has Stage 2 output but no replay files yet.</p>
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
