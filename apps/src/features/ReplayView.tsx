// Loads a run's replay files and hands them to the reusable player (docs/5-studio_gui.md §6.4). The
// Replay tab plays the planned show, a return path, or a weather scenario flown through the digital twin.

import { useEffect, useState } from "react";
import { readRunBytes, readRunJson, runJob } from "../bridge/api";
import type { RunRecord } from "../bridge/types";
import { ReplayPlayer } from "../replay/ReplayPlayer";
import type { ReplayData, ReplayFocus, ReplayHeader, Separation } from "../replay/types";

/** What the Replay tab plays: the show, a return path (the show up to formation k, then the flight home),
 *  or a weather scenario's simulated flight (docs/4-condition_simulator.md §6). */
export type ReplaySource =
  | { kind: "show" }
  | { kind: "return"; keyframe: number; from: string }
  | { kind: "scenario"; id: string; name: string };

const folder = (source: ReplaySource) =>
  source.kind === "show"
    ? "stage2/replay"
    : source.kind === "return"
      ? `stage2/returns/replay_${source.keyframe}`
      : `stage3/scenarios/${source.id}`;

/** Simulated scenarios of a run (they have a playback), by id, from run.json. */
const simulatedIds = (run: RunRecord) =>
  Object.entries(run.conditions?.scenarios ?? {})
    .filter(([, s]) => s.status === "succeeded" || s.status === "failed_safety")
    .map(([id]) => id);

/** Scenario names, read from each scenario.json (run.json keeps only their outcomes). */
function useScenarioNames(run: RunRecord): Map<string, string> {
  const ids = simulatedIds(run).join(",");
  const [names, setNames] = useState<Map<string, string>>(new Map());
  useEffect(() => {
    let live = true;
    const list = ids ? ids.split(",") : [];
    Promise.all(
      list.map((id) =>
        readRunJson<{ name: string }>(run.run_id, `stage3/scenarios/${id}/scenario.json`)
          .then((s) => [id, s?.name ?? id] as const)
          .catch(() => [id, id] as const),
      ),
    ).then((pairs) => live && setNames(new Map(pairs)));
    return () => {
      live = false;
    };
  }, [run.run_id, ids]);
  return names;
}

const cache = new Map<string, { stamp: string; data: ReplayData }>();

/** A replay folder: the Stage 2 replay format, plus `reference.f32` for a simulated flight. */
export async function loadReplay(runId: string, dir: string): Promise<ReplayData | null> {
  const header = await readRunJson<ReplayHeader>(runId, `${dir}/replay.json`);
  if (!header) return null;
  const refFile = header.files?.reference;
  const [separation, positions, colors, reference] = await Promise.all([
    readRunJson<Separation>(runId, `${dir}/separation.json`),
    readRunBytes(runId, `${dir}/positions.f32`),
    readRunBytes(runId, `${dir}/colors.u8`),
    refFile ? readRunBytes(runId, `${dir}/${refFile}`) : Promise.resolve(null),
  ]);
  if (!separation) return null;
  const expected = header.frames * header.fleet_size * 3;
  const pos = new Float32Array(positions);
  const col = new Uint8Array(colors);
  const ref = reference ? new Float32Array(reference) : undefined;
  if (pos.length !== expected || col.length !== expected || (ref && ref.length !== expected)) {
    throw new Error(`replay files don't match replay.json (${pos.length} values, expected ${expected}); rebuild it`);
  }
  return { header, separation, positions: pos, colors: col, reference: ref };
}

export function ReplayView({
  run,
  focus,
  source = { kind: "show" },
  onSource,
  onEditScenario,
}: {
  run: RunRecord;
  focus: ReplayFocus | null;
  source?: ReplaySource;
  /** Play something else: the picker at the top. */
  onSource: (source: ReplaySource) => void;
  /** Open a scenario's weather in Stage 3 › Weather scenarios. */
  onEditScenario: (id: string) => void;
}) {
  const dir = folder(source);
  const key = `${run.run_id}:${dir}`;
  const stamp =
    source.kind === "scenario"
      ? `scenario:${run.conditions?.scenarios?.[source.id]?.ended_at ?? ""}`
      : `${run.stage2.status}:${run.stage2.ended_at ?? ""}:${run.stage2_returns?.ended_at ?? ""}`;
  const names = useScenarioNames(run);
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
    if (source.kind === "scenario") return; // a playback is written by `simulate`, not rebuilt
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
    source.kind !== "show" || run.stage2.status === "succeeded" || run.stage2.status === "failed_safety";

  // The picker: the show, the return path being viewed (opened from Stage 2), and every simulated scenario.
  const ids = simulatedIds(run);
  const value = source.kind === "show" ? "show" : source.kind === "return" ? `return:${source.keyframe}` : `scenario:${source.id}`;
  const toolbar = (
    <div className="replay-toolbar">
      <label className="replay-pick">
        <span className="field-label">Playing</span>
        <select
          value={value}
          onChange={(e) => {
            const v = e.target.value;
            if (v === "show") onSource({ kind: "show" });
            else if (v.startsWith("scenario:")) {
              const id = v.slice("scenario:".length);
              onSource({ kind: "scenario", id, name: names.get(id) ?? id });
            }
          }}
        >
          <option value="show">{run.stage2.status === "failed_safety" ? "Rejected show (Stage 2)" : "Planned show (Stage 2)"}</option>
          {source.kind === "return" && <option value={value}>Return path: flight home from {source.from}</option>}
          {ids.length > 0 && (
            <optgroup label="Weather scenarios, simulated">
              {ids.map((id) => (
                <option key={id} value={`scenario:${id}`}>
                  {names.get(id) ?? (source.kind === "scenario" && source.id === id ? source.name : id)}
                </option>
              ))}
            </optgroup>
          )}
        </select>
      </label>
      <p className="muted small replay-about">
        {source.kind === "scenario"
          ? "The simulated flight in this scenario's weather. Grey dots are the planned positions."
          : source.kind === "return"
            ? "The show until the formation, then the planned flight home."
            : ids.length
              ? "The paths Stage 2 planned. Simulated weather scenarios are in the list."
              : "The paths Stage 2 planned."}
      </p>
      {source.kind === "scenario" && (
        <button className="link" onClick={() => onEditScenario(source.id)}>
          Edit this weather
        </button>
      )}
    </div>
  );

  if (state === "ready" && data) {
    const label =
      source.kind === "scenario"
        ? `${source.name}, flown through the digital twin`
        : source.kind === "return"
          ? `The show until ${source.from}, then the flight home`
          : run.stage2.status === "failed_safety"
            ? "Rejected attempt, after the transitions that passed"
            : "Planned show";
    return (
      <div className="replay-wrap">
        {toolbar}
        <ReplayPlayer
          key={dir + stamp}
          data={data}
          focus={focus}
          label={label}
          onRebuild={source.kind === "scenario" ? undefined : rebuild}
          rebuilding={rebuilding}
        />
      </div>
    );
  }
  return (
    <div className="page">
      {hasOutput && toolbar}
      <header className="page-head">
        <h1>Replay</h1>
      </header>
      {state === "loading" && <p className="status-line">Loading the replay…</p>}
      {(state === "missing" || state === "error") && (
        <>
          {error && <p className="notice notice-bad">{error}</p>}
          {source.kind === "scenario" ? (
            <p>The playback of {source.name} is missing. Simulate the scenario again in Stage 3 › Weather scenarios.</p>
          ) : hasOutput ? (
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
