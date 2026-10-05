// Compare two runs' Stage 2 results: closest-pair distance over time
// overlaid, the headline numbers side by side, and the planner settings that differ.

import { useEffect, useMemo, useState } from "react";
import { readRunJson, runJob } from "../bridge/api";
import type { ConfigField, Overrides, RunRecord } from "../bridge/types";
import { duration, runCreated, runName, STATUS } from "../app/format";
import { flatten, type Value } from "../app/overrides";
import { SERIES_COLORS, SeparationCompare } from "../replay/SeparationCompare";
import { formatTime, metres } from "../replay/sampling";
import type { ReplayHeader, Separation } from "../replay/types";

interface Loaded {
  separation: Separation | null;
  floor: number | null;
  settings: Map<string, Value>; // every field's effective value
  fields: ConfigField[];
}

const hasResult = (r: RunRecord) => r.stage2.status === "succeeded" || r.stage2.status === "failed_safety";

async function load(run: RunRecord): Promise<Loaded> {
  const [header, separation, overrides, config] = await Promise.all([
    readRunJson<ReplayHeader>(run.run_id, "stage2/replay/replay.json"),
    readRunJson<Separation>(run.run_id, "stage2/replay/separation.json"),
    readRunJson<Overrides>(run.run_id, "stage2/config_overrides.json"),
    runJob(["config", run.run_dir]),
  ]);
  const event = config.events.find((e) => e.type === "config");
  const fields = event && event.type === "config" ? event.fields : [];
  const values = flatten(overrides ?? {});
  const settings = new Map(fields.map((f) => [f.path, f.path in values ? values[f.path] : f.baseline] as const));
  return { separation, floor: header?.overlays.gatekeeper_floor_m ?? null, settings, fields };
}

/** The run to offer first: the one this was copied from, a copy of this one, then the same show file. */
function suggest(run: RunRecord, candidates: RunRecord[]): RunRecord | undefined {
  return (
    candidates.find((r) => r.run_id === run.copied_from) ??
    candidates.find((r) => r.copied_from === run.run_id) ??
    candidates.find((r) => r.input.sha256 === run.input.sha256) ??
    candidates[0]
  );
}

const show = (v: Value | undefined) =>
  v === undefined ? "n/a" : typeof v === "boolean" ? (v ? "On" : "Off") : v === "disabled" ? "Off" : String(v);

export function CompareView({ run, runs }: { run: RunRecord; runs: RunRecord[] }) {
  const candidates = useMemo(() => runs.filter((r) => r.run_id !== run.run_id && hasResult(r)), [runs, run.run_id]);
  const [otherId, setOtherId] = useState<string | null>(() => suggest(run, candidates)?.run_id ?? null);
  const other = candidates.find((r) => r.run_id === otherId) ?? null;
  const [data, setData] = useState<{ a: Loaded; b: Loaded } | null>(null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (!other || !hasResult(run)) return;
    let live = true;
    setData(null);
    setError(null);
    Promise.all([load(run), load(other)])
      .then(([a, b]) => live && setData({ a, b }))
      .catch((e) => live && setError(String(e)));
    return () => {
      live = false;
    };
  }, [run, other]);

  if (!hasResult(run)) {
    return (
      <div className="page">
        <header className="page-head">
          <h1>Compare</h1>
        </header>
        <p>Run Stage 2 on this run first. Comparing needs a result from both runs.</p>
      </div>
    );
  }

  const sameShow = other && other.input.sha256 === run.input.sha256;
  const nameA = "This run";
  const nameB = other ? runName(other) : "";
  const differences =
    data && other
      ? data.a.fields.filter((f) => show(data.a.settings.get(f.path)) !== show(data.b.settings.get(f.path)))
      : [];

  return (
    <div className="page">
      <header className="page-head">
        <h1>Compare</h1>
        <p className="lede">
          How close the drones come over the whole show in this run and another one, for example the same show
          planned with different settings.
        </p>
      </header>

      {candidates.length === 0 ? (
        <p>
          No other run has a Stage 2 result yet. On the Stage 2 tab, "Try these settings in a new run" makes a copy
          of this show to plan with other settings.
        </p>
      ) : (
        <>
          <label className="field compare-pick">
            <span className="field-label">Compare with</span>
            <select value={otherId ?? ""} onChange={(e) => setOtherId(e.target.value)}>
              {candidates.map((r) => (
                <option key={r.run_id} value={r.run_id}>
                  {runName(r)}, {runCreated(r)} ({STATUS[r.stage2.status].label.toLowerCase()}
                  {r.input.sha256 === run.input.sha256 ? ", same show" : ""})
                </option>
              ))}
            </select>
          </label>
          {other && !sameShow && (
            <p className="notice notice-warn">
              These runs plan different show files, so their formations and timing differ too, not only the settings.
            </p>
          )}
          {error && <p className="notice notice-bad">{error}</p>}
          {!data && !error && <p className="status-line">Loading both runs…</p>}

          {data && other && (
            <>
              {data.a.separation && data.b.separation ? (
                <SeparationCompare
                  a={{ label: nameA, separation: data.a.separation, floor: data.a.floor }}
                  b={{ label: nameB, separation: data.b.separation, floor: data.b.floor }}
                />
              ) : (
                <p className="notice">
                  {data.a.separation ? nameB : nameA} has no replay data yet. Build it on that run's Replay tab.
                </p>
              )}

              <table className="table data compare-table">
                <thead>
                  <tr>
                    <th scope="col">
                      <span className="visually-hidden">Measure</span>
                    </th>
                    {[nameA, nameB].map((n, i) => (
                      <th key={i} scope="col">
                        <span className="series-swatch" style={{ background: SERIES_COLORS[i] }} aria-hidden="true" />
                        {n}
                      </th>
                    ))}
                  </tr>
                </thead>
                <tbody>
                  <tr>
                    <th scope="row">Stage 2</th>
                    {[run, other].map((r) => (
                      <td key={r.run_id} className={`tone-${STATUS[r.stage2.status].tone}`}>
                        {STATUS[r.stage2.status].label}
                      </td>
                    ))}
                  </tr>
                  <tr>
                    <th scope="row">Closest approach (sampled)</th>
                    {[data.a, data.b].map((d, i) => {
                      const w = d.separation?.worst;
                      const below = w && d.floor !== null && (w.distance_m ?? Infinity) < d.floor;
                      return (
                        <td key={i} className={below ? "tone-bad" : undefined}>
                          {w ? `${metres(w.distance_m, 2)} at ${formatTime(w.time_sec)}` : "n/a"}
                        </td>
                      );
                    })}
                  </tr>
                  <tr>
                    <th scope="row">Required distance</th>
                    {[data.a, data.b].map((d, i) => (
                      <td key={i}>{metres(d.floor, 2)}</td>
                    ))}
                  </tr>
                  <tr>
                    <th scope="row">Show length</th>
                    {[data.a, data.b].map((d, i) => {
                      const s = d.separation;
                      return <td key={i}>{s ? duration(s.times[s.times.length - 1] - s.times[0]) : "n/a"}</td>;
                    })}
                  </tr>
                  <tr>
                    <th scope="row">Planning time</th>
                    {[run, other].map((r) => (
                      <td key={r.run_id}>{duration(r.stage2.wall_time_sec)}</td>
                    ))}
                  </tr>
                  {(run.stage2.status === "failed_safety" || other.stage2.status === "failed_safety") && (
                    <tr>
                      <th scope="row">Rejected at</th>
                      {[run, other].map((r) => (
                        <td key={r.run_id}>
                          {r.stage2.transition
                            ? `${r.stage2.transition.from_keyframe.replace("holding_area", "holding area")} to ${r.stage2.transition.to_keyframe.replace("holding_area", "holding area")}`
                            : "n/a"}
                        </td>
                      ))}
                    </tr>
                  )}
                </tbody>
              </table>

              <h3>Planner settings that differ</h3>
              {differences.length === 0 ? (
                <p className="muted">Both runs used the same planner settings.</p>
              ) : (
                <table className="table data compare-table">
                  <thead>
                    <tr>
                      <th scope="col">Setting</th>
                      <th scope="col">{nameA}</th>
                      <th scope="col">{nameB}</th>
                    </tr>
                  </thead>
                  <tbody>
                    {differences.map((f) => (
                      <tr key={f.path}>
                        <th scope="row">
                          {f.label} <span className="setting-help">{f.group}</span>
                        </th>
                        <td>{show(data.a.settings.get(f.path))}</td>
                        <td>{show(data.b.settings.get(f.path))}</td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              )}
            </>
          )}
        </>
      )}
    </div>
  );
}
