// Conditions: weather scenarios for a show, flown through the digital twin and played back
// (docs/4-condition_simulator.md §6, milestone C1). The rain rule (return to the holding area) is
// not simulated yet: rain is drawn and its levels are marked, nothing reacts to it.

import { useCallback, useEffect, useMemo, useState } from "react";
import { runJob } from "../bridge/api";
import type {
  ConditionsInfo,
  JobExit,
  RtkState,
  RunRecord,
  Scenario,
  ScenarioEntry,
  ScenarioResult,
} from "../bridge/types";
import { cancelSimulate, startSimulate, useSimulateJob, type SimulateJob } from "../app/simulateJobs";
import { isStage2Running } from "../app/stage2Jobs";
import { clock, duration as formatDuration } from "../app/format";
import { formatTime, metres } from "../replay/sampling";
import { ReplayPlayer } from "../replay/ReplayPlayer";
import type { ReplayData, ReplayFocus } from "../replay/types";
import { compass, rainLabel, RTK_LABEL } from "../replay/weather";
import { loadReplay } from "./ReplayView";
import { TimelineEditor, type Channel, type KeyRef } from "./TimelineEditor";

/** Seconds the simulation keeps flying after the show (twin_sim/scenario_runner.py TAIL_SEC). */
const TAIL_SEC = 2;

interface Props {
  run: RunRecord;
  onFinished: () => void;
}

function blankScenario(name: string, defaults: ConditionsInfo["defaults"], seed: number): Scenario {
  return {
    name,
    seed,
    wind: [{ t: 0, speed_mps: 3, from_deg: 270, turbulence: 0.15 }],
    gusts: [],
    rtk: [{ t: 0, state: "fixed" }],
    rain: [{ t: 0, mm_h: 0 }],
    rain_rule: { ...defaults.rain_rule },
  };
}

const same = (a: unknown, b: unknown) => JSON.stringify(a) === JSON.stringify(b);
const pct = (v: number) => `${Math.round(v * 100)} %`;

function errorOf(events: { type: string }[], fallback: string): { message: string; errors: string[] } {
  const e = events.find((x) => x.type === "error") as { message: string; errors?: string[] } | undefined;
  return { message: e?.message ?? fallback, errors: e?.errors ?? [] };
}

export function ConditionsView({ run, onFinished }: Props) {
  if (run.stage2.status !== "succeeded" || isStage2Running(run.run_id)) {
    return (
      <div className="page">
        <header className="page-head">
          <h1>Conditions</h1>
        </header>
        <p>
          Weather scenarios fly the planned show, so they need a run whose Stage 2 passed.{" "}
          {run.stage2.status === "not_run" ? "Run Stage 2 first." : "This one hasn't."}
        </p>
      </div>
    );
  }
  return <Conditions key={run.run_id} run={run} onFinished={onFinished} />;
}

function Conditions({ run, onFinished }: Props) {
  const [info, setInfo] = useState<ConditionsInfo | null>(null);
  const [loadError, setLoadError] = useState<string | null>(null);
  const [currentId, setCurrentId] = useState<string | null>(null);
  // Unsaved edits, tied to the scenario they belong to: switching scenarios can never carry them over.
  const [edit, setEdit] = useState<{ id: string; scenario: Scenario } | null>(null);
  const [selected, setSelected] = useState<KeyRef | null>(null);
  const [busy, setBusy] = useState(false);
  const [problem, setProblem] = useState<{ message: string; errors: string[] } | null>(null);
  const [confirmDelete, setConfirmDelete] = useState(false);
  const [mode, setMode] = useState<"timeline" | "playback">("timeline");
  const [focus, setFocus] = useState<ReplayFocus | null>(null);
  const job = useSimulateJob(run.run_id);
  const running = !!job && !job.exit;

  const reload = useCallback(
    async (select?: string | null) => {
      const { events, exit } = await runJob(["conditions", run.run_dir]);
      const ev = events.find((e) => e.type === "conditions");
      if (exit.code !== 0 || !ev || ev.type !== "conditions") {
        setLoadError(errorOf(events, `exit code ${exit.code}`).message);
        return;
      }
      const { type: _type, ...data } = ev;
      setInfo(data);
      setLoadError(null);
      setCurrentId((prev) => {
        const want = select === undefined ? prev : select;
        return data.scenarios.some((s) => s.id === want) ? want : (data.scenarios[0]?.id ?? null);
      });
    },
    [run.run_dir],
  );

  useEffect(() => {
    void reload();
  }, [reload]);

  const entry: ScenarioEntry | null = info?.scenarios.find((s) => s.id === currentId) ?? null;
  const draft: Scenario | null = entry ? (edit?.id === entry.id ? edit.scenario : entry.scenario) : null;
  const setDraft = (scenario: Scenario) => entry && setEdit({ id: entry.id, scenario });
  useEffect(() => {
    setSelected(null);
    setProblem(null);
    setConfirmDelete(false);
  }, [entry?.id]);

  const dirty = !!entry && !!draft && !same(entry.scenario, draft);
  const showDuration = info?.show.duration_sec ?? 0;
  const timelineDuration = showDuration + TAIL_SEC;

  async function save(scenario: Scenario, id: string | null): Promise<string | null> {
    setBusy(true);
    setProblem(null);
    try {
      const args = ["scenario-save", run.run_dir, "--json", JSON.stringify(scenario), ...(id ? ["--id", id] : [])];
      const { events, exit } = await runJob(args);
      const saved = events.find((e) => e.type === "scenario_saved");
      if (exit.code !== 0 || !saved || saved.type !== "scenario_saved") {
        setProblem(errorOf(events, `exit code ${exit.code}`));
        return null;
      }
      await reload(saved.id);
      setEdit(null); // after the reload, so the edited values don't flicker back to the old ones
      return saved.id;
    } catch (e) {
      setProblem({ message: String(e), errors: [] });
      return null;
    } finally {
      setBusy(false);
    }
  }

  async function remove(id: string) {
    setBusy(true);
    try {
      const { events, exit } = await runJob(["scenario-delete", run.run_dir, "--id", id]);
      if (exit.code !== 0) setProblem(errorOf(events, `exit code ${exit.code}`));
      await reload(null);
    } finally {
      setBusy(false);
      setConfirmDelete(false);
    }
  }

  function newScenario() {
    if (!info) return;
    const n = info.scenarios.length + 1;
    void save(blankScenario(`Scenario ${n}`, info.defaults, 20260924 + n), null);
  }

  function duplicate() {
    if (!draft) return;
    void save({ ...draft, name: `${draft.name} (copy)` }, null);
  }

  async function simulate() {
    if (!entry || !draft) return;
    let id: string | null = entry.id;
    if (dirty) id = await save(draft, entry.id);
    if (!id) return;
    setMode("timeline");
    const device = run.stage3?.monte_carlo?.config?.device ?? "auto";
    startSimulate(run.run_id, run.run_dir, id, device, (exit: JobExit) => {
      void reload(id).then(() => {
        // 0 = passed, 1 = a criterion failed; a killed process can also exit with 1, hence `cancelled`.
        if (!exit.cancelled && (exit.code === 0 || exit.code === 1)) {
          setFocus(null);
          setMode("playback");
        }
      });
      onFinished();
    }).catch((e) => setProblem({ message: String(e), errors: [] }));
  }

  const openAt = (time: number, drones: number[]) => {
    setFocus({ time, drones, key: Date.now() });
    setMode("playback");
  };

  if (loadError) {
    return (
      <div className="page">
        <header className="page-head">
          <h1>Conditions</h1>
        </header>
        <p className="notice notice-bad">Could not read this run's scenarios: {loadError}</p>
      </div>
    );
  }
  if (!info) {
    return (
      <div className="page">
        <p className="status-line">Loading scenarios…</p>
      </div>
    );
  }

  if (mode === "playback" && entry?.playback && !running) {
    return (
      <Playback
        run={run}
        entry={entry}
        focus={focus}
        onBack={() => setMode("timeline")}
      />
    );
  }

  return (
    <div className="page page-wide conditions">
      <header className="page-head">
        <h1>Conditions</h1>
        <p className="lede">
          Write the weather for this show (wind, gusts, RTK quality and rain, each free to change during the show),
          then fly the whole show through the digital twin and watch it.
        </p>
      </header>

      <div className="scenario-bar">
        <label className="field">
          <span className="field-label">Scenario</span>
          <select
            value={currentId ?? ""}
            disabled={!info.scenarios.length || busy || running}
            onChange={(e) => setCurrentId(e.target.value)}
          >
            {!info.scenarios.length && <option value="">No scenarios yet</option>}
            {info.scenarios.map((s) => (
              <option key={s.id} value={s.id}>
                {s.scenario.name}
              </option>
            ))}
          </select>
        </label>
        <button onClick={newScenario} disabled={busy || running} className={info.scenarios.length ? "" : "primary"}>
          New scenario
        </button>
        <button onClick={duplicate} disabled={!draft || busy || running}>
          Duplicate
        </button>
        {entry &&
          (confirmDelete ? (
            <>
              <button className="danger" onClick={() => void remove(entry.id)} disabled={busy || running}>
                Delete {entry.scenario.name} and its results
              </button>
              <button className="link" onClick={() => setConfirmDelete(false)}>
                Keep it
              </button>
            </>
          ) : (
            <button onClick={() => setConfirmDelete(true)} disabled={busy || running}>
              Delete
            </button>
          ))}
      </div>

      {problem && (
        <div className="notice notice-bad">
          <p>{problem.errors.length ? "The scenario wasn't saved:" : problem.message}</p>
          {problem.errors.length > 0 && (
            <ul>
              {problem.errors.map((e) => (
                <li key={e}>{e}</li>
              ))}
            </ul>
          )}
        </div>
      )}

      {!entry || !draft ? (
        <p className="empty-scenarios">
          No scenarios yet. A new scenario starts with a light westerly breeze, RTK fixed and no rain; change it on the
          timeline.
        </p>
      ) : (
        <>
          <div className="scenario-meta">
            <label className="field">
              <span className="field-label">Name</span>
              <input value={draft.name} disabled={running} onChange={(e) => setDraft({ ...draft, name: e.target.value })} />
            </label>
            <label className="field field-narrow">
              <span className="field-label">Random seed</span>
              <input
                type="number"
                min={0}
                value={draft.seed}
                disabled={running}
                onChange={(e) => setDraft({ ...draft, seed: Math.max(0, Math.floor(Number(e.target.value) || 0)) })}
              />
            </label>
            <label className="field field-narrow">
              <span className="field-label">Rain alert level (mm/h)</span>
              <input
                type="number"
                min={0}
                step={0.1}
                value={draft.rain_rule.alert_mm_h}
                disabled={running}
                onChange={(e) =>
                  setDraft({ ...draft, rain_rule: { ...draft.rain_rule, alert_mm_h: Math.max(0, Number(e.target.value) || 0) } })
                }
              />
            </label>
            <label className="field field-narrow">
              <span className="field-label">Rain limit level (mm/h)</span>
              <input
                type="number"
                min={0}
                step={0.1}
                value={draft.rain_rule.limit_mm_h}
                disabled={running}
                onChange={(e) =>
                  setDraft({ ...draft, rain_rule: { ...draft.rain_rule, limit_mm_h: Math.max(0, Number(e.target.value) || 0) } })
                }
              />
            </label>
          </div>

          <div className="timeline-wrap">
            <div>
              <TimelineEditor
                scenario={draft}
                show={info.show}
                duration={timelineDuration}
                selected={selected}
                onSelect={setSelected}
                disabled={running}
                onChange={(next, select) => {
                  setDraft(next);
                  if (select !== undefined) setSelected(select);
                }}
              />
              <p className="muted small timeline-help">
                Click a lane to add a key. Drag a key to move it; on Wind and Rain, drag up or down to change the value.
                With a key focused, the arrow keys nudge it (Shift for 5 s) and Delete removes it. Rain is drawn and its
                levels are marked; the return to the holding area isn't simulated yet.
              </p>
            </div>
            <Inspector
              scenario={draft}
              selected={selected}
              duration={timelineDuration}
              disabled={running}
              onChange={(next, select) => {
                setDraft(next);
                if (select !== undefined) setSelected(select);
              }}
            />
          </div>

          <div className="actions">
            {running ? null : (
              <>
                <button className="primary" onClick={() => void simulate()} disabled={busy}>
                  {dirty ? "Save and simulate" : "Simulate this scenario"}
                </button>
                <button onClick={() => void save(draft, entry.id)} disabled={!dirty || busy}>
                  Save changes
                </button>
                {dirty && (
                  <button className="link" onClick={() => setEdit(null)}>
                    Undo changes
                  </button>
                )}
              </>
            )}
          </div>

          {running && job!.scenarioId === entry.id && <Simulating job={job!} runId={run.run_id} />}
          {running && job!.scenarioId !== entry.id && (
            <p className="notice">Another scenario of this run is being simulated; wait for it to finish.</p>
          )}
          {!running && job?.exit && job.scenarioId === entry.id && (job.exit.cancelled || (job.exit.code !== 0 && job.exit.code !== 1)) && (
            <p className="notice notice-bad">
              {job.exit.cancelled ? "The simulation was cancelled." : `The simulation stopped: ${job.errors[job.errors.length - 1] ?? `exit code ${job.exit.code}`}`}
            </p>
          )}
          {!running && entry.result && (
            <Result
              run={run}
              entry={entry}
              edited={!same(entry.result.scenario, entry.scenario)}
              onOpen={openAt}
            />
          )}
        </>
      )}
    </div>
  );
}

// --------------------------------------------------------------------------------------------- //

function Inspector({
  scenario,
  selected,
  duration,
  disabled,
  onChange,
}: {
  scenario: Scenario;
  selected: KeyRef | null;
  duration: number;
  disabled: boolean;
  onChange: (next: Scenario, select?: KeyRef | null) => void;
}) {
  const key = selected ? (scenario[selected.channel][selected.index] as unknown as Record<string, number | string> | undefined) : undefined;
  if (!selected || !key) {
    return (
      <aside className="inspector">
        <h3>Key</h3>
        <p className="muted small">Pick a key on the timeline to see and set its exact values.</p>
      </aside>
    );
  }
  const channel: Channel = selected.channel;

  function set(field: string, value: number | string) {
    const keys = [...(scenario[channel] as unknown as Record<string, number | string>[])];
    keys[selected!.index] = { ...keys[selected!.index], [field]: value };
    const order = keys.map((k, i) => ({ k, i })).sort((a, b) => (a.k.t as number) - (b.k.t as number));
    const index = order.findIndex((o) => o.i === selected!.index);
    onChange({ ...scenario, [channel]: order.map((o) => o.k) }, { channel, index });
  }

  const num = (field: string, label: string, opts: { min?: number; max?: number; step?: number; unit?: string } = {}) => (
    <label className="field">
      <span className="field-label">
        {label}
        {opts.unit && <span className="muted"> ({opts.unit})</span>}
      </span>
      <input
        type="number"
        min={opts.min}
        max={opts.max}
        step={opts.step ?? 0.1}
        value={key[field] as number}
        disabled={disabled}
        onChange={(e) => {
          const v = Number(e.target.value);
          if (Number.isFinite(v)) set(field, v);
        }}
      />
    </label>
  );

  const title = { wind: "Wind", gusts: "Gust", rtk: "RTK quality", rain: "Rain" }[channel];
  return (
    <aside className="inspector">
      <h3>
        {title} at {formatTime(key.t as number)}
      </h3>
      <div className="inspector-fields">
        {num("t", "Show time", { min: 0, max: duration, step: 0.5, unit: "s" })}
        {channel === "wind" && (
          <>
            {num("speed_mps", "Mean speed", { min: 0, max: 40, unit: "m/s" })}
            {num("from_deg", "Blowing from", { min: 0, max: 359, step: 5, unit: `° ${compass(key.from_deg as number)}` })}
            {num("turbulence", "Turbulence", { min: 0, max: 1, step: 0.05, unit: "gust spread / mean" })}
          </>
        )}
        {channel === "gusts" && (
          <>
            {num("peak_mps", "Peak on top of the wind", { min: 0, max: 30, unit: "m/s" })}
            {num("duration_s", "Lasts", { min: 0.1, max: 60, step: 0.5, unit: "s" })}
            {num("from_deg", "Coming from", { min: 0, max: 359, step: 5, unit: `° ${compass(key.from_deg as number)}` })}
          </>
        )}
        {channel === "rtk" && (
          <label className="field">
            <span className="field-label">From here on</span>
            <select value={key.state as string} disabled={disabled} onChange={(e) => set("state", e.target.value as RtkState)}>
              <option value="fixed">{RTK_LABEL.fixed} (centimetre accuracy)</option>
              <option value="float">{RTK_LABEL.float} (about 0.3 m)</option>
              <option value="gps">{RTK_LABEL.gps} (about 1.5 m)</option>
            </select>
          </label>
        )}
        {channel === "rain" && (
          <>
            {num("mm_h", "Intensity", { min: 0, max: 200, unit: "mm/h" })}
            <p className="muted small">{rainLabel(key.mm_h as number)}</p>
          </>
        )}
      </div>
      <button
        className="link"
        disabled={disabled}
        onClick={() =>
          onChange({ ...scenario, [channel]: (scenario[channel] as unknown[]).filter((_, i) => i !== selected.index) }, null)
        }
      >
        Delete this key
      </button>
    </aside>
  );
}

function Simulating({ job, runId }: { job: SimulateJob; runId: string }) {
  const [now, setNow] = useState(Date.now());
  const [cancelError, setCancelError] = useState<string | null>(null);
  useEffect(() => {
    const id = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(id);
  }, []);
  // The first progress event (0 s) comes before the twin is built; until it moves, the twin is starting.
  const p = job.progress && job.progress.done > 0 ? job.progress : null;
  const fraction = p && p.total > 0 ? Math.min(1, p.done / p.total) : 0;
  const title =
    job.phase === "recording"
      ? "Writing the playback"
      : p
        ? `Flown ${formatTime(p.done)} of ${formatTime(p.total)}`
        : "Starting the digital twin";
  return (
    <div className="running sim-running">
      <div className="running-head">
        <span className="light light-busy" aria-hidden="true" />
        <div>
          <p className="running-phase">{title}</p>
          <p className="muted">
            Running for <span className="num">{clock(now - job.startedAt)}</span>
          </p>
        </div>
        <button
          className="danger"
          disabled={job.cancelling || job.jobId < 0}
          onClick={() => cancelSimulate(runId).catch((e) => setCancelError(String(e)))}
        >
          {job.cancelling ? "Cancelling…" : "Cancel simulation"}
        </button>
      </div>
      <div className={`bar ${p ? "" : "bar-indeterminate"}`} role="progressbar" aria-valuemin={0} aria-valuemax={100} aria-valuenow={Math.round(fraction * 100)}>
        <span style={p ? { width: `${fraction * 100}%` } : undefined} />
      </div>
      {cancelError && <p className="notice notice-bad">{cancelError}</p>}
    </div>
  );
}

function Result({
  run,
  entry,
  edited,
  onOpen,
}: {
  run: RunRecord;
  entry: ScenarioEntry;
  edited: boolean;
  onOpen: (time: number, drones: number[]) => void;
}) {
  const r = entry.result as ScenarioResult;
  const stale = !!r.stage2_ended_at && r.stage2_ended_at !== run.stage2.ended_at;
  const crashes = r.crash_pairs.length;
  const low = r.low_soc_drones.length;
  const headline = r.passed
    ? "The show held up in this weather"
    : crashes
      ? `${crashes} ${crashes === 1 ? "pair" : "pairs"} of drones came within ${metres(r.criteria.d_crash_m, 1)}`
      : `${low} ${low === 1 ? "drone landed" : "drones landed"} with less than ${pct(r.criteria.min_landing_soc)} battery`;
  return (
    <section className="sim-result" aria-labelledby="sim-result-title">
      {stale && (
        <p className="notice notice-warn">
          This result is from an earlier Stage 2 result. Simulate again to fly the current paths.
        </p>
      )}
      {edited && !stale && (
        <p className="notice notice-warn">The scenario changed after this simulation. Simulate again to see the effect.</p>
      )}
      <div className={`verdict ${r.passed ? "verdict-ok" : "verdict-bad"}`}>
        <span className={`light light-${r.passed ? "ok" : "bad"}`} aria-hidden="true" />
        <h2 id="sim-result-title">{headline}</h2>
      </div>
      <dl className="facts facts-wide">
        <div>
          <dt>Closest approach</dt>
          <dd className={(r.closest?.distance_m ?? Infinity) < r.criteria.d_crash_m ? "tone-bad" : (r.closest?.distance_m ?? Infinity) < r.criteria.d_warning_m ? "tone-warn" : undefined}>
            {r.closest ? (
              <button className="fact-link" onClick={() => onOpen(r.closest!.time_sec, [r.closest!.a, r.closest!.b])}>
                {metres(r.closest.distance_m, 2)}
              </button>
            ) : (
              "over 3 m"
            )}
          </dd>
        </div>
        <div>
          <dt>Largest deviation from plan</dt>
          <dd className={r.largest_deviation.distance_m > 0.5 ? "tone-warn" : undefined}>
            <button className="fact-link" onClick={() => onOpen(r.largest_deviation.time_sec, [r.largest_deviation.drone])}>
              {metres(r.largest_deviation.distance_m, 2)}
            </button>
          </dd>
        </div>
        <div>
          <dt>Lowest battery at landing</dt>
          <dd className={r.lowest_battery.soc < r.criteria.min_landing_soc ? "tone-bad" : undefined}>
            {pct(r.lowest_battery.soc)}
          </dd>
        </div>
        <div>
          <dt>Rain</dt>
          <dd>
            {r.rain.alert_time_sec === null ? (
              rainLabel(r.rain.peak_mm_h)
            ) : (
              <button className="fact-link" onClick={() => onOpen(r.rain.alert_time_sec!, [])}>
                alert at {formatTime(r.rain.alert_time_sec)}
              </button>
            )}
          </dd>
        </div>
      </dl>
      <p className="muted small">
        {r.closest && <>Closest pair: drones {r.closest.a} and {r.closest.b} at {formatTime(r.closest.time_sec)}. </>}
        Furthest from plan: drone {r.largest_deviation.drone} at {formatTime(r.largest_deviation.time_sec)}.{" "}
        {r.rain.alert_time_sec !== null && (
          <>
            Rain reaches the alert level at {formatTime(r.rain.alert_time_sec)}
            {r.rain.limit_time_sec !== null && <> and the limit at {formatTime(r.rain.limit_time_sec)}</>}; the return to
            the holding area isn't simulated yet.{" "}
          </>
        )}
        Flown on {r.device.replace(/\s*\(opencl:\d+:\d+\)$/, "").replace(/\((R|TM)\)/g, "")} in{" "}
        {formatDuration(r.wall_time_sec)} ({r.realtime_factor.toFixed(1)}× real time).
      </p>
      {r.crash_pairs.length > 0 && (
        <div className="table-scroll">
          <table className="table">
            <thead>
              <tr>
                <th scope="col">Drones</th>
                <th scope="col" className="num">Closest</th>
                <th scope="col" className="num">At</th>
                <th scope="col">
                  <span className="visually-hidden">Actions</span>
                </th>
              </tr>
            </thead>
            <tbody>
              {r.crash_pairs.slice(0, 20).map((p) => (
                <tr key={`${p.drone_a}-${p.drone_b}`}>
                  <td>
                    {p.drone_a} and {p.drone_b}
                  </td>
                  <td className="num tone-bad">{metres(p.min_distance_m, 2)}</td>
                  <td className="num">{formatTime(p.time_sec)}</td>
                  <td className="row-action">
                    <button className="link" onClick={() => onOpen(p.time_sec, [p.drone_a, p.drone_b])}>
                      Show
                    </button>
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      )}
      {entry.playback && (
        <div className="actions">
          <button onClick={() => onOpen(0, [])}>Watch the playback</button>
        </div>
      )}
    </section>
  );
}

// --------------------------------------------------------------------------------------------- //

const playbackCache = new Map<string, { stamp: string; data: ReplayData }>();

function Playback({
  run,
  entry,
  focus,
  onBack,
}: {
  run: RunRecord;
  entry: ScenarioEntry;
  focus: ReplayFocus | null;
  onBack: () => void;
}) {
  const dir = `stage3/scenarios/${entry.id}`;
  const key = `${run.run_id}:${dir}`;
  const stamp = entry.result?.simulated_at ?? "";
  const [data, setData] = useState<ReplayData | null>(() => {
    const hit = playbackCache.get(key);
    return hit?.stamp === stamp ? hit.data : null;
  });
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    const hit = playbackCache.get(key);
    if (hit?.stamp === stamp) {
      setData(hit.data);
      return;
    }
    let live = true;
    setData(null);
    loadReplay(run.run_id, dir)
      .then((d) => {
        if (!live) return;
        if (!d) setError("The playback files are missing; simulate the scenario again.");
        else {
          playbackCache.set(key, { stamp, data: d });
          setData(d);
        }
      })
      .catch((e) => live && setError(String(e)));
    return () => {
      live = false;
    };
  }, [run.run_id, dir, key, stamp]);

  const label = useMemo(() => `${entry.scenario.name}, flown through the digital twin`, [entry.scenario.name]);
  if (error || !data) {
    return (
      <div className="page">
        <button className="link" onClick={onBack}>
          Back to the timeline
        </button>
        {error ? <p className="notice notice-bad">{error}</p> : <p className="status-line">Loading the playback…</p>}
      </div>
    );
  }
  return (
    <div className="replay-wrap">
      <button className="link replay-back" onClick={onBack}>
        Back to the timeline
      </button>
      <ReplayPlayer key={dir + stamp} data={data} focus={focus} label={label} />
    </div>
  );
}
