// Conditions (Stage 3 › Weather scenarios): weather scenarios for a show, flown through the digital twin,
// and the rain return readiness of the show.
// When a scenario's rain reaches the alert level, the simulation flies the fleet home on its return
// paths. This page holds the inputs and the results; the playback is in the Replay tab.

import { useCallback, useEffect, useState } from "react";
import { Alert, Button, Input, InputNumber, Popconfirm, Select, Table, Tag, Tooltip } from "antd";
import { runJob } from "../bridge/api";
import type {
  ConditionsInfo,
  JobExit,
  RainReturn,
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
import { compass, rainLabel, RTK_LABEL } from "../replay/weather";
import { ReadinessPanel } from "./ReadinessPanel";
import { VerdictCard } from "./VerdictCard";
import { TimelineEditor, type Channel, type KeyRef } from "./TimelineEditor";
import { JobProgress } from "./JobOutput";

/** Seconds the simulation keeps flying after the show (twin_sim/scenario_runner.py TAIL_SEC). */
const TAIL_SEC = 2;

interface Props {
  run: RunRecord;
  onFinished: () => void;
  /** Play a simulated scenario in the Replay tab, at `time` with `drones` highlighted. */
  onPlay: (id: string, name: string, time: number, drones: number[]) => void;
  /** The scenario to show first (e.g. "Edit this weather" from the Replay tab). */
  scenarioId?: string | null;
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

/** First show time the rain reaches `level` (twin_sim/scenario_runner.py `rain_crossing`). */
function rainCrossing(keys: Scenario["rain"], level: number): number | null {
  const k = [...keys].sort((a, b) => a.t - b.t);
  if (!k.length) return level <= 0 ? 0 : null;
  if (k[0].mm_h >= level) return 0;
  for (let i = 1; i < k.length; i++) {
    if (k[i].mm_h >= level) {
      const a = k[i - 1];
      const b = k[i];
      return b.t === a.t ? b.t : a.t + ((level - a.mm_h) / (b.mm_h - a.mm_h)) * (b.t - a.t);
    }
  }
  return null;
}
const FULL = { width: "100%" };
const pct = (v: number) => `${Math.round(v * 100)} %`;

function errorOf(events: { type: string }[], fallback: string): { message: string; errors: string[] } {
  const e = events.find((x) => x.type === "error") as { message: string; errors?: string[] } | undefined;
  return { message: e?.message ?? fallback, errors: e?.errors ?? [] };
}

export function ConditionsView({ run, onFinished, onPlay, scenarioId }: Props) {
  if (run.stage2.status !== "succeeded" || isStage2Running(run.run_id)) {
    return (
      <div className="page">
        <header className="page-head">
          <h1>Weather scenarios</h1>
        </header>
        <p>
          Weather scenarios fly the planned show, so they need a run whose Stage 2 passed.{" "}
          {run.stage2.status === "not_run" ? "Run Stage 2 first." : "This one hasn't."}
        </p>
      </div>
    );
  }
  return <Conditions key={run.run_id} run={run} onFinished={onFinished} onPlay={onPlay} scenarioId={scenarioId} />;
}

function Conditions({ run, onFinished, onPlay, scenarioId }: Props) {
  const [info, setInfo] = useState<ConditionsInfo | null>(null);
  const [loadError, setLoadError] = useState<string | null>(null);
  const [currentId, setCurrentId] = useState<string | null>(null);
  // Unsaved edits, tied to the scenario they belong to: switching scenarios can never carry them over.
  const [edit, setEdit] = useState<{ id: string; scenario: Scenario } | null>(null);
  const [selected, setSelected] = useState<KeyRef | null>(null);
  const [busy, setBusy] = useState(false);
  const [problem, setProblem] = useState<{ message: string; errors: string[] } | null>(null);
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
    void reload(scenarioId ?? undefined);
  }, [reload, scenarioId]);

  const entry: ScenarioEntry | null = info?.scenarios.find((s) => s.id === currentId) ?? null;
  const draft: Scenario | null = entry ? (edit?.id === entry.id ? edit.scenario : entry.scenario) : null;
  const setDraft = (scenario: Scenario) => entry && setEdit({ id: entry.id, scenario });
  useEffect(() => {
    setSelected(null);
    setProblem(null);
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
    runSimulation(id);
  }

  /** Fly scenario `id`; its result shows here when it lands, and "Watch the playback" opens the Replay tab. */
  function runSimulation(id: string) {
    const device = run.stage3?.monte_carlo?.config?.device ?? "auto";
    startSimulate(run.run_id, run.run_dir, id, device, (_exit: JobExit) => {
      void reload(id);
      onFinished();
    }).catch((e) => setProblem({ message: String(e), errors: [] }));
  }

  /** "Fly this": a copy of the scenario whose rain reaches the alert level at `alert` and the limit
   *  level `window` seconds later, simulated straight away. */
  async function flyThis(alert: number, window: number) {
    if (!draft) return;
    const rule = draft.rain_rule;
    const t = Math.round(alert * 10) / 10;
    const w = Math.round(window * 10) / 10;
    const rain = [
      ...(t > 0 ? [{ t: 0, mm_h: 0 }] : []),
      { t, mm_h: rule.alert_mm_h },
      { t: t + w, mm_h: rule.limit_mm_h },
      { t: t + w + 30, mm_h: Math.max(rule.limit_mm_h * 1.5, rule.limit_mm_h + 1) },
    ];
    const id = await save({ ...draft, name: `${draft.name}, rain at ${formatTime(t)} (${w} s to the limit)`, rain }, null);
    if (!id) return;
    runSimulation(id);
  }

  const openAt = (time: number, drones: number[]) => entry && onPlay(entry.id, entry.scenario.name, time, drones);

  if (loadError) {
    return (
      <div className="page">
        <header className="page-head">
          <h1>Weather scenarios</h1>
        </header>
        <Alert type="error" showIcon title={`Could not read this run's scenarios: ${loadError}`} />
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

  return (
    <div className="page page-wide conditions">
      <header className="page-head">
        <h1>Weather scenarios</h1>
        <p className="lede">
          Write the weather for this show (wind, gusts, RTK quality and rain, each free to change during the show),
          then fly the whole show through the digital twin. Watch the flight in the Replay tab.
        </p>
      </header>

      <section className="scenario-toolbar" aria-label="Scenarios">
        <div className="scenario-list" aria-label="Scenario">
          {info.scenarios.length === 0 && <span className="muted">No scenarios yet</span>}
          {info.scenarios.map((s) => {
            return (
              <Tooltip
                key={s.id}
                title={`${s.scenario.name}: ${!s.result ? "not flown yet" : s.result.passed ? "held up" : "failed"}`}
              >
                <Tag.CheckableTag
                  checked={s.id === currentId}
                  disabled={busy || running}
                  // A click on the current scenario keeps it selected.
                  onChange={() => setCurrentId(s.id)}
                  style={{ maxWidth: "20rem", marginInlineEnd: 0, padding: "4px 12px" }}
                >
                  {s.scenario.name}
                </Tag.CheckableTag>
              </Tooltip>
            );
          })}
        </div>
        <div className="scenario-ops">
          <Button
            size="large"
            type={info.scenarios.length ? "default" : "primary"}
            onClick={newScenario}
            disabled={busy || running}
          >
            New scenario
          </Button>
          <Button size="large" onClick={duplicate} disabled={!draft || busy || running}>
            Duplicate
          </Button>
          {entry && (
            <Popconfirm
              title={`Delete "${entry.scenario.name}"?`}
              description="Its results and playback go too."
              okText="Delete it and its results"
              okButtonProps={{ danger: true }}
              cancelText="Keep it"
              onConfirm={() => remove(entry.id)}
            >
              <Button size="large" disabled={busy || running}>
                Delete
              </Button>
            </Popconfirm>
          )}
        </div>
      </section>

      {problem && (
        <Alert
          type="error"
          showIcon
          title={problem.errors.length ? "The scenario wasn't saved:" : problem.message}
          description={
            problem.errors.length > 0 && (
              <ul>
                {problem.errors.map((e) => (
                  <li key={e}>{e}</li>
                ))}
              </ul>
            )
          }
        />
      )}

      {(!entry || !draft) && (
        <p className="empty-scenarios">
          No scenarios yet. A new scenario starts with a light westerly breeze, RTK fixed and no rain; change it on the
          timeline.
        </p>
      )}

      {entry && draft && (
        <>
          <section className="card" aria-labelledby="weather-title">
            <h2 id="weather-title">Scenario and weather</h2>
            <div className="scenario-meta">
              <fieldset className="meta-group">
                <legend>Scenario</legend>
                <MetaField
                  label="Name"
                  help="How this scenario is listed here and in the Replay tab."
                  text
                  value={draft.name}
                  fallback={entry.scenario.name}
                  placeholder="Name this scenario"
                  disabled={running}
                  onChange={(v) => setDraft({ ...draft, name: String(v) })}
                />
                <MetaField
                  label="Random seed"
                  help="Fixes the turbulence and GPS noise drawn for this flight, so the scenario flies the same way every time. Change it to try another draw of the same weather. Empty keeps the saved seed."
                  value={draft.seed}
                  fallback={entry.scenario.seed}
                  integer
                  disabled={running}
                  onChange={(v) => setDraft({ ...draft, seed: Number(v) })}
                />
              </fieldset>
              <fieldset className="meta-group meta-group-rain">
                <legend>
                  When it rains <span className="legend-note">empty fields use the drone profile's values</span>
                </legend>
                {(
                  [
                    ["alert_mm_h", "Alert level", "mm/h", 0.1, "Rain intensity at which the fleet is called home. The return command follows after the reaction time."],
                    ["limit_mm_h", "Limit level", "mm/h", 0.1, "Rain the drones must not fly in. Every drone has to be home before the rain reaches it."],
                    ["reaction_s", "Reaction time", "s", 0.5, "From the rain alert until the return command reaches the drones: noticing the rain, deciding, sending the command."],
                    ["margin_s", "Margin", "s", 1, "Spare time the flight home must leave before the rain reaches the limit level."],
                  ] as const
                ).map(([key, label, unit, step, help]) => (
                  <MetaField
                    key={key}
                    label={label}
                    unit={unit}
                    help={help}
                    value={draft.rain_rule[key]}
                    fallback={info.defaults.rain_rule[key]}
                    placeholder={String(info.defaults.rain_rule[key])}
                    step={step}
                    disabled={running}
                    onChange={(v) => setDraft({ ...draft, rain_rule: { ...draft.rain_rule, [key]: Number(v) } })}
                  />
                ))}
              </fieldset>
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
                  With a key focused, the arrow keys nudge it (Shift for 5 s) and Delete removes it. When the rain reaches
                  the alert level, the simulated fleet is called home after the reaction time.
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
          </section>

          <section className="card">
            {info.readiness.error ? (
              <Alert type="warning" showIcon title={`Rain return readiness isn't available: ${info.readiness.error}`} />
            ) : (
              <ReadinessPanel
                run={run}
                readiness={info.readiness}
                scenario={draft}
                duration={timelineDuration}
                scenarioWindow={(() => {
                  const a = rainCrossing(draft.rain, draft.rain_rule.alert_mm_h);
                  const l = rainCrossing(draft.rain, draft.rain_rule.limit_mm_h);
                  return a !== null && l !== null ? l - a : null;
                })()}
                disabled={running || busy}
                onPlanned={() => void reload()}
                onFlyThis={(t, w) => void flyThis(t, w)}
                onSetAlert={(level) => setDraft({ ...draft, rain_rule: { ...draft.rain_rule, alert_mm_h: level } })}
              />
            )}
          </section>

          <section className="card fly-card" aria-labelledby="fly-title">
            <h2 id="fly-title">Fly this scenario</h2>
            <p className="muted stage3-about">
              The whole show through the digital twin in this weather. It passes when no two drones come within 0.5 m
              and every drone is home with at least 15 % battery.
            </p>
            <div className="actions">
              {running ? null : (
                <>
                  <Button type="primary" size="large" onClick={() => void simulate()} disabled={busy}>
                    {dirty ? "Save and simulate" : "Simulate this scenario"}
                  </Button>
                  <Button size="large" onClick={() => void save(draft, entry.id)} disabled={!dirty || busy}>
                    Save changes
                  </Button>
                  {dirty && (
                    <Button type="link" onClick={() => setEdit(null)}>
                      Undo changes
                    </Button>
                  )}
                </>
              )}
            </div>

            {running && job!.scenarioId === entry.id && <Simulating job={job!} runId={run.run_id} />}
            {running && job!.scenarioId !== entry.id && (
              <Alert type="info" showIcon title="Another scenario of this run is being simulated; wait for it to finish." />
            )}
            {!running && job?.exit && job.scenarioId === entry.id && (job.exit.cancelled || (job.exit.code !== 0 && job.exit.code !== 1)) && (
              <Alert
                type="error"
                showIcon
                title={job.exit.cancelled ? "The simulation was cancelled." : `The simulation stopped: ${job.errors[job.errors.length - 1] ?? `exit code ${job.exit.code}`}`}
              />
            )}
          </section>
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

/** A scenario input that shows its fallback as a placeholder: empty while the value equals the fallback
 *  (a default from the drone profile, or the saved value), and emptying it goes back to the fallback. */
function MetaField({
  label,
  unit,
  help,
  value,
  fallback,
  placeholder,
  text,
  integer,
  step,
  disabled,
  onChange,
}: {
  label: string;
  unit?: string;
  help: string;
  value: number | string;
  fallback: number | string;
  placeholder?: string;
  text?: boolean;
  integer?: boolean;
  step?: number;
  disabled: boolean;
  onChange: (v: number | string) => void;
}) {
  // While focused, the field keeps exactly what was typed ("0." on the way to "0.5").
  const [local, setLocal] = useState<string | null>(null);
  const shown = local ?? (value === fallback && !text ? "" : String(value));
  const type = (raw: string) => {
    setLocal(raw);
    const t = raw.trim();
    if (t === "") return onChange(fallback);
    if (text) return onChange(raw);
    const n = Number(t);
    if (Number.isFinite(n) && n >= 0) onChange(integer ? Math.floor(n) : n);
  };
  const common = {
    value: shown,
    placeholder: placeholder ?? String(fallback),
    disabled,
    onFocus: () => setLocal(shown),
    onBlur: () => setLocal(null),
  };
  return (
    <label className="field">
      <span className="field-label">
        {label}
        {unit && <span className="muted"> ({unit})</span>}
        <Tooltip title={help} trigger={["hover", "focus"]}>
          <span className="hint-icon" tabIndex={0} role="img" aria-label={help}>
            ?
          </span>
        </Tooltip>
      </span>
      {text ? (
        <Input {...common} onChange={(e) => type(e.target.value)} />
      ) : (
        <InputNumber<string>
          {...common}
          stringMode
          controls={false}
          style={FULL}
          min="0"
          step={integer ? 1 : step}
          onChange={(v) => type(v ?? "")}
        />
      )}
    </label>
  );
}

/** A key's number in the inspector. While focused it keeps exactly what is typed, empty included: the value
 *  the key had when the field was focused shows as the placeholder and is what an empty field keeps. */
function KeyNumber({
  label,
  unit,
  min,
  max,
  step,
  value,
  disabled,
  onChange,
}: {
  label: string;
  unit?: string;
  min?: number;
  max?: number;
  step: number;
  value: number;
  disabled: boolean;
  onChange: (v: number) => void;
}) {
  const [local, setLocal] = useState<string | null>(null);
  const [before, setBefore] = useState(value);
  return (
    <label className="field">
      <span className="field-label">
        {label}
        {unit && <span className="muted"> ({unit})</span>}
      </span>
      <InputNumber<string>
        stringMode
        style={FULL}
        min={min === undefined ? undefined : String(min)}
        max={max === undefined ? undefined : String(max)}
        step={step}
        value={local ?? String(value)}
        placeholder={String(local === null ? value : before)}
        disabled={disabled}
        onFocus={() => {
          setBefore(value);
          setLocal(String(value));
        }}
        onBlur={() => setLocal(null)}
        onChange={(v) => {
          const t = (v ?? "").trim();
          setLocal(t);
          if (t === "") {
            if (value !== before) onChange(before);
            return;
          }
          const n = Number(t);
          if (Number.isFinite(n)) onChange(n);
        }}
      />
    </label>
  );
}

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
    <KeyNumber
      // Not keyed by the key's index: typing a show time can reorder the keys, and the field must keep focus.
      // Picking another key blurs the field first, which drops what was typed.
      key={`${channel}-${field}`}
      label={label}
      unit={opts.unit}
      min={opts.min}
      max={opts.max}
      step={opts.step ?? 0.1}
      value={key[field] as number}
      disabled={disabled}
      onChange={(v) => set(field, v)}
    />
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
            <Select<RtkState>
              style={FULL}
              value={key.state as RtkState}
              disabled={disabled}
              onChange={(v) => set("state", v)}
              options={[
                { value: "fixed", label: `${RTK_LABEL.fixed} (centimetre accuracy)` },
                { value: "float", label: `${RTK_LABEL.float} (about 0.3 m)` },
                { value: "gps", label: `${RTK_LABEL.gps} (about 1.5 m)` },
              ]}
            />
          </label>
        )}
        {channel === "rain" && (
          <>
            {num("mm_h", "Intensity", { min: 0, max: 200, unit: "mm/h" })}
            <p className="muted small">{rainLabel(key.mm_h as number)}</p>
          </>
        )}
      </div>
      <Button
        type="link"
        style={{ paddingInline: 0 }}
        disabled={disabled}
        onClick={() =>
          onChange({ ...scenario, [channel]: (scenario[channel] as unknown[]).filter((_, i) => i !== selected.index) }, null)
        }
      >
        Delete this key
      </Button>
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
        <Button
          danger
          size="large"
          loading={job.cancelling}
          disabled={job.jobId < 0}
          onClick={() => cancelSimulate(runId).catch((e) => setCancelError(String(e)))}
        >
          {job.cancelling ? "Cancelling…" : "Cancel simulation"}
        </Button>
      </div>
      <JobProgress pct={p ? Math.round(fraction * 100) : null} />
      {cancelError && <Alert type="error" showIcon title={cancelError} />}
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
  // Lists default to empty, so a result written by an earlier build still shows.
  const home: RainReturn | null = r.rain.return
    ? { ...r.rain.return, not_home: r.rain.return.not_home ?? [], off_slot_drones: r.rain.return.off_slot_drones ?? [] }
    : null;
  const late = !!home && (!home.all_home || home.home_by_deadline === false);
  const headline = r.passed
    ? home && home.formation >= 0
      ? "Every drone was home before the rain got too heavy"
      : "The show held up in this weather"
    : crashes
      ? `${crashes} ${crashes === 1 ? "pair" : "pairs"} of drones came within ${metres(r.criteria.d_crash_m, 1)}`
      : late
        ? !home!.all_home
          ? `${home!.not_home.length} ${home!.not_home.length === 1 ? "drone didn't" : "drones didn't"} get home`
          : `The last drone got home ${seconds(-(home!.spare_sec ?? 0))} after the rain limit`
        : `${low} ${low === 1 ? "drone landed" : "drones landed"} with less than ${pct(r.criteria.min_landing_soc)} battery`;
  return (
    <section className="sim-result" aria-label="Result">
      {stale && (
        <Alert
          type="warning"
          showIcon
          title="This result is from an earlier Stage 2 result. Simulate again to fly the current paths."
        />
      )}
      {edited && !stale && (
        <Alert
          type="warning"
          showIcon
          title="The scenario changed after this simulation. Simulate again to see the effect."
        />
      )}
      <VerdictCard tone={r.passed ? "ok" : "bad"} title={headline}>
        <dl className="stat-cards">
          <div>
            <dt>Closest approach</dt>
            <dd className={(r.closest?.distance_m ?? Infinity) < r.criteria.d_crash_m ? "tone-bad" : (r.closest?.distance_m ?? Infinity) < r.criteria.d_warning_m ? "tone-warn" : undefined}>
              {r.closest ? (
                <Button type="link" className="fact-link" onClick={() => onOpen(r.closest!.time_sec, [r.closest!.a, r.closest!.b])}>
                  {metres(r.closest.distance_m, 2)}
                </Button>
              ) : (
                "over 3 m"
              )}
            </dd>
          </div>
          <div>
            <dt>Largest deviation from plan</dt>
            <dd className={r.largest_deviation.distance_m > 0.5 ? "tone-warn" : undefined}>
              <Button type="link" className="fact-link" onClick={() => onOpen(r.largest_deviation.time_sec, [r.largest_deviation.drone])}>
                {metres(r.largest_deviation.distance_m, 2)}
              </Button>
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
                <Button type="link" className="fact-link" onClick={() => onOpen(r.rain.alert_time_sec!, [])}>
                  alert at {formatTime(r.rain.alert_time_sec)}
                </Button>
              )}
            </dd>
          </div>
        </dl>
        {home && home.formation >= 0 && <RainReturnFacts home={home} reaction={r.rain.reaction_s} onOpen={onOpen} />}
        {r.rain.not_applied && (
          <Alert type="warning" showIcon title={`The rain rule wasn't applied: ${r.rain.not_applied}.`} />
        )}
        <p className="muted small">
          {r.closest && <>Closest pair: drones {r.closest.a} and {r.closest.b} at {formatTime(r.closest.time_sec)}. </>}
          Furthest from plan: drone {r.largest_deviation.drone} at {formatTime(r.largest_deviation.time_sec)}.{" "}
          {r.rain.alert_time_sec !== null && (
            <>
              Rain reaches the alert level at {formatTime(r.rain.alert_time_sec)}
              {r.rain.limit_time_sec !== null && <> and the limit at {formatTime(r.rain.limit_time_sec)}</>}.{" "}
            </>
          )}
          Flown on {r.device.replace(/\s*\(opencl:\d+:\d+\)$/, "").replace(/\((R|TM)\)/g, "")} in{" "}
          {formatDuration(r.wall_time_sec)} ({r.realtime_factor.toFixed(1)}× real time).
        </p>
        {r.crash_pairs.length > 0 && (
          <Table<ScenarioResult["crash_pairs"][number]>
            size="small"
            pagination={false}
            scroll={{ y: 400 }}
            rowKey={(p) => `${p.drone_a}-${p.drone_b}`}
            dataSource={r.crash_pairs.slice(0, 20)}
            columns={[
              { title: "Drones", key: "drones", render: (_, p) => `${p.drone_a} and ${p.drone_b}` },
              {
                title: "Closest",
                key: "closest",
                align: "right",
                className: "num tone-bad",
                render: (_, p) => metres(p.min_distance_m, 2),
              },
              { title: "At", key: "at", align: "right", className: "num", render: (_, p) => formatTime(p.time_sec) },
              {
                title: <span className="visually-hidden">Actions</span>,
                key: "action",
                align: "right",
                render: (_, p) => (
                  <Button type="link" size="small" onClick={() => onOpen(p.time_sec, [p.drone_a, p.drone_b])}>
                    Show in replay
                  </Button>
                ),
              },
            ]}
          />
        )}
        {entry.playback && (
          <div className="actions">
            <Button type="primary" size="large" onClick={() => onOpen(0, [])}>
              Watch in Replay
            </Button>
            <span className="muted small">Times and values above open the replay at that moment.</span>
          </div>
        )}
      </VerdictCard>
    </section>
  );
}

const seconds = (v: number) => `${Math.round(v)} s`;

const HOW: Record<RainReturn["method"], string> = {
  return_path: "its planned return path",
  abort_point: "the return planned from that moment in the move",
  return_leg: "the show's own return leg",
  rest_of_show: "the rest of the show (no return path from there)",
  none: "nothing: there is no way home from there",
  landed: "nothing: the show had landed",
};

function RainReturnFacts({
  home,
  reaction,
  onOpen,
}: {
  home: RainReturn;
  reaction: number | undefined;
  onOpen: (time: number, drones: number[]) => void;
}) {
  const deadline = home.deadline_sec;
  const spare = home.spare_sec;
  return (
    <div className="rain-return">
      <h3>The return to the holding area</h3>
      <dl className="stat-cards">
        <div>
          <dt>Return called</dt>
          <dd>
            <Button type="link" className="fact-link" onClick={() => onOpen(home.command_sec, [])}>
              {formatTime(home.command_sec)}
            </Button>
          </dd>
        </div>
        <div>
          <dt>Flying home from</dt>
          <dd>
            <Button type="link" className="fact-link" onClick={() => onOpen(home.start_sec, [])}>
              {home.method === "abort_point" ? `the move to ${home.formation_name}` : (home.formation_name ?? "n/a")} at{" "}
              {formatTime(home.start_sec)}
            </Button>
          </dd>
        </div>
        <div>
          <dt>Last drone home</dt>
          <dd className={home.all_home ? undefined : "tone-bad"}>
            {home.last_landing_sec !== null && home.all_home ? (
              <Button type="link" className="fact-link" onClick={() => onOpen(home.last_landing_sec!, home.last_drone !== null ? [home.last_drone] : [])}>
                {formatTime(home.last_landing_sec)}
              </Button>
            ) : (
              `${home.not_home.length} not home`
            )}
          </dd>
        </div>
        <div>
          <dt>Rain limit</dt>
          <dd className={deadline === null ? undefined : !home.all_home || (spare ?? 0) < 0 ? "tone-bad" : "tone-ok"}>
            {deadline === null ? (
              "not reached"
            ) : (
              <Button type="link" className="fact-link" onClick={() => onOpen(deadline, [])}>
                {formatTime(deadline)}
                {!home.all_home ? (
                  " (missed)"
                ) : (
                  spare !== null && <> ({spare >= 0 ? `${seconds(spare)} to spare` : `${seconds(-spare)} late`})</>
                )}
              </Button>
            )}
          </dd>
        </div>
      </dl>
      <p className="muted small">
        The return was called at {formatTime(home.command_sec)}
        {reaction !== undefined && <>, {seconds(reaction)} after the rain alert</>}.{" "}
        {home.method === "abort_point" ? (
          <>
            The fleet flew on to {formatTime(home.start_sec)} in the move to {home.formation_name} and turned for home
            there, on the return planned from that moment
          </>
        ) : (
          <>The fleet finished the move it was in and flew {HOW[home.method]}</>
        )}
        {home.planned_home_sec !== null && <>; the plan has it home at {formatTime(home.planned_home_sec)}</>}
        {home.lag_sec !== null && home.all_home && (
          <> and the last drone was within {metres(home.home_radius_m, 1)} of its slot {home.lag_sec >= 0 ? `${home.lag_sec.toFixed(1)} s after that` : `${(-home.lag_sec).toFixed(1)} s before that`}</>
        )}
        .{" "}
        {home.not_home.length > 0 && <>Not home at the end: drones {home.not_home.slice(0, 12).join(", ")}{home.not_home.length > 12 ? " and more" : ""}. </>}
        {home.farthest_from_slot_m !== null && (
          <>
            Farthest from its slot at the end: drone {home.farthest_from_slot_drone}, {metres(home.farthest_from_slot_m, 2)}
            {home.off_slot_drones.length > 0 && <> ({home.off_slot_drones.length} more than 0.3 m off)</>}.
          </>
        )}
      </p>
    </div>
  );
}
