// Rain return readiness: for every moment of the show, can the
// fleet be home before the rain gets too heavy, and if not, what would close the gap? The chart shares
// the timeline's time axis.

import { useEffect, useMemo, useRef, useState } from "react";
import type { Readiness, RunRecord, Scenario, Suggestions } from "../bridge/types";
import { runJob } from "../bridge/api";
import { coverage, spans } from "../app/readiness";
import { cancelReturns, startReturns, useReturnsJob } from "../app/returnsJobs";
import { formatTime } from "../replay/sampling";
import { LABEL_W, PAD_R } from "./TimelineEditor";
import { ReturnsRunning } from "./ReturnPaths";
import { SuggestionList } from "./SuggestionList";

interface Props {
  run: RunRecord;
  readiness: Readiness;
  scenario: Scenario;
  /** Seconds the timeline covers, so the chart lines up with it. */
  duration: number;
  /** The scenario's own window: from the rain alert to the limit level, when its rain reaches both. */
  scenarioWindow: number | null;
  disabled: boolean;
  onPlanned: () => void;
  onFlyThis: (alertTime: number, rainWindow: number) => void;
  /** "Earlier trigger": set this scenario's alert level (an unsaved edit). */
  onSetAlert: (alertMmH: number) => void;
}

const H = 132;
const PAD_T = 10;
const PAD_B = 22;

const seconds = (v: number) => `${Math.round(v)} s`;

export function ReadinessPanel({ run, readiness, scenario, duration, scenarioWindow, disabled, onPlanned, onFlyThis, onSetAlert }: Props) {
  const hostRef = useRef<HTMLDivElement>(null);
  const [width, setWidth] = useState(800);
  const job = useReturnsJob(run.run_id);
  const planning = !!job && !job.exit;
  const [planError, setPlanError] = useState<string | null>(null);
  /** The suggestion whose button started the running job: its progress shows in that row, where the click was. */
  const [planFrom, setPlanFrom] = useState<number | null>(null);
  useEffect(() => {
    if (!planning) setPlanFrom(null);
  }, [planning]);
  const { reaction_s: reaction, margin_s: margin } = scenario.rain_rule;

  useEffect(() => {
    const ro = new ResizeObserver(([e]) => setWidth(Math.max(320, e.contentRect.width)));
    ro.observe(hostRef.current!);
    return () => ro.disconnect();
  }, []);

  const end = readiness.end_sec;
  const base = useMemo(() => coverage(readiness.pieces, end, reaction, margin, Infinity), [readiness, end, reaction, margin]);
  const [rainWindow, setRainWindow] = useState<number>(() => Math.round(scenarioWindow ?? base.required ?? 60));
  useEffect(() => {
    if (scenarioWindow !== null) setRainWindow(Math.round(scenarioWindow * 10) / 10);
  }, [scenarioWindow]);
  const cov = useMemo(() => coverage(readiness.pieces, end, reaction, margin, rainWindow), [readiness, end, reaction, margin, rainWindow]);
  const list = useMemo(() => spans(readiness.pieces, end, reaction, margin), [readiness, end, reaction, margin]);
  const [picked, setPicked] = useState<number | null>(null);
  const [hover, setHover] = useState<number | null>(null);

  const plotW = width - LABEL_W - PAD_R;
  const x = (t: number) => LABEL_W + (Math.min(Math.max(t, 0), duration) / Math.max(duration, 1e-6)) * plotW;
  const tOf = (px: number) => Math.min(end, Math.max(0, ((px - LABEL_W) / plotW) * duration));
  const finite = list.filter((s) => s.needed !== null).map((s) => s.needed as number);
  const yMax = Math.max(30, rainWindow * 1.25, ...finite.map((v) => v * 1.08));
  const y = (v: number) => PAD_T + (1 - Math.min(v, yMax) / yMax) * (H - PAD_T - PAD_B);
  const sliderMax = Math.max(120, Math.ceil(((base.required ?? 120) * 1.5) / 10) * 10);

  const entries = readiness.returns.entries;
  const last = readiness.formations.length - 1;
  const missing = readiness.formations.filter(
    (f) => f.return_sec === null && !(f.index === last && readiness.return_leg_sec !== null),
  );
  const failed = entries.filter((e) => e.status !== "succeeded");

  /** Plan return paths: by default the missing ones (all when none are planned or they are out of date). */
  function plan(
    formations: number[] | null = readiness.returns.stale || !readiness.returns.planned ? null : missing.map((f) => f.index),
    points: [number, number][] = [],
  ) {
    setPlanError(null);
    startReturns(run.run_id, run.run_dir, formations, () => onPlanned(), points).catch((e) => setPlanError(String(e)));
  }

  // Suggestions come from the bridge for one window and rule; they go out of date when either, or the
  // planned return paths, change.
  const rule = scenario.rain_rule;
  const suggestKey = [rainWindow, reaction, margin, rule.alert_mm_h, rule.limit_mm_h, JSON.stringify(readiness.pieces)].join("|");
  const [suggested, setSuggested] = useState<{ key: string; result: Suggestions } | null>(null);
  const [suggesting, setSuggesting] = useState(false);
  const [suggestError, setSuggestError] = useState<string | null>(null);
  async function suggest() {
    setSuggesting(true);
    setSuggestError(null);
    const key = suggestKey;
    try {
      const args = ["suggest", run.run_dir, "--window-s", String(rainWindow), "--reaction-s", String(reaction),
        "--margin-s", String(margin), "--alert-mm-h", String(rule.alert_mm_h), "--limit-mm-h", String(rule.limit_mm_h)];
      const { events, exit } = await runJob(args);
      const found = events.find((e) => e.type === "suggestions");
      if (exit.code !== 0 || !found || found.type !== "suggestions") {
        const err = events.find((e) => e.type === "error");
        throw new Error(err && err.type === "error" ? err.message : `exit code ${exit.code}`);
      }
      const { type: _type, ...result } = found;
      setSuggested({ key, result });
    } catch (e) {
      setSuggestError(String(e));
    } finally {
      setSuggesting(false);
    }
  }

  /** What happens if the alert comes at t. */
  function explain(t: number): string {
    const s = list.find((sp) => sp.a <= t && t < sp.b);
    if (!s) return "";
    const needed = s.needed === null ? null : s.needed + s.slope * (t - s.a);
    const name = readiness.formations[s.formation]?.name ?? "";
    const how =
      s.method === "abort_point" && s.pointSec != null
        ? `fly on to ${formatTime(s.pointSec)} in the move to ${name}, then the return planned from there`
        : s.method === "return_path"
        ? `finish the move to ${name}, then fly its return`
        : s.method === "return_leg"
          ? "fly the show's own return leg"
          : s.method === "rest_of_show"
            ? `no return from ${name}: fly the rest of the show`
            : s.method === "landed"
              ? "the show has landed"
              : `no way home from ${name}`;
    return `Alert at ${formatTime(t)}: ${how}; ${needed === null ? "can't be covered" : `${seconds(needed)} needed of ${seconds(rainWindow)}`}`;
  }

  const readout = hover ?? picked;
  const uncoveredText = cov.uncovered
    .slice(0, 3)
    .map((u) => `${formatTime(u.start)}–${formatTime(u.end)}`)
    .join(", ");
  const shortest = cov.uncovered.reduce((m, u) => Math.max(m, u.shortBy), 0);

  return (
    <section className="readiness" aria-labelledby="readiness-title">
      <h2 id="readiness-title" className="readiness-title">
        Rain return readiness
      </h2>
      <p className="muted readiness-about">
        If it rains, the show stops and every drone returns to the holding area. For each moment the rain could reach
        the alert level, this is the time until every drone is home: {seconds(reaction)} to react, the flight home from
        the planned paths, and a {seconds(margin)} margin. It must fit in the time the rain takes to go from the alert
        to the limit level.
      </p>

      {readiness.returns.stale && (
        <p className="notice notice-warn">The return paths were planned from an earlier Stage 2 result. Plan them again.</p>
      )}
      {planning && planFrom === null ? (
        <div className="running-head readiness-planning">
          <span className="light light-busy" aria-hidden="true" />
          <div>
            <p className="running-phase">
              Planning return paths
              {job!.current &&
                `: ${job!.current.abortTime !== null ? `from ${formatTime(job!.current.abortTime)} in the move to ` : ""}${readiness.formations[job!.current.keyframe]?.name ?? ""} (${job!.current.index + 1} of ${job!.current.count})`}
            </p>
            <p className="muted small">Each one is planned like a show transition; this can take several minutes each.</p>
          </div>
          <button className="danger" disabled={job!.cancelling} onClick={() => cancelReturns(run.run_id)}>
            {job!.cancelling ? "Cancelling…" : "Cancel"}
          </button>
        </div>
      ) : (
        !planning &&
        (!readiness.returns.planned || missing.length > 0) && (
          <div className="notice readiness-plan">
            <p>
              {!readiness.returns.planned
                ? "No return paths are planned yet. Without them the fleet can only fly the rest of the show to its own return leg."
                : `No return path from ${missing.map((f) => f.name).join(", ")}${failed.length ? " (planning failed)" : ""}: from there the fleet flies the rest of the show.`}{" "}
              Each return is planned like a show transition, so it takes about as long as one (on the 150-drone
              cone, about 7 minutes each).
            </p>
            <div className="actions">
              <button className="primary" onClick={() => plan()} disabled={disabled}>
                Plan return paths
              </button>
            </div>
            {planError && <p className="tone-bad">{planError}</p>}
          </div>
        )
      )}

      <div className="readiness-chart" ref={hostRef}>
        <svg
          width={width}
          height={H}
          className="rc-svg"
          role="img"
          aria-label={`Time to get every drone home over the show; ${Math.round(cov.coveredFraction * 100)} % covered by a ${seconds(rainWindow)} rain window`}
          onPointerMove={(e) => setHover(tOf(e.clientX - e.currentTarget.getBoundingClientRect().left))}
          onPointerLeave={() => setHover(null)}
          onPointerDown={(e) => setPicked(tOf(e.clientX - e.currentTarget.getBoundingClientRect().left))}
        >
          <text x={0} y={(H - PAD_B + PAD_T) / 2} className="tl-lane-label" dominantBaseline="middle">
            Time home
          </text>
          <rect x={LABEL_W} y={PAD_T} width={plotW} height={H - PAD_T - PAD_B} className="tl-lane" />
          {cov.uncovered.map((u, i) => (
            <rect key={i} x={x(u.start)} y={PAD_T} width={Math.max(1, x(u.end) - x(u.start))} height={H - PAD_T - PAD_B} className="rc-uncovered" />
          ))}
          {[0.5, 1].map((f) => (
            <text key={f} x={LABEL_W + 4} y={y(f * yMax) + 11} className="tl-scale">
              {seconds(f * yMax)}
            </text>
          ))}
          {list.map((s, i) =>
            s.needed === null ? (
              <rect key={i} x={x(s.a)} y={PAD_T} width={Math.max(1, x(s.b) - x(s.a))} height={H - PAD_T - PAD_B} className="rc-none" />
            ) : (
              <line key={i} x1={x(s.a)} x2={x(s.b)} y1={y(s.needed)} y2={y(s.needed + s.slope * (s.b - s.a))} className="rc-line" />
            ),
          )}
          <line x1={LABEL_W} x2={LABEL_W + plotW} y1={y(rainWindow)} y2={y(rainWindow)} className="rc-window" />
          <text x={LABEL_W + plotW - 4} y={y(rainWindow) - 4} className="rc-window-label" textAnchor="end">
            rain window {seconds(rainWindow)}
          </text>
          {picked !== null && <line x1={x(picked)} x2={x(picked)} y1={PAD_T - 4} y2={H - PAD_B} className="rc-pick" />}
          {hover !== null && <line x1={x(hover)} x2={x(hover)} y1={PAD_T} y2={H - PAD_B} className="rc-hover" />}
        </svg>
      </div>
      <p className="readiness-readout small" aria-live="polite">
        {readout !== null ? explain(readout) : "Point at the chart to see what happens if the alert comes then; click to pick a moment."}
      </p>

      <div className="readiness-controls">
        <label className="field readiness-window">
          <span className="field-label">
            Rain goes from alert to limit in <span className="num">{seconds(rainWindow)}</span>
            {scenarioWindow !== null && Math.abs(scenarioWindow - rainWindow) > 0.05 && (
              <button className="link" onClick={() => setRainWindow(Math.round(scenarioWindow * 10) / 10)}>
                {" "}
                (this scenario: {seconds(scenarioWindow)})
              </button>
            )}
          </span>
          <input type="range" min={5} max={sliderMax} step={1} value={Math.min(rainWindow, sliderMax)} onChange={(e) => setRainWindow(Number(e.target.value))} />
        </label>
        <button
          onClick={() => picked !== null && onFlyThis(picked, rainWindow)}
          disabled={disabled || picked === null}
          title={picked === null ? "Click the chart to pick the moment the rain reaches the alert level" : undefined}
        >
          {picked === null ? "Fly this" : `Fly this: alert at ${formatTime(picked)}`}
        </button>
      </div>

      <dl className="facts facts-wide">
        <div>
          <dt>Covered</dt>
          <dd className={cov.coveredFraction >= 1 ? "tone-ok" : "tone-bad"}>{Math.floor(cov.coveredFraction * 1000) / 10} % of the show</dd>
        </div>
        <div>
          <dt>Window this show needs</dt>
          <dd>{cov.required === null ? "can't be covered" : seconds(Math.ceil(cov.required))}</dd>
        </div>
        <div>
          <dt>Hardest moment</dt>
          <dd>{cov.worst ? formatTime(cov.worst.time) : "n/a"}</dd>
        </div>
      </dl>
      <p className="muted small">
        {cov.uncovered.length === 0
          ? `Every moment is covered with this window.`
          : `Not covered: ${uncoveredText}${cov.uncovered.length > 3 ? ` and ${cov.uncovered.length - 3} more` : ""} (short by up to ${Number.isFinite(shortest) ? seconds(shortest) : "all of it"}).`}{" "}
        {cov.required !== null &&
          `The show is covered if the rain takes at least ${seconds(Math.ceil(cov.required))} to go from the alert to the limit level.`}
      </p>

      {cov.uncovered.length > 0 && (
        <div className="readiness-suggest">
          <div className="readiness-suggest-head">
            <h3>What would close the gap</h3>
            <button onClick={() => void suggest()} disabled={disabled || suggesting || planning}>
              {suggesting ? "Working it out…" : suggested ? "Suggest again" : `Suggest for ${seconds(rainWindow)}`}
            </button>
          </div>
          {!suggested && !suggesting && (
            <p className="muted small">
              For this window, the options that would cover more of the show, with their numbers. New return paths are
              estimated in a moment; planning them takes longer.
            </p>
          )}
          {suggested && suggested.key !== suggestKey && (
            <p className="notice notice-warn small">
              These were worked out for a {seconds(suggested.result.window_sec)} window and the return paths planned
              then. Suggest again to bring them up to date.
            </p>
          )}
          {suggestError && <p className="notice notice-bad">Could not work out suggestions: {suggestError}</p>}
          {suggested && (
            <SuggestionList
              result={suggested.result}
              current={suggested.key === suggestKey}
              disabled={disabled || planning}
              busyIndex={planning ? planFrom : null}
              progress={planning ? <ReturnsRunning job={job!} runId={run.run_id} names={run.input.keyframes} /> : null}
              onStart={setPlanFrom}
              onPlanPoints={(points) => plan(null, points)}
              onPlanReturn={(k) => plan([k])}
              onSetAlert={onSetAlert}
            />
          )}
        </div>
      )}
    </section>
  );
}
