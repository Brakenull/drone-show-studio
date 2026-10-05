// Run Stage 2 and read its outcome.

import { useEffect, useState } from "react";
import { readRunJson, readRunText, runJob } from "../bridge/api";
import type { FailureSummary, JobExit, Overrides, RunRecord } from "../bridge/types";
import { PlannerSettings } from "./PlannerSettings";
import {
  cancelStage2,
  solveFraction,
  startStage2,
  useStage2Job,
  type SolveState,
  type Stage2Job,
} from "../app/stage2Jobs";
import { isStage3Running } from "../app/stage3Jobs";
import { isReturnsRunning } from "../app/returnsJobs";
import { ReturnPaths } from "./ReturnPaths";
import { VerdictCard } from "./VerdictCard";
import { clock, duration, STATUS } from "../app/format";
import { formatTime, metres } from "../replay/sampling";
import type { Separation } from "../replay/types";

interface Props {
  run: RunRecord;
  runsDir: string;
  onChanged: () => void;
  onFinished: (runId: string, exit: JobExit) => void;
  onShowInReplay: (time: number, drones: number[]) => void;
  /** Open a return path's replay (the show up to that formation, then the flight home) at its abort time. */
  onViewReturn: (keyframe: number | string, from: string, abortTime: number) => void;
  /** A copy of this run was made ("Try these settings in a new run"). */
  onCreated: (runId: string) => void;
}

const PHASE_TEXT: Record<string, string> = {
  starting: "Starting",
  loading: "Loading the path planner",
  solving: "Planning collision-free paths",
  replay: "Preparing the 3D replay",
};

export function Stage2View({ run, runsDir, onChanged, onFinished, onShowInReplay, onViewReturn, onCreated }: Props) {
  const job = useStage2Job(run.run_id);
  const running = !!job && !job.exit;
  const status = running ? "running" : run.stage2.status;
  const [startError, setStartError] = useState<string | null>(null);

  async function start(overrides: Overrides) {
    setStartError(null);
    const promise = startStage2(run.run_id, run.run_dir, overrides, (exit) => onFinished(run.run_id, exit));
    // run.json flips to "running" as soon as the bridge starts; refresh the sidebar shortly after.
    setTimeout(onChanged, 800);
    promise.catch((e) => setStartError(String(e)));
  }

  return (
    <div className="page">
      <header className="page-head">
        <h1>Stage 2: path planning</h1>
        <p className="lede">
          Assigns every drone to a spot in each formation and plans smooth paths between them, then checks every
          pair of drones at 100 samples per second. If any pair gets too close, Stage 2 rejects the show and
          shows where.
        </p>
      </header>

      {startError && (
        <p className="notice notice-bad" role="alert">
          Could not start Stage 2: {startError}
        </p>
      )}

      {running ? (
        <RunningPanel job={job!} fleet={run.input.fleet_size} runId={run.run_id} />
      ) : (
        <Outcome
          run={run}
          status={status}
          job={job}
          controls={
            <PlannerSettings
              run={run}
              runLabel={status === "not_run" ? "Run Stage 2" : "Run Stage 2 again"}
              primary={status === "not_run"}
              // Stage 3 reads this run's Stage 2 output; re-planning would change it underneath.
              // Re-planning also deletes the return paths planned from this result.
              blocked={
                isStage3Running(run.run_id)
                  ? "Wait for Stage 3 to finish first."
                  : isReturnsRunning(run.run_id)
                    ? "Wait for the return paths to finish planning first."
                    : null
              }
              onRun={start}
              onCopy={async (overrides) => {
                const { events } = await runJob([
                  "new-run",
                  `${run.run_dir}/input/phase1.json`,
                  "--runs-dir",
                  runsDir,
                  "--copy-from",
                  run.run_dir,
                  "--overrides-json",
                  JSON.stringify(overrides),
                ]);
                const created = events.find((e) => e.type === "run_created");
                if (!created || created.type !== "run_created") {
                  const err = events.find((e) => e.type === "error");
                  throw new Error(err && err.type === "error" ? err.message : "The copy wasn't created.");
                }
                onCreated(created.run_id);
              }}
            />
          }
          onShowInReplay={onShowInReplay}
          onChanged={onChanged}
          onViewReturn={onViewReturn}
        />
      )}
    </div>
  );
}

function RunningPanel({ job, fleet, runId }: { job: Stage2Job; fleet: number; runId: string }) {
  const [now, setNow] = useState(Date.now());
  const [cancelError, setCancelError] = useState<string | null>(null);
  useEffect(() => {
    const id = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(id);
  }, []);
  const solving = job.phase === "solving" && job.solve ? job.solve : null;
  const pct = solving
    ? Math.round(solveFraction(solving) * 100)
    : job.progress
      ? Math.round((job.progress.done / Math.max(job.progress.total, 1)) * 100)
      : null;

  return (
    <section className="running" aria-live="polite">
      <div className="running-head">
        <span className="light light-busy" aria-hidden="true" />
        <div>
          <p className="running-phase">{PHASE_TEXT[job.phase] ?? job.phase}</p>
          <p className="muted">
            {fleet} drones, running for <span className="num">{clock(now - job.startedAt)}</span>
          </p>
        </div>
        <button
          className="danger"
          disabled={job.cancelling || job.jobId < 0}
          onClick={() => cancelStage2(runId).catch((e) => setCancelError(String(e)))}
        >
          {job.cancelling ? "Cancelling…" : "Cancel run"}
        </button>
      </div>
      {solving && <SolveStatus solve={solving} />}
      <div
        className={`bar ${pct === null ? "bar-indeterminate" : ""}`}
        title={solving ? "Approximate: a pass that converges early or a retry moves it in jumps" : undefined}
        role="progressbar"
        aria-valuenow={pct ?? undefined}
        aria-valuemin={0}
        aria-valuemax={100}
      >
        <span style={pct === null ? undefined : { width: `${pct}%` }} />
      </div>
      <p className="muted small">
        Large shows can take more than an hour. The run keeps going while you look at other runs; closing
        Studio stops it.
      </p>
      {cancelError && <p className="notice notice-bad">{cancelError}</p>}
      <LogPane lines={job.log} />
    </section>
  );
}

const place = (name: string) => (name === "holding_area" ? "holding area" : name);

function SolveStatus({ solve }: { solve: SolveState }) {
  const steps = [
    solve.attempt > 1 && `retry ${solve.attempt - 1} of ${solve.maxAttempts - 1}`,
    solve.substageCount > 1 && `part ${solve.substage} of ${solve.substageCount}`,
    solve.iteration > 0 && `refining pass ${solve.iteration} (up to ${solve.maxIterations})`,
  ].filter(Boolean);
  return (
    <div className="solve-status">
      <p>
        Transition <span className="num">{solve.transition + 1}</span> of{" "}
        <span className="num">{solve.transitionCount}</span>: {place(solve.from)} → {place(solve.to)}
      </p>
      {steps.length > 0 && <p className="muted small">{steps.join(" · ")}</p>}
      {solve.rejected.map((r) => (
        <p key={r.attempt} className="notice notice-warn small">
          Try {r.attempt} failed the safety check: two drones came within {metres(r.worst)} (needs{" "}
          {metres(r.required)}). Trying again with more time for this transition.
        </p>
      ))}
    </div>
  );
}

function LogPane({ lines }: { lines: string[] }) {
  if (!lines.length) return null;
  return (
    <details className="log">
      <summary>Output ({lines.length} lines)</summary>
      <pre>{lines.slice(-200).join("\n")}</pre>
    </details>
  );
}

interface OutcomeProps {
  run: RunRecord;
  status: RunRecord["stage2"]["status"];
  job: Stage2Job | null;
  /** Planner settings and the run buttons, shown under the result. */
  controls: React.ReactNode;
  onShowInReplay: (time: number, drones: number[]) => void;
  onChanged: () => void;
  onViewReturn: (keyframe: number | string, from: string, abortTime: number) => void;
}

function Outcome({ run, status, job, controls, onShowInReplay, onChanged, onViewReturn }: OutcomeProps) {
  if (status === "not_run") {
    return (
      <section className="outcome">
        <p>This run hasn't been through Stage 2 yet.</p>
        {run.copied_from && (
          <p className="muted">A copy of {run.copied_from}, to try other planner settings on the same show.</p>
        )}
        {controls}
      </section>
    );
  }
  const again = <div className="outcome-controls">{controls}</div>;
  const weaker = run.stage2.config_warnings ?? [];
  const note = weaker.length > 0 && (status === "succeeded" || status === "failed_safety") && (
    <div className="notice notice-warn">
      <p>
        <strong>This result used weaker safety settings than the defaults:</strong>
      </p>
      <ul>
        {weaker.map((w, i) => (
          <li key={i}>{w.message}</li>
        ))}
      </ul>
    </div>
  );
  if (status === "succeeded")
    return (
      <>
        {note}
        <Passed run={run} again={again} onChanged={onChanged} onViewReturn={onViewReturn} />
      </>
    );
  if (status === "failed_safety")
    return (
      <>
        {note}
        <Rejected run={run} again={again} onShowInReplay={onShowInReplay} />
      </>
    );
  return <Stopped run={run} status={status} job={job} again={again} />;
}

function Passed({
  run,
  again,
  onChanged,
  onViewReturn,
}: {
  run: RunRecord;
  again: React.ReactNode;
  onChanged: () => void;
  onViewReturn: (keyframe: number | string, from: string, abortTime: number) => void;
}) {
  const [sep, setSep] = useState<Separation | null>(null);
  useEffect(() => {
    readRunJson<Separation>(run.run_id, "stage2/replay/separation.json").then(setSep).catch(() => setSep(null));
  }, [run.run_id, run.stage2.ended_at]);
  return (
    <section className="outcome">
      <VerdictCard tone="ok" title="Stage 2 planned the whole show">
        <p>Every pair of drones stayed at or above the required distance for the full show.</p>
        <dl className="stat-cards">
          <div>
            <dt>Show length</dt>
            <dd>{duration(run.stage2.total_duration_sec)}</dd>
          </div>
          <div>
            <dt>Closest approach</dt>
            <dd>{sep ? metres(sep.worst.distance_m, 3) : "n/a"}</dd>
          </div>
          <div>
            <dt>Planning time</dt>
            <dd>{duration(run.stage2.wall_time_sec)}</dd>
          </div>
        </dl>
      </VerdictCard>
      <ReturnPaths run={run} onChanged={onChanged} onView={onViewReturn} />
      {again}
    </section>
  );
}

function Rejected({
  run,
  again,
  onShowInReplay,
}: {
  run: RunRecord;
  again: React.ReactNode;
  onShowInReplay: (time: number, drones: number[]) => void;
}) {
  const [failure, setFailure] = useState<FailureSummary | null>(null);
  const [loadError, setLoadError] = useState<string | null>(null);
  useEffect(() => {
    readRunJson<FailureSummary>(run.run_id, "stage2/failure.json")
      .then((f) => (f ? setFailure(f) : setLoadError("failure.json is missing from this run.")))
      .catch((e) => setLoadError(String(e)));
  }, [run.run_id, run.stage2.ended_at]);

  if (loadError) return <p className="notice notice-bad">{loadError}</p>;
  if (!failure) return <p className="status-line">Loading the failure report…</p>;
  const t = failure.transition;
  const where = (name: string) => (name === "holding_area" ? "the holding area" : name);
  return (
    <section className="outcome">
      <VerdictCard tone="bad" title="Stage 2 rejected this show">
        <p className="verdict-detail">
          Flying from {where(t.from_keyframe)} to {where(t.to_keyframe)}, two drones come within{" "}
          <strong className="tone-bad">{metres(failure.worst_separation_m, 3)}</strong> of each other. The safety check
          requires <strong>{metres(failure.required_separation_m, 2)}</strong>.{" "}
          {failure.violating_pair_count === 1
            ? "One pair is too close."
            : `${failure.violating_pair_count} pairs are too close.`}
        </p>

        <h3>Too-close pairs</h3>
        <table className="table data">
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
            {failure.violations.slice(0, 100).map((v) => (
              <tr key={`${v.drone_a}-${v.drone_b}`}>
                <td>
                  {v.drone_a} and {v.drone_b}
                </td>
                <td className="num tone-bad">{metres(v.distance_m, 3)}</td>
                <td className="num">{formatTime(v.time_sec)}</td>
                <td className="row-action">
                  <button className="link" onClick={() => onShowInReplay(v.time_sec, [v.drone_a, v.drone_b])}>
                    Show in replay
                  </button>
                </td>
              </tr>
            ))}
          </tbody>
        </table>
        {failure.violations_truncated && (
          <p className="muted small">Showing the {failure.violations.length} closest pairs.</p>
        )}

        <h3>What Stage 2 tried</h3>
        <p className="muted">
          Before rejecting a transition, Stage 2 retries it with more time and stronger spreading. If the closest
          distance doesn't improve, the formations themselves need to change.
        </p>
        <table className="table data">
          <thead>
            <tr>
              <th scope="col">Attempt</th>
              <th scope="col" className="num">Transition time</th>
              <th scope="col" className="num">Closest</th>
            </tr>
          </thead>
          <tbody>
            {failure.attempts.map((a) => (
              <tr key={a.attempt}>
                <td>{a.attempt}</td>
                <td className="num">{a.duration_sec.toFixed(1)} s</td>
                <td className="num tone-bad">{metres(a.worst_separation_m, 3)}</td>
              </tr>
            ))}
          </tbody>
        </table>
        <p className="muted small planning-took">Planning took {duration(run.stage2.wall_time_sec)}.</p>
      </VerdictCard>
      {again}
    </section>
  );
}

function Stopped({
  run,
  status,
  job,
  again,
}: {
  run: RunRecord;
  status: RunRecord["stage2"]["status"];
  job: Stage2Job | null;
  again: React.ReactNode;
}) {
  const [logTail, setLogTail] = useState<string[]>([]);
  useEffect(() => {
    if (job?.log.length) {
      setLogTail(job.log);
      return;
    }
    readRunText(run.run_id, "stage2/log.ndjson")
      .then((text) =>
        setLogTail(
          (text ?? "")
            .split("\n")
            .filter(Boolean)
            .map((l) => {
              try {
                const e = JSON.parse(l);
                return e.line ?? e.message ?? e.detail ?? JSON.stringify(e);
              } catch {
                return l;
              }
            }),
        ),
      )
      .catch(() => setLogTail([]));
  }, [run.run_id, job]);

  const cancelled = status === "cancelled";
  return (
    <section className="outcome">
      <VerdictCard
        tone={cancelled ? "idle" : STATUS[status].tone === "bad" ? "bad" : "warn"}
        title={cancelled ? "This run was cancelled" : "Stage 2 stopped with an error"}
      >
        {!cancelled && run.stage2.message && <p className="notice notice-bad">{run.stage2.message}</p>}
        {!cancelled && (
          <p className="muted">
            This isn't a safety verdict. The planner didn't finish, so nothing was checked. The output below usually
            says why.
          </p>
        )}
        <LogPane lines={logTail} />
      </VerdictCard>
      {again}
    </section>
  );
}
