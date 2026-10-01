// "Return paths" on a passed Stage 2 run (docs/4-condition_simulator.md B4, §6): plan, after the show
// passed, a checked flight from each formation straight back to the holding area.

import { useEffect, useState } from "react";
import { readRunJson } from "../bridge/api";
import type { JobExit, ReturnEntry, ReturnIndex, RunRecord } from "../bridge/types";
import { cancelReturns, startReturns, useReturnsJob, type ReturnsJob } from "../app/returnsJobs";
import { solveFraction } from "../app/stage2Jobs";
import { clock, duration, STATUS } from "../app/format";
import { formatTime, metres } from "../replay/sampling";

interface Props {
  run: RunRecord;
  onChanged: () => void;
}

interface Phase1Head {
  project_metadata: { legs?: { return?: unknown } };
}

const ROW_STATUS: Record<string, string> = {
  succeeded: "Planned",
  failed_safety: "Rejected",
  failed_error: "Error",
};

export function ReturnPaths({ run, onChanged }: Props) {
  const job = useReturnsJob(run.run_id);
  const running = !!job && !job.exit;
  const [index, setIndex] = useState<ReturnIndex | null>(null);
  const [hasReturnLeg, setHasReturnLeg] = useState(false);
  const [startError, setStartError] = useState<string | null>(null);
  const section = run.stage2_returns;

  useEffect(() => {
    readRunJson<Phase1Head>(run.run_id, "input/phase1.json")
      .then((p) => setHasReturnLeg(!!p?.project_metadata.legs?.return))
      .catch(() => setHasReturnLeg(false));
  }, [run.run_id]);
  useEffect(() => {
    readRunJson<ReturnIndex>(run.run_id, "stage2/returns/index.json")
      .then((i) => setIndex(i && i.stage2_ended_at === run.stage2.ended_at ? i : null))
      .catch(() => setIndex(null));
  }, [run.run_id, run.stage2.ended_at, section?.ended_at, job?.results.length]);

  const names = run.input.keyframes;
  const byFormation = new Map<number, ReturnEntry>((index?.returns ?? []).map((e) => [e.keyframe_index, e]));
  const coveredByLeg = (k: number) => hasReturnLeg && k === names.length - 1;
  const missing = names.map((_, k) => k).filter((k) => !coveredByLeg(k) && byFormation.get(k)?.status !== "succeeded");

  function plan(formations: number[] | null) {
    setStartError(null);
    startReturns(run.run_id, run.run_dir, formations, (_exit: JobExit) => onChanged()).catch((e) =>
      setStartError(String(e)),
    );
    setTimeout(onChanged, 800);
  }

  return (
    <section className="returns">
      <h3>Return paths</h3>
      <p className="muted">
        If the show has to stop, for example when rain starts, the drones fly from the formation they are at
        straight back to the holding area. Plan these flights now and each one is checked like the show itself.
        Each takes about as long to plan as one transition of the show.
      </p>

      <table className="table">
        <thead>
          <tr>
            <th scope="col">From</th>
            <th scope="col" className="num">Reached at</th>
            <th scope="col" className="num">Flight home</th>
            <th scope="col" className="num">Closest</th>
            <th scope="col">Status</th>
            <th scope="col">
              <span className="visually-hidden">Actions</span>
            </th>
          </tr>
        </thead>
        <tbody>
          {names.map((name, k) => {
            const entry = byFormation.get(k);
            const planning = running && job!.current?.keyframe === k;
            if (coveredByLeg(k)) {
              return (
                <tr key={k}>
                  <td>{name}</td>
                  <td className="num">–</td>
                  <td className="num">–</td>
                  <td className="num">–</td>
                  <td className="muted">The show's own return leg</td>
                  <td />
                </tr>
              );
            }
            const tone = planning ? "busy" : entry ? STATUS[entry.status].tone : "idle";
            return (
              <tr key={k}>
                <td>{name}</td>
                <td className="num">{entry ? formatTime(entry.abort_time_sec) : "–"}</td>
                <td className="num">{entry?.flown_duration_sec != null ? duration(entry.flown_duration_sec) : "–"}</td>
                <td
                  className={`num ${entry?.status === "failed_safety" ? "tone-bad" : ""}`}
                  title={entry?.required_separation_m ? `needs ${metres(entry.required_separation_m, 2)}` : undefined}
                >
                  {entry?.worst_separation_m != null ? metres(entry.worst_separation_m, 3) : "–"}
                </td>
                <td title={entry?.message ?? undefined}>
                  <span className={`light light-${tone}`} aria-hidden="true" />
                  {planning ? "Planning" : entry ? (ROW_STATUS[entry.status] ?? STATUS[entry.status].label) : "Not planned"}
                </td>
                <td className="row-action">
                  {!running && (
                    <button className="link" onClick={() => plan([k])}>
                      {entry ? "Plan again" : "Plan"}
                    </button>
                  )}
                </td>
              </tr>
            );
          })}
        </tbody>
      </table>

      {startError && <p className="notice notice-bad">Could not start planning: {startError}</p>}
      {running ? (
        <ReturnsRunning job={job!} runId={run.run_id} names={names} />
      ) : (
        <>
          {job?.exit && <Finished job={job} />}
          {missing.length > 0 && (
            <div className="returns-controls">
              <button className="primary" onClick={() => plan(missing)}>
                Plan return paths
              </button>
              <span className="muted small">
                {missing.length === 1 ? "1 formation" : `${missing.length} formations`}
              </span>
            </div>
          )}
        </>
      )}
    </section>
  );
}

function Finished({ job }: { job: ReturnsJob }) {
  if (job.exit?.cancelled) return <p className="notice small">Planning was cancelled. Finished return paths are kept.</p>;
  const failed = job.results.filter((r) => r.status !== "succeeded");
  if (job.errors.length && !job.results.length)
    return <p className="notice notice-bad">{job.errors[job.errors.length - 1]}</p>;
  if (!failed.length) return null;
  return (
    <p className="notice notice-warn small">
      {failed.length === 1 ? "One formation has" : `${failed.length} formations have`} no return path: the safety check
      rejected {failed.length === 1 ? "its flight" : "their flights"} home. Hover its status for the details.
    </p>
  );
}

function ReturnsRunning({ job, runId, names }: { job: ReturnsJob; runId: string; names: string[] }) {
  const [now, setNow] = useState(Date.now());
  const [cancelError, setCancelError] = useState<string | null>(null);
  useEffect(() => {
    const id = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(id);
  }, []);
  const cur = job.current;
  const within = job.solve ? solveFraction(job.solve) : 0;
  const pct = cur ? Math.round(((cur.index + within) / Math.max(cur.count, 1)) * 100) : null;
  const solve = job.solve;
  const steps = solve
    ? [
        solve.attempt > 1 && `retry ${solve.attempt - 1} of ${solve.maxAttempts - 1}`,
        solve.substageCount > 1 && `part ${solve.substage} of ${solve.substageCount}`,
        solve.iteration > 0 && `refining pass ${solve.iteration} (up to ${solve.maxIterations})`,
      ].filter(Boolean)
    : [];

  return (
    <div className="running returns-running" aria-live="polite">
      <div className="running-head">
        <span className="light light-busy" aria-hidden="true" />
        <div>
          <p className="running-phase">
            {cur
              ? `Return ${cur.index + 1} of ${cur.count}: ${names[cur.keyframe] ?? "formation"} to the holding area`
              : "Starting"}
          </p>
          <p className="muted">
            Running for <span className="num">{clock(now - job.startedAt)}</span>
            {steps.length > 0 && <> · {steps.join(" · ")}</>}
          </p>
        </div>
        <button
          className="danger"
          disabled={job.cancelling || job.jobId < 0}
          onClick={() => cancelReturns(runId).catch((e) => setCancelError(String(e)))}
        >
          {job.cancelling ? "Cancelling…" : "Cancel planning"}
        </button>
      </div>
      <div className={`bar ${pct === null ? "bar-indeterminate" : ""}`} role="progressbar"
        aria-valuenow={pct ?? undefined} aria-valuemin={0} aria-valuemax={100}>
        <span style={pct === null ? undefined : { width: `${pct}%` }} />
      </div>
      <p className="muted small">Finished return paths are saved as they complete, so cancelling keeps them.</p>
      {cancelError && <p className="notice notice-bad">{cancelError}</p>}
    </div>
  );
}
