// "Return paths" on a passed Stage 2 run (docs/4-condition_simulator.md B4, §6): plan, after the show
// passed, a checked flight from each formation straight back to the holding area.

import { useEffect, useRef, useState } from "react";
import { readRunJson, readRunText, runJob } from "../bridge/api";
import type { JobExit, ReturnEntry, ReturnIndex, RunRecord } from "../bridge/types";
import { cancelReturns, startReturns, useReturnsJob, type ReturnsJob } from "../app/returnsJobs";
import { solveFraction } from "../app/stage2Jobs";
import { clock, duration, STATUS } from "../app/format";
import { formatTime, metres } from "../replay/sampling";

interface Props {
  run: RunRecord;
  onChanged: () => void;
  /** Open the replay of a planned return at its abort time; `keyframe` is a formation index or an abort
   *  point's id. */
  onView: (keyframe: number | string, from: string, abortTime: number) => void;
}

interface Phase1Head {
  project_metadata: { legs?: { return?: unknown } };
}

const ROW_STATUS: Record<string, string> = {
  succeeded: "Planned",
  failed_safety: "Rejected",
  failed_error: "Error",
};

export function ReturnPaths({ run, onChanged, onView }: Props) {
  const job = useReturnsJob(run.run_id);
  const running = !!job && !job.exit;
  const [index, setIndex] = useState<ReturnIndex | null>(null);
  const [hasReturnLeg, setHasReturnLeg] = useState(false);
  const [startError, setStartError] = useState<string | null>(null);
  const section = run.stage2_returns;
  /** Formations whose 3D view (stage2/returns/replay_<k>/) exists on disk. */
  const [built, setBuilt] = useState<Set<number> | null>(null);
  const [replayCheck, setReplayCheck] = useState(0);
  /** The planned return whose missing 3D view the dialog offers to build. */
  const [toBuild, setToBuild] = useState<{ keyframe: number | string; from: string; abortTime: number } | null>(null);

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

  useEffect(() => {
    const planned = (index?.returns ?? []).filter((e) => e.status === "succeeded").map((e) => e.keyframe_index);
    let live = true;
    Promise.all(
      planned.map((k) =>
        readRunText(run.run_id, `stage2/returns/replay_${k}/replay.json`)
          .then((text) => (text !== null ? k : null))
          .catch(() => null),
      ),
    ).then((found) => {
      if (live) setBuilt(new Set(found.filter((k): k is number => k !== null)));
    });
    return () => {
      live = false;
    };
  }, [run.run_id, index, replayCheck]);

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
    <section className="returns card">
      <h3>Return paths</h3>
      <p className="muted">
        If the show has to stop, for example when rain starts, the drones fly from the formation they are at
        straight back to the holding area. Plan these flights now and each one is checked like the show itself.
        Each takes about as long to plan as one transition of the show. With staggered takeoff the drones stop at
        the first formation, so its flight home is simply the takeoff flown backwards.
      </p>

      <table className="table data">
        <thead>
          <tr>
            <th scope="col">From</th>
            <th scope="col" className="num">Reached at</th>
            <th scope="col" className="num">Flight home</th>
            <th scope="col" className="num">Closest</th>
            <th scope="col">Status</th>
            <th scope="col">3D view</th>
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
                  <td className="muted">In the show</td>
                  <td />
                </tr>
              );
            }
            const tone = planning ? "busy" : entry ? STATUS[entry.status].tone : "idle";
            const reversed = entry?.method === "reversed_takeoff" && entry.status === "succeeded";
            const viewable = entry?.status === "succeeded" ? (built ? built.has(k) : null) : undefined;
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
                  {planning
                    ? "Planning"
                    : reversed
                      ? "Takeoff flown backwards"
                      : entry
                        ? (ROW_STATUS[entry.status] ?? STATUS[entry.status].label)
                        : "Not planned"}
                </td>
                <td>
                  {viewable === undefined ? (
                    "–"
                  ) : viewable === null ? (
                    <span className="muted">Checking</span>
                  ) : (
                    <>
                      <span className={`light light-${viewable ? "ok" : "idle"}`} aria-hidden="true" />
                      {viewable ? "Ready" : "Not built"}
                    </>
                  )}
                </td>
                <td className="row-action">
                  {entry?.status === "succeeded" && (
                    <button
                      className="link"
                      onClick={() =>
                        viewable
                          ? onView(k, name, entry.abort_time_sec)
                          : setToBuild({ keyframe: k, from: name, abortTime: entry.abort_time_sec })
                      }
                    >
                      View
                    </button>
                  )}
                  {!running && !reversed && (
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

      {(index?.points?.length ?? 0) > 0 && (
        <AbortPointTable
          points={index!.points!}
          names={names}
          running={running}
          planningAt={running ? (job!.current?.abortTime ?? null) : null}
          onView={(id, from, t, built) => (built ? onView(id, from, t) : setToBuild({ keyframe: id, from, abortTime: t }))}
          onPlan={(k, t) => {
            setStartError(null);
            startReturns(run.run_id, run.run_dir, null, () => onChanged(), [[k, t]]).catch((e) => setStartError(String(e)));
            setTimeout(onChanged, 800);
          }}
        />
      )}

      {startError && <p className="notice notice-bad">Could not start planning: {startError}</p>}
      {toBuild && (
        <BuildViewDialog
          runDir={run.run_dir}
          fleet={run.input.fleet_size}
          target={toBuild}
          onClose={() => setToBuild(null)}
          onBuilt={() => {
            setReplayCheck((n) => n + 1);
            setToBuild(null);
            onView(toBuild.keyframe, toBuild.from, toBuild.abortTime);
          }}
        />
      )}
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

/** Asks before building a return's missing 3D view, builds it in place, then opens it. */
function BuildViewDialog({
  runDir,
  fleet,
  target,
  onClose,
  onBuilt,
}: {
  runDir: string;
  fleet: number;
  target: { keyframe: number | string; from: string };
  onClose: () => void;
  onBuilt: () => void;
}) {
  const ref = useRef<HTMLDialogElement>(null);
  const primaryRef = useRef<HTMLButtonElement>(null);
  const [building, setBuilding] = useState(false);
  const [error, setError] = useState<string | null>(null);
  useEffect(() => {
    ref.current?.showModal();
    primaryRef.current?.focus(); // showModal() focuses the first button; the expected action is building
  }, []);

  async function build() {
    setBuilding(true);
    setError(null);
    try {
      const { exit, events } = await runJob(["replay", runDir, "--return", String(target.keyframe)]);
      if (exit.code !== 0) {
        const err = events.find((e) => e.type === "error");
        throw new Error(err && err.type === "error" ? err.message : `exit code ${exit.code}`);
      }
      onBuilt();
    } catch (e) {
      setError(String(e));
      setBuilding(false);
    }
  }

  return (
    <dialog
      ref={ref}
      className="dialog"
      aria-labelledby="build-view-title"
      // Escape closes the dialog; while building it stays open until the build ends.
      onCancel={(e) => {
        e.preventDefault();
        if (!building) onClose();
      }}
    >
      <h2 id="build-view-title">No 3D view yet</h2>
      <p>
        The flight home from {target.from} hasn't been prepared for the 3D view. Building it samples the show up to{" "}
        {target.from} and the flight home; for {fleet} drones that takes{" "}
        {fleet >= 150 ? "about 10 to 20 seconds" : "a few seconds"}.
      </p>
      {building && <div className="bar bar-indeterminate" role="progressbar" aria-label="Building the 3D view"><span /></div>}
      {error && <p className="notice notice-bad">Could not build the 3D view: {error}</p>}
      <div className="dialog-actions">
        <button onClick={onClose} disabled={building}>
          Not now
        </button>
        <button ref={primaryRef} className="primary" onClick={build} disabled={building}>
          {building ? "Building…" : "Build and view"}
        </button>
      </div>
    </dialog>
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

/** A running `stage2-returns` job: which return, time running, the solver's step, a progress bar and Cancel.
 *  Also shown in the readiness suggestions, in the row whose button started it. */
export function ReturnsRunning({ job, runId, names }: { job: ReturnsJob; runId: string; names: string[] }) {
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
              ? `Return ${cur.index + 1} of ${cur.count}: ${cur.abortTime !== null ? `from ${formatTime(cur.abortTime)} in the move to ` : ""}${names[cur.keyframe] ?? "formation"} to the holding area`
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

/** Returns planned from moments inside a move (docs/4-condition_simulator.md §5.3), from the readiness
 *  suggestions: the fleet turns for home there instead of finishing the move. */
function AbortPointTable({
  points,
  names,
  running,
  planningAt,
  onView,
  onPlan,
}: {
  points: ReturnEntry[];
  names: string[];
  running: boolean;
  planningAt: number | null;
  onView: (id: string, from: string, abortTime: number, built: boolean) => void;
  onPlan: (formation: number, time: number) => void;
}) {
  return (
    <>
      <h4 className="returns-subhead">From inside a move</h4>
      <table className="table data">
        <thead>
          <tr>
            <th scope="col">In the move to</th>
            <th scope="col" className="num">From</th>
            <th scope="col" className="num">Flight home</th>
            <th scope="col" className="num">Closest</th>
            <th scope="col">Status</th>
            <th scope="col">
              <span className="visually-hidden">Actions</span>
            </th>
          </tr>
        </thead>
        <tbody>
          {points.map((p) => {
            const name = names[p.keyframe_index] ?? p.from_keyframe;
            const planning = planningAt !== null && Math.abs(planningAt - p.abort_time_sec) < 1e-3;
            const tone = planning ? "busy" : STATUS[p.status].tone;
            const from = `the move to ${name} at ${formatTime(p.abort_time_sec)}`;
            return (
              <tr key={p.id}>
                <td>{name}</td>
                <td className="num">{formatTime(p.abort_time_sec)}</td>
                <td className="num">{p.flown_duration_sec != null ? duration(p.flown_duration_sec) : "–"}</td>
                <td className={`num ${p.status === "failed_safety" ? "tone-bad" : ""}`}>
                  {p.worst_separation_m != null ? metres(p.worst_separation_m, 3) : "–"}
                </td>
                <td title={p.message ?? undefined}>
                  <span className={`light light-${tone}`} aria-hidden="true" />
                  {planning ? "Planning" : (ROW_STATUS[p.status] ?? STATUS[p.status].label)}
                </td>
                <td className="row-action">
                  {p.status === "succeeded" && (
                    <button className="link" onClick={() => onView(p.id!, from, p.abort_time_sec, !!p.replay)}>
                      View
                    </button>
                  )}
                  {!running && (
                    <button className="link" onClick={() => onPlan(p.keyframe_index, p.abort_time_sec)}>
                      Plan again
                    </button>
                  )}
                </td>
              </tr>
            );
          })}
        </tbody>
      </table>
    </>
  );
}
