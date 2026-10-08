// "Return paths" on a passed Stage 2 run: plan, after the show
// passed, a checked flight from each formation straight back to the holding area.

import { useEffect, useState } from "react";
import { Alert, Button, Modal, Table, type TableColumnsType } from "antd";
import { readRunJson, readRunText, runJob } from "../bridge/api";
import type { JobExit, ReturnEntry, ReturnIndex, RunRecord } from "../bridge/types";
import { cancelReturns, startReturns, useReturnsJob, type ReturnsJob } from "../app/returnsJobs";
import { solveFraction } from "../app/stage2Jobs";
import { clock, duration, STATUS } from "../app/format";
import { formatTime, metres } from "../replay/sampling";
import { JobProgress } from "./JobOutput";

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

const dash = "–";

/** A status light and its words. */
const Lit = ({ tone, children }: { tone: string; children: React.ReactNode }) => (
  <>
    <span className={`light light-${tone}`} aria-hidden="true" />
    {children}
  </>
);

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

  /** One formation's row: the planned return from it, or the show's own return leg for the last one. */
  function row(k: number) {
    const entry = byFormation.get(k);
    const planning = running && job!.current?.keyframe === k;
    const reversed = entry?.method === "reversed_takeoff" && entry.status === "succeeded";
    const viewable = entry?.status === "succeeded" ? (built ? built.has(k) : null) : undefined;
    return { entry, planning, reversed, viewable, leg: coveredByLeg(k) };
  }

  const formationColumns: TableColumnsType<{ name: string; k: number }> = [
    { title: "From", dataIndex: "name" },
    {
      title: "Reached at",
      key: "at",
      align: "right",
      className: "num",
      render: (_, { k }) => {
        const { entry, leg } = row(k);
        return !leg && entry ? formatTime(entry.abort_time_sec) : dash;
      },
    },
    {
      title: "Flight home",
      key: "home",
      align: "right",
      className: "num",
      render: (_, { k }) => {
        const { entry, leg } = row(k);
        return !leg && entry?.flown_duration_sec != null ? duration(entry.flown_duration_sec) : dash;
      },
    },
    {
      title: "Closest",
      key: "closest",
      align: "right",
      onCell: ({ k }) => {
        const { entry, leg } = row(k);
        return {
          className: `num ${!leg && entry?.status === "failed_safety" ? "tone-bad" : ""}`,
          title: !leg && entry?.required_separation_m ? `needs ${metres(entry.required_separation_m, 2)}` : undefined,
        };
      },
      render: (_, { k }) => {
        const { entry, leg } = row(k);
        return !leg && entry?.worst_separation_m != null ? metres(entry.worst_separation_m, 3) : dash;
      },
    },
    {
      title: "Status",
      key: "status",
      onCell: ({ k }) => {
        const { entry, leg } = row(k);
        return leg ? { className: "muted" } : { title: entry?.message ?? undefined };
      },
      render: (_, { k }) => {
        const { entry, planning, reversed, leg } = row(k);
        if (leg) return "The show's own return leg";
        const tone = planning ? "busy" : entry ? STATUS[entry.status].tone : "idle";
        return (
          <Lit tone={tone}>
            {planning
              ? "Planning"
              : reversed
                ? "Takeoff flown backwards"
                : entry
                  ? (ROW_STATUS[entry.status] ?? STATUS[entry.status].label)
                  : "Not planned"}
          </Lit>
        );
      },
    },
    {
      title: "3D view",
      key: "view",
      onCell: ({ k }) => (row(k).leg ? { className: "muted" } : {}),
      render: (_, { k }) => {
        const { viewable, leg } = row(k);
        if (leg) return "In the show";
        if (viewable === undefined) return dash;
        if (viewable === null) return <span className="muted">Checking</span>;
        return <Lit tone={viewable ? "ok" : "idle"}>{viewable ? "Ready" : "Not built"}</Lit>;
      },
    },
    {
      title: <span className="visually-hidden">Actions</span>,
      key: "actions",
      align: "right",
      className: "row-action",
      render: (_, { name, k }) => {
        const { entry, reversed, viewable, leg } = row(k);
        if (leg) return null;
        return (
          <>
            {entry?.status === "succeeded" && (
              <Button
                type="link"
                size="small"
                onClick={() =>
                  viewable
                    ? onView(k, name, entry.abort_time_sec)
                    : setToBuild({ keyframe: k, from: name, abortTime: entry.abort_time_sec })
                }
              >
                View
              </Button>
            )}
            {!running && !reversed && (
              <Button type="link" size="small" onClick={() => plan([k])}>
                {entry ? "Plan again" : "Plan"}
              </Button>
            )}
          </>
        );
      },
    },
  ];

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

      <Table<{ name: string; k: number }>
        size="small"
        pagination={false}
        rowKey="k"
        dataSource={names.map((name, k) => ({ name, k }))}
        columns={formationColumns}
      />

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

      {startError && <Alert type="error" showIcon title={`Could not start planning: ${startError}`} />}
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
              <Button type="primary" size="large" onClick={() => plan(missing)}>
                Plan return paths
              </Button>
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
  const [building, setBuilding] = useState(false);
  const [error, setError] = useState<string | null>(null);

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
    <Modal
      open
      centered
      title="No 3D view yet"
      // While building, the dialog stays open until the build ends.
      closable={!building}
      keyboard={!building}
      mask={{ closable: false }}
      onCancel={onClose}
      footer={[
        <Button key="later" onClick={onClose} disabled={building}>
          Not now
        </Button>,
        // The expected action is building, so it takes the focus.
        <Button key="build" type="primary" autoFocus onClick={build} loading={building}>
          {building ? "Building…" : "Build and view"}
        </Button>,
      ]}
    >
      <p className="muted">
        The flight home from {target.from} hasn't been prepared for the 3D view. Building it samples the show up to{" "}
        {target.from} and the flight home; for {fleet} drones that takes{" "}
        {fleet >= 150 ? "about 10 to 20 seconds" : "a few seconds"}.
      </p>
      {error && <Alert type="error" showIcon title={`Could not build the 3D view: ${error}`} />}
    </Modal>
  );
}

function Finished({ job }: { job: ReturnsJob }) {
  if (job.exit?.cancelled)
    return <Alert type="info" showIcon title="Planning was cancelled. Finished return paths are kept." />;
  const failed = job.results.filter((r) => r.status !== "succeeded");
  if (job.errors.length && !job.results.length)
    return <Alert type="error" showIcon title={job.errors[job.errors.length - 1]} />;
  if (!failed.length) return null;
  return (
    <Alert
      type="warning"
      showIcon
      title={`${failed.length === 1 ? "One formation has" : `${failed.length} formations have`} no return path: the safety check rejected ${failed.length === 1 ? "its flight" : "their flights"} home. Hover its status for the details.`}
    />
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
        <Button
          danger
          size="large"
          loading={job.cancelling}
          disabled={job.jobId < 0}
          onClick={() => cancelReturns(runId).catch((e) => setCancelError(String(e)))}
        >
          {job.cancelling ? "Cancelling…" : "Cancel planning"}
        </Button>
      </div>
      <JobProgress pct={pct} />
      <p className="muted small">Finished return paths are saved as they complete, so cancelling keeps them.</p>
      {cancelError && <Alert type="error" showIcon title={cancelError} />}
    </div>
  );
}

/** Returns planned from moments inside a move, from the readiness
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
  const name = (p: ReturnEntry) => names[p.keyframe_index] ?? p.from_keyframe;
  const planning = (p: ReturnEntry) => planningAt !== null && Math.abs(planningAt - p.abort_time_sec) < 1e-3;
  return (
    <>
      <h4 className="returns-subhead">From inside a move</h4>
      <Table<ReturnEntry>
        size="small"
        pagination={false}
        rowKey={(p) => p.id!}
        dataSource={points}
        columns={[
          { title: "In the move to", key: "to", render: (_, p) => name(p) },
          { title: "From", key: "from", align: "right", className: "num", render: (_, p) => formatTime(p.abort_time_sec) },
          {
            title: "Flight home",
            key: "home",
            align: "right",
            className: "num",
            render: (_, p) => (p.flown_duration_sec != null ? duration(p.flown_duration_sec) : dash),
          },
          {
            title: "Closest",
            key: "closest",
            align: "right",
            onCell: (p) => ({ className: `num ${p.status === "failed_safety" ? "tone-bad" : ""}` }),
            render: (_, p) => (p.worst_separation_m != null ? metres(p.worst_separation_m, 3) : dash),
          },
          {
            title: "Status",
            key: "status",
            onCell: (p) => ({ title: p.message ?? undefined }),
            render: (_, p) => (
              <Lit tone={planning(p) ? "busy" : STATUS[p.status].tone}>
                {planning(p) ? "Planning" : (ROW_STATUS[p.status] ?? STATUS[p.status].label)}
              </Lit>
            ),
          },
          {
            title: <span className="visually-hidden">Actions</span>,
            key: "actions",
            align: "right",
            className: "row-action",
            render: (_, p) => (
              <>
                {p.status === "succeeded" && (
                  <Button
                    type="link"
                    size="small"
                    onClick={() =>
                      onView(p.id!, `the move to ${name(p)} at ${formatTime(p.abort_time_sec)}`, p.abort_time_sec, !!p.replay)
                    }
                  >
                    View
                  </Button>
                )}
                {!running && (
                  <Button type="link" size="small" onClick={() => onPlan(p.keyframe_index, p.abort_time_sec)}>
                    Plan again
                  </Button>
                )}
              </>
            ),
          },
        ]}
      />
    </>
  );
}
