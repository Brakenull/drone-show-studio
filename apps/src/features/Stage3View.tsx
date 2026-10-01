// Stage 3: Monte Carlo stress test and flight-file packing (docs/5-studio_gui.md §6.3), one section of
// the Stage 3 tab at a time (features/Stage3Tab.tsx).

import { useEffect, useMemo, useRef, useState } from "react";
import { openRunFolder, readRunJson, readRunText } from "../bridge/api";
import type {
  JobExit,
  McPair,
  McRecord,
  McReport,
  PackManifest,
  RunRecord,
  RunStatus,
  SimDevice,
} from "../bridge/types";
import { cancelStage3, secondsLeft, startStage3, useStage3Job, type Stage3Job } from "../app/stage3Jobs";
import { isStage2Running } from "../app/stage2Jobs";
import { clock, duration, STATUS } from "../app/format";
import { isStale } from "../app/stages";
import { VerdictCard } from "./VerdictCard";
import { formatTime, metres } from "../replay/sampling";

interface Props {
  run: RunRecord;
  /** Which section of the Stage 3 tab: the stress test or the flight files. */
  part: "stress" | "pack";
  /** From `doctor`; null while it is still checking. */
  devices: SimDevice[] | null;
  onFinished: (runId: string, exit: JobExit) => void;
  onShowInReplay: (time: number, drones: number[], note: string) => void;
}

const D_CRASH_M = 0.5;
const MIN_LANDING_SOC = 0.15;
const REFERENCE_NOTE =
  "Drawn on the planned paths. In the simulated flight the drones drifted from these, so the distance shown here differs.";

const soc = (v: number | null | undefined) =>
  v === null || v === undefined || !Number.isFinite(v) ? "n/a" : `${Math.round(v * 100)} %`;

function bytes(n: number | null | undefined): string {
  if (n === null || n === undefined) return "n/a";
  if (n < 1024) return `${n} B`;
  if (n < 1024 * 1024) return `${(n / 1024).toFixed(1)} KB`;
  return `${(n / 1024 / 1024).toFixed(1)} MB`;
}

/** "Intel(R) Iris(R) Xe Graphics" -> "Intel Iris Xe Graphics". */
const plainName = (name: string) =>
  name
    .replace(/\((R|TM|C)\)/gi, "")
    .replace(/\s+/g, " ")
    .trim();

/** What `auto` picks: the first graphics device, else the first device (the bridge lists GPUs first). */
const autoDevice = (devices: SimDevice[]) => devices.find((d) => d.kind === "gpu") ?? devices[0];

function deviceLabel(id: string, devices: SimDevice[]): string {
  if (id === "auto") {
    const d = autoDevice(devices);
    return d ? `Automatic (${plainName(d.name)})` : "Automatic";
  }
  const d = devices.find((x) => x.id === id);
  return d ? plainName(d.name) : id;
}

/** The device an id stands for, by name ("auto" resolved). */
function deviceName(id: string, devices: SimDevice[]): string {
  const d = id === "auto" ? autoDevice(devices) : devices.find((x) => x.id === id);
  return d ? plainName(d.name) : id;
}

/** A report's device: "Intel(R) Iris(R) Xe Graphics (opencl:0:0)" now, "cpu" or "cuda:0" before OpenCL. */
function reportDevice(device: string): string {
  if (device === "cpu") return "the CPU";
  return plainName(device.replace(/\s*\(opencl:\d+:\d+\)$/, ""));
}

export function Stage3View({ run, part, devices, onFinished, onShowInReplay }: Props) {
  if (run.stage2.status !== "succeeded" || isStage2Running(run.run_id)) {
    return (
      <div className="page">
        <header className="page-head">
          <h1>Stage 3: test and pack</h1>
        </header>
        <p>
          Stage 3 flies the paths Stage 2 planned, so it needs a run whose Stage 2 passed.{" "}
          {run.stage2.status === "not_run" ? "Run Stage 2 first." : "This one hasn't."}
        </p>
      </div>
    );
  }
  return (
    <div className="page">
      <header className="page-head">
        <h1>Stage 3: test and pack</h1>
        <p className="lede">
          Fly the planned show many times in simulated weather to check it holds up, then write the flight file each
          drone loads before the show.
        </p>
      </header>
      {part === "stress" ? (
        <MonteCarlo run={run} devices={devices} onFinished={onFinished} onShowInReplay={onShowInReplay} />
      ) : (
        <Pack run={run} onFinished={onFinished} />
      )}
    </div>
  );
}

// --------------------------------------------------------------------------------------------- //
// Monte Carlo
// --------------------------------------------------------------------------------------------- //

type Tone = "ok" | "warn" | "bad" | "idle";

function recordTone(r: McRecord | undefined): Tone {
  if (!r) return "idle";
  if (!r.passed) return "bad";
  return r.warning_pairs.length || r.brownout_drones.length ? "warn" : "ok";
}

function recordResult(r: McRecord): string {
  if (r.crash_pairs.length) return r.crash_pairs.length === 1 ? "Crash" : `${r.crash_pairs.length} crashes`;
  if (r.low_soc_drones.length) return "Battery too low";
  if (r.warning_pairs.length) return "Close call";
  if (r.brownout_drones.length) return "Voltage dip";
  return "Passed";
}

/** The pair worth looking at in the replay: the closest crash, else the closest near miss. */
const closestPair = (r: McRecord): McPair | undefined => r.crash_pairs[0] ?? r.warning_pairs[0];

const flightName = (r: McRecord) => (r.run < 0 ? "Calm air" : `Flight ${r.run + 1}`);

function MonteCarlo({ run, devices, onFinished, onShowInReplay }: Omit<Props, "part">) {
  const part = run.stage3?.monte_carlo;
  const job = useStage3Job(run.run_id, "monte_carlo");
  const running = !!job && !job.exit;
  const status: RunStatus = running ? "running" : (part?.status ?? "not_run");

  const [runs, setRuns] = useState(part?.config?.runs ?? 100);
  // A run's saved device may be gone (or be "cpu" / "cuda:0" from before OpenCL): fall back to automatic.
  const [chosen, setChosen] = useState(part?.config?.device ?? "auto");
  const device = devices?.some((d) => d.id === chosen) ? chosen : "auto";
  const noDevice = devices !== null && devices.length === 0;
  const [startError, setStartError] = useState<string | null>(null);

  const [report, setReport] = useState<McReport | null>(null);
  useEffect(() => {
    setReport(null);
    if (running || !(status === "succeeded" || status === "failed_safety")) return;
    readRunJson<McReport>(run.run_id, "stage3/monte_carlo_report.json")
      .then(setReport)
      .catch(() => setReport(null));
  }, [run.run_id, status, running, part?.ended_at]);

  const records: McRecord[] = running
    ? job!.records
    : report
      ? [...(report.summary.nominal ? [report.summary.nominal] : []), ...report.runs]
      : [];
  const planned = running ? (job!.planned?.runs ?? runs) : (report?.summary.runs ?? 0);

  function start() {
    setStartError(null);
    const args = ["--runs", String(runs), "--device", device];
    startStage3(run.run_id, run.run_dir, "monte_carlo", args, (exit) => onFinished(run.run_id, exit)).catch((e) =>
      setStartError(String(e)),
    );
  }

  const settings = (
    <div className="mc-settings">
      <label className="field">
        <span className="field-label">Flights</span>
        <input
          type="number"
          min={1}
          max={1000}
          value={runs}
          onChange={(e) => setRuns(Math.max(1, Math.min(1000, Number(e.target.value) || 1)))}
        />
      </label>
      <label className="field field-device">
        <span className="field-label">Simulate on</span>
        <select value={device} disabled={!devices?.length} onChange={(e) => setChosen(e.target.value)}>
          {devices === null ? (
            <option value="auto">Checking…</option>
          ) : noDevice ? (
            <option value="auto">No device found</option>
          ) : (
            ["auto", ...devices.map((d) => d.id)].map((id) => (
              <option key={id} value={id}>
                {deviceLabel(id, devices)}
              </option>
            ))
          )}
        </select>
      </label>
      <button className={status === "not_run" ? "primary" : ""} disabled={noDevice} onClick={start}>
        {status === "not_run" ? "Start stress test" : "Run stress test again"}
      </button>
    </div>
  );

  return (
    <section className="card" aria-labelledby="mc-title">
      <h2 id="mc-title" className="stage3-title">
        Stress test in simulated weather
      </h2>
      <p className="muted stage3-about">
        Each flight gets its own wind, gusts, temperature, GPS drift and small differences between drones. The show
        passes when no two drones come within {metres(D_CRASH_M, 1)} and every drone lands with at least{" "}
        {soc(MIN_LANDING_SOC)} battery.
      </p>

      {startError && <p className="notice notice-bad">Could not start the stress test: {startError}</p>}
      {!running && noDevice && (
        <p className="notice notice-bad">
          No device to simulate on. The stress test runs on the graphics chip through its OpenCL driver, or on the
          processor with a CPU OpenCL runtime. Install either one, then use Check again on the Settings page.
        </p>
      )}
      {!running && isStale(run, part) && (
        <p className="notice notice-warn">
          These results are from an earlier Stage 2 result. Run the stress test again to test the current paths.
        </p>
      )}

      {running ? <McRunning job={job!} runId={run.run_id} devices={devices ?? []} /> : settings}

      {(running || report) && (
        <McResults
          records={records}
          planned={planned}
          report={running ? null : report}
          onShowInReplay={onShowInReplay}
        />
      )}
      {!running && (status === "failed_error" || status === "failed_input" || status === "cancelled") && (
        <Stopped runId={run.run_id} part="monte_carlo" status={status} message={part?.message} job={job} />
      )}
    </section>
  );
}

function McRunning({ job, runId, devices }: { job: Stage3Job; runId: string; devices: SimDevice[] }) {
  const [now, setNow] = useState(Date.now());
  const [cancelError, setCancelError] = useState<string | null>(null);
  useEffect(() => {
    const id = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(id);
  }, []);
  const done = job.records.filter((r) => r.run >= 0).length;
  const total = job.planned?.runs ?? 0;
  const left = secondsLeft(job, now);
  const title = !job.records.length
    ? "Flying the show in calm air first"
    : `Flown ${done} of ${total} flight${total === 1 ? "" : "s"}`;
  return (
    <div className="running-head mc-running">
      <span className="light light-busy" aria-hidden="true" />
      <div>
        <p className="running-phase">{title}</p>
        <p className="muted">
          Running for <span className="num">{clock(now - job.startedAt)}</span>
          {left !== null && left > 0 && <>, about {duration(left)} left</>}
          {job.planned && <> on {deviceName(job.planned.device, devices)}</>}
        </p>
      </div>
      <button
        className="danger"
        disabled={job.cancelling || job.jobId < 0}
        onClick={() => cancelStage3(runId, "monte_carlo").catch((e) => setCancelError(String(e)))}
      >
        {job.cancelling ? "Cancelling…" : "Cancel stress test"}
      </button>
      {cancelError && <p className="notice notice-bad">{cancelError}</p>}
    </div>
  );
}

function McResults({
  records,
  planned,
  report,
  onShowInReplay,
}: {
  records: McRecord[];
  planned: number;
  report: McReport | null;
  onShowInReplay: Props["onShowInReplay"];
}) {
  const [selected, setSelected] = useState<number | null>(null);
  const rowRefs = useRef(new Map<number, HTMLTableRowElement>());
  const byRun = useMemo(() => new Map(records.map((r) => [r.run, r])), [records]);
  const nominal = byRun.get(-1);

  const select = (runIndex: number) => {
    setSelected(runIndex);
    rowRefs.current.get(runIndex)?.scrollIntoView({ block: "nearest", behavior: "smooth" });
  };

  const cells = Array.from({ length: planned }, (_, i) => i);
  const s = report?.summary;
  const crashes = s?.crash_runs.length ?? 0;
  const lowBattery = s?.low_soc_runs.length ?? 0;

  return (
    <>
      {s && (
        <VerdictCard
          inner
          tone={s.passed ? "ok" : "bad"}
          title={
            s.passed
              ? `The show held up in all ${s.runs} flights`
              : crashes
                ? `${crashes} of ${s.runs} flights had a crash`
                : `${lowBattery} of ${s.runs} flights landed with too little battery`
          }
        >
          <dl className="stat-cards">
            <div>
              <dt>Closest approach</dt>
              <dd className={(s.worst_min_separation_m ?? Infinity) < D_CRASH_M ? "tone-bad" : undefined}>
                {metres(s.worst_min_separation_m, 2)}
              </dd>
            </div>
            <div>
              <dt>Lowest battery at landing</dt>
              <dd className={(s.worst_final_soc ?? 1) < MIN_LANDING_SOC ? "tone-bad" : undefined}>
                {soc(s.worst_final_soc)}
              </dd>
            </div>
            <div>
              <dt>Close calls (under {metres(1, 1)})</dt>
              <dd className={s.warning_runs.length ? "tone-warn" : undefined}>
                {s.warning_runs.length} flight{s.warning_runs.length === 1 ? "" : "s"}
              </dd>
            </div>
            <div>
              <dt>Voltage dips</dt>
              <dd className={s.brownout_runs.length ? "tone-warn" : undefined}>
                {s.brownout_runs.length} flight{s.brownout_runs.length === 1 ? "" : "s"}
              </dd>
            </div>
          </dl>
          <p className="muted small">
            Tested on {reportDevice(report!.device)} in {duration(report!.wall_time_sec)}.
            {(s.warning_runs.length > 0 || s.brownout_runs.length > 0) &&
              s.passed &&
              " Close calls and voltage dips don't fail the test, but they are worth a look in the table below."}
          </p>
        </VerdictCard>
      )}

      <div className="run-grid" role="list" aria-label="Flights">
        <button
          role="listitem"
          className={`run-cell run-cell-nominal tone-cell-${recordTone(nominal)} ${selected === -1 ? "is-selected" : ""}`}
          title={nominal ? `Calm air: ${recordResult(nominal)}, closest ${metres(nominal.min_separation_m, 2)}` : "Calm air: waiting"}
          aria-label={nominal ? `Calm air, ${recordResult(nominal)}` : "Calm air, waiting"}
          disabled={!nominal}
          onClick={() => select(-1)}
        />
        {cells.map((i) => {
          const r = byRun.get(i);
          return (
            <button
              key={i}
              role="listitem"
              className={`run-cell tone-cell-${recordTone(r)} ${selected === i ? "is-selected" : ""}`}
              title={r ? `Flight ${i + 1}: ${recordResult(r)}, closest ${metres(r.min_separation_m, 2)}` : `Flight ${i + 1}: waiting`}
              aria-label={r ? `Flight ${i + 1}, ${recordResult(r)}` : `Flight ${i + 1}, waiting`}
              disabled={!r}
              onClick={() => select(i)}
            />
          );
        })}
      </div>
      <p className="run-grid-key muted small">
        <span className="run-cell run-cell-key tone-cell-ok" aria-hidden="true" /> passed
        <span className="run-cell run-cell-key tone-cell-warn" aria-hidden="true" /> passed with a close call or
        voltage dip
        <span className="run-cell run-cell-key tone-cell-bad" aria-hidden="true" /> failed
        <span className="run-cell run-cell-key run-cell-nominal tone-cell-idle" aria-hidden="true" /> the first
        light is the show in calm air
      </p>

      {records.length > 0 && (
        <div className="table-scroll">
          <table className="table data mc-table">
            <thead>
              <tr>
                <th scope="col">Flight</th>
                <th scope="col">Result</th>
                <th scope="col" className="num">Closest</th>
                <th scope="col" className="num">Lowest battery</th>
                <th scope="col" className="num">Wind</th>
                <th scope="col" className="num">Gust</th>
                <th scope="col" className="num">Sim speed</th>
                <th scope="col">
                  <span className="visually-hidden">Actions</span>
                </th>
              </tr>
            </thead>
            <tbody>
              {records.map((r) => {
                const tone = recordTone(r);
                const pair = closestPair(r);
                return (
                  <tr
                    key={r.run}
                    ref={(el) => {
                      if (el) rowRefs.current.set(r.run, el);
                      else rowRefs.current.delete(r.run);
                    }}
                    className={selected === r.run ? "is-selected" : undefined}
                    onClick={() => setSelected(r.run)}
                  >
                    <td>{flightName(r)}</td>
                    <td className={tone === "ok" ? undefined : `tone-${tone}`}>{recordResult(r)}</td>
                    <td className={`num ${(r.min_separation_m ?? Infinity) < D_CRASH_M ? "tone-bad" : ""}`}>
                      {metres(r.min_separation_m, 2)}
                    </td>
                    <td className={`num ${r.min_final_soc < MIN_LANDING_SOC ? "tone-bad" : ""}`}>
                      {soc(r.min_final_soc)}
                    </td>
                    <td className="num">{r.scenario.mean_wind_mps.toFixed(1)} m/s</td>
                    <td className="num">{r.scenario.gust_peak_mps.toFixed(1)} m/s</td>
                    <td className="num">{r.realtime_factor.toFixed(2)}×</td>
                    <td className="row-action">
                      {pair && (
                        <button
                          className="link"
                          onClick={(e) => {
                            e.stopPropagation();
                            onShowInReplay(pair.time_sec, [pair.drone_a, pair.drone_b], REFERENCE_NOTE);
                          }}
                          title={`Drones ${pair.drone_a} and ${pair.drone_b}, ${metres(pair.min_distance_m, 2)} at ${formatTime(pair.time_sec)}`}
                        >
                          Show {pair.drone_a} and {pair.drone_b} in replay
                        </button>
                      )}
                    </td>
                  </tr>
                );
              })}
            </tbody>
          </table>
        </div>
      )}
      {records.length > 0 && (
        <p className="muted small">
          Sim speed is how much faster than real time the simulation ran; below 1× a flight takes longer than the
          show. Flights simulated side by side share their batch's average speed.
        </p>
      )}
    </>
  );
}

// --------------------------------------------------------------------------------------------- //
// Pack
// --------------------------------------------------------------------------------------------- //

const PACK_PHASE: Record<string, string> = {
  starting: "Starting",
  packing: "Writing one flight file per drone",
  verifying: "Reading every file back and checking it",
};

function Pack({ run, onFinished }: { run: RunRecord; onFinished: Props["onFinished"] }) {
  const part = run.stage3?.pack;
  const job = useStage3Job(run.run_id, "pack");
  const running = !!job && !job.exit;
  const status: RunStatus = running ? "running" : (part?.status ?? "not_run");
  const [startError, setStartError] = useState<string | null>(null);
  const [cancelError, setCancelError] = useState<string | null>(null);
  const [manifest, setManifest] = useState<PackManifest | null>(null);

  useEffect(() => {
    setManifest(null);
    if (running || status !== "succeeded") return;
    readRunJson<PackManifest>(run.run_id, "stage3/bin/manifest.json")
      .then(setManifest)
      .catch(() => setManifest(null));
  }, [run.run_id, status, running, part?.ended_at]);

  function start() {
    setStartError(null);
    startStage3(run.run_id, run.run_dir, "pack", [], (exit) => onFinished(run.run_id, exit)).catch((e) =>
      setStartError(String(e)),
    );
  }

  const verified = manifest?.files.filter((f) => f.verified).length ?? 0;
  return (
    <section className="card" aria-labelledby="pack-title">
      <h2 id="pack-title" className="stage3-title">
        Flight files
      </h2>
      <p className="muted stage3-about">
        One file per drone with its position and LED colour every 50 ms, in the format the drone's flight controller
        reads. Every file is read back and checked after it is written.
      </p>
      {startError && <p className="notice notice-bad">Could not start packing: {startError}</p>}
      {!running && isStale(run, part) && (
        <p className="notice notice-warn">
          These files are from an earlier Stage 2 result. Pack again before loading them onto drones.
        </p>
      )}

      {running ? (
        <div className="running-head mc-running">
          <span className="light light-busy" aria-hidden="true" />
          <div>
            <p className="running-phase">{PACK_PHASE[job!.phase] ?? job!.phase}</p>
            <p className="muted">{run.input.fleet_size} drones</p>
          </div>
          <button
            className="danger"
            disabled={job!.cancelling || job!.jobId < 0}
            onClick={() => cancelStage3(run.run_id, "pack").catch((e) => setCancelError(String(e)))}
          >
            {job!.cancelling ? "Cancelling…" : "Cancel"}
          </button>
          {cancelError && <p className="notice notice-bad">{cancelError}</p>}
        </div>
      ) : (
        <div className="actions">
          <button className={status === "not_run" ? "primary" : ""} onClick={start}>
            {status === "not_run" ? "Pack flight files" : "Pack flight files again"}
          </button>
          {status === "succeeded" && (
            <button onClick={() => openRunFolder(run.run_id, "stage3/bin")}>Open the files' folder</button>
          )}
        </div>
      )}

      {!running && status === "succeeded" && part && (
        <>
          <VerdictCard inner tone="ok" title={`${part.verified_files} of ${part.files} files written and checked`}>
            <dl className="stat-cards">
              <div>
                <dt>Samples per file</dt>
                <dd>{part.records_per_file?.toLocaleString()}</dd>
              </div>
              <div>
                <dt>Every</dt>
                <dd>{part.sampling_dt_ms} ms</dd>
              </div>
              <div>
                <dt>File size</dt>
                <dd>{bytes(part.file_size_bytes)}</dd>
              </div>
              <div>
                <dt>All files</dt>
                <dd>{bytes(part.total_bytes)}</dd>
              </div>
            </dl>
          </VerdictCard>
          {manifest && (
            <details className="manifest">
              <summary>
                File list ({manifest.files.length} files, {verified} checked)
              </summary>
              <div className="table-scroll">
                <table className="table data">
                  <thead>
                    <tr>
                      <th scope="col">Drone</th>
                      <th scope="col">File</th>
                      <th scope="col" className="num">Samples</th>
                      <th scope="col" className="num">Size</th>
                      <th scope="col">CRC-32</th>
                      <th scope="col">Checked</th>
                    </tr>
                  </thead>
                  <tbody>
                    {manifest.files.map((f) => (
                      <tr key={f.drone_id}>
                        <td>{f.drone_id}</td>
                        <td className="path">{f.file}</td>
                        <td className="num">{manifest.records_per_file.toLocaleString()}</td>
                        <td className="num">{f.size_bytes.toLocaleString()} B</td>
                        <td className="path">{f.crc32}</td>
                        <td className={f.verified ? "tone-ok" : "tone-bad"}>{f.verified ? "Yes" : "No"}</td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            </details>
          )}
        </>
      )}
      {!running && (status === "failed_error" || status === "failed_input" || status === "cancelled") && (
        <Stopped runId={run.run_id} part="pack" status={status} message={part?.message} job={job} />
      )}
    </section>
  );
}

// --------------------------------------------------------------------------------------------- //

function Stopped({
  runId,
  part,
  status,
  message,
  job,
}: {
  runId: string;
  part: "monte_carlo" | "pack";
  status: RunStatus;
  message?: string | null;
  job: Stage3Job | null;
}) {
  const [lines, setLines] = useState<string[]>([]);
  useEffect(() => {
    if (job?.log.length) {
      setLines(job.log);
      return;
    }
    readRunText(runId, `stage3/${part}_log.ndjson`)
      .then((text) =>
        setLines(
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
      .catch(() => setLines([]));
  }, [runId, part, job]);

  const cancelled = status === "cancelled";
  return (
    <div className="stage3-stopped">
      <p className={`notice ${cancelled ? "" : "notice-bad"}`}>
        <span className={`light light-${STATUS[status].tone}`} aria-hidden="true" />{" "}
        {cancelled ? "Cancelled before it finished." : (message ?? "Stopped with an error.")}
      </p>
      {lines.length > 0 && (
        <details className="log">
          <summary>Output ({lines.length} lines)</summary>
          <pre>{lines.slice(-200).join("\n")}</pre>
        </details>
      )}
    </div>
  );
}
