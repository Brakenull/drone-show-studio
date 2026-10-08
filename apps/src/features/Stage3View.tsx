// Stage 3: Monte Carlo stress test and flight-file packing, one section of
// the Stage 3 tab at a time (features/Stage3Tab.tsx).

import { useEffect, useMemo, useRef, useState } from "react";
import {
  Alert,
  Button,
  Collapse,
  Input,
  InputNumber,
  Segmented,
  Select,
  Table,
} from "antd";
import { openRunFolder, readRunJson, readRunText } from "../bridge/api";
import type {
  JobExit,
  McPair,
  McRecord,
  McReport,
  McWeather,
  PackManifest,
  PackPart,
  RunRecord,
  RunStatus,
  SimDevice,
} from "../bridge/types";
import {
  cancelStage3,
  secondsLeft,
  startStage3,
  useStage3Job,
  type Stage3Job,
} from "../app/stage3Jobs";
import { isStage2Running } from "../app/stage2Jobs";
import { clock, duration } from "../app/format";
import { isStale } from "../app/stages";
import { VerdictCard } from "./VerdictCard";
import { LogPane } from "./JobOutput";
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

/** A long table: scrolls inside a frame with its header kept in view. */
const SCROLL = { x: "max-content", y: 384 };
const FRAME = { maxWidth: "56rem" };

const soc = (v: number | null | undefined) =>
  v === null || v === undefined || !Number.isFinite(v)
    ? "n/a"
    : `${Math.round(v * 100)} %`;

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
const autoDevice = (devices: SimDevice[]) =>
  devices.find((d) => d.kind === "gpu") ?? devices[0];

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
  const d =
    id === "auto" ? autoDevice(devices) : devices.find((x) => x.id === id);
  return d ? plainName(d.name) : id;
}

/** A report's device: "Intel(R) Iris(R) Xe Graphics (opencl:0:0)" now, "cpu" or "cuda:0" before OpenCL. */
function reportDevice(device: string): string {
  if (device === "cpu") return "the CPU";
  return plainName(device.replace(/\s*\(opencl:\d+:\d+\)$/, ""));
}

export function Stage3View({
  run,
  part,
  devices,
  onFinished,
  onShowInReplay,
}: Props) {
  if (run.stage2.status !== "succeeded" || isStage2Running(run.run_id)) {
    return (
      <div className="page">
        <header className="page-head">
          <h1>Stage 3: test and pack</h1>
        </header>
        <p>
          Stage 3 flies the paths Stage 2 planned, so it needs a run whose Stage
          2 passed.{" "}
          {run.stage2.status === "not_run"
            ? "Run Stage 2 first."
            : "This one hasn't."}
        </p>
      </div>
    );
  }
  return (
    <div className="page">
      <header className="page-head">
        <h1>Stage 3: test and pack</h1>
        <p className="lede">
          Fly the planned show many times in simulated weather to check it holds
          up, then write the flight file each drone loads before the show.
        </p>
      </header>
      {part === "stress" ? (
        <MonteCarlo
          run={run}
          devices={devices}
          onFinished={onFinished}
          onShowInReplay={onShowInReplay}
        />
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
  if (r.crash_pairs.length)
    return r.crash_pairs.length === 1
      ? "Crash"
      : `${r.crash_pairs.length} crashes`;
  if (r.low_soc_drones.length) return "Battery too low";
  if (r.warning_pairs.length) return "Close call";
  if (r.brownout_drones.length) return "Voltage dip";
  return "Passed";
}

/** The pair worth looking at in the replay: the closest crash, else the closest near miss. */
const closestPair = (r: McRecord): McPair | undefined =>
  r.crash_pairs[0] ?? r.warning_pairs[0];

const flightName = (r: McRecord) =>
  r.run < 0 ? "Calm air" : `Flight ${r.run + 1}`;

function MonteCarlo({
  run,
  devices,
  onFinished,
  onShowInReplay,
}: Omit<Props, "part">) {
  const part = run.stage3?.monte_carlo;
  const job = useStage3Job(run.run_id, "monte_carlo");
  const running = !!job && !job.exit;
  const status: RunStatus = running ? "running" : (part?.status ?? "not_run");

  const [runs, setRuns] = useState(part?.config?.runs ?? 100);
  // The forecast is always the default; random weather is the fallback (offline, or too far ahead).
  const [weather, setWeather] = useState<McWeather>("forecast");
  const [showStart, setShowStart] = useState(part?.config?.show_start ?? "");
  const needsStart = weather === "forecast" && !showStart;
  // A run's saved device may be gone (or be "cpu" / "cuda:0" from before OpenCL): fall back to automatic.
  const [chosen, setChosen] = useState(part?.config?.device ?? "auto");
  const device = devices?.some((d) => d.id === chosen) ? chosen : "auto";
  const noDevice = devices !== null && devices.length === 0;
  const [startError, setStartError] = useState<string | null>(null);

  const [report, setReport] = useState<McReport | null>(null);
  useEffect(() => {
    setReport(null);
    if (running || !(status === "succeeded" || status === "failed_safety"))
      return;
    readRunJson<McReport>(run.run_id, "stage3/monte_carlo_report.json")
      .then(setReport)
      .catch(() => setReport(null));
  }, [run.run_id, status, running, part?.ended_at]);

  const records: McRecord[] = running
    ? job!.records
    : report
      ? [
          ...(report.summary.nominal ? [report.summary.nominal] : []),
          ...report.runs,
        ]
      : [];
  const planned = running
    ? (job!.planned?.runs ?? runs)
    : (report?.summary.runs ?? 0);

  function start() {
    setStartError(null);
    const args = [
      "--runs",
      String(runs),
      "--device",
      device,
      "--weather",
      weather,
    ];
    if (weather === "forecast") args.push("--show-start", showStart);
    startStage3(run.run_id, run.run_dir, "monte_carlo", args, (exit) =>
      onFinished(run.run_id, exit),
    ).catch((e) => setStartError(String(e)));
  }

  const settings = (
    <>
      <div className="mc-settings">
        <div className="field">
          <span className="field-label" id="mc-weather-label">
            Weather
          </span>
          <Segmented<McWeather>
            aria-labelledby="mc-weather-label"
            value={weather}
            onChange={setWeather}
            options={[
              { value: "forecast", label: "Forecast" },
              { value: "random", label: "Random" },
            ]}
          />
        </div>
        {weather === "forecast" && (
          <label className="field">
            <span className="field-label">Show starts (local time)</span>
            <Input
              type="datetime-local"
              className="mc-show-start"
              value={showStart}
              status={needsStart ? "warning" : undefined}
              onChange={(e) => setShowStart(e.target.value)}
            />
          </label>
        )}
        <label className="field">
          <span className="field-label">Flights</span>
          <InputNumber<number>
            className="mc-flights"
            min={1}
            max={1000}
            precision={0}
            value={runs}
            onChange={(v) => v !== null && setRuns(v)}
          />
        </label>
        <label className="field">
          <span className="field-label">Simulate on</span>
          <Select
            value={device}
            disabled={!devices?.length}
            loading={devices === null}
            popupMatchSelectWidth={false}
            onChange={setChosen}
            options={
              devices === null
                ? [{ value: "auto", label: "Checking…" }]
                : noDevice
                  ? [{ value: "auto", label: "No device found" }]
                  : ["auto", ...devices.map((d) => d.id)].map((id) => ({
                      value: id,
                      label: deviceLabel(id, devices),
                    }))
            }
          />
        </label>
        <Button
          size="large"
          type={status === "not_run" ? "primary" : "default"}
          disabled={noDevice || needsStart}
          title={needsStart ? "Set when the show starts" : undefined}
          onClick={start}
        >
          {status === "not_run" ? "Start stress test" : "Run stress test again"}
        </Button>
      </div>
      <p
        className={`small mc-weather-hint ${weather === "random" ? "tone-warn" : "muted"}`}
      >
        {weather === "random"
          ? "Random weather, not the forecast: each flight draws wind up to 8 m/s and a gust up to 5 m/s. Use it when there is no forecast (offline, or the show is more than about two weeks away)."
          : "The Open-Meteo ensemble forecast for the project's GPS origin: each flight flies one of its forecasts. Available up to about two weeks ahead."}
      </p>
    </>
  );

  return (
    <section className="card" aria-labelledby="mc-title">
      <h2 id="mc-title" className="stage3-title">
        Stress test in simulated weather
      </h2>
      <p className="muted stage3-about">
        Each flight gets its own wind, gusts, temperature, GPS drift and small
        differences between drones. The show passes when no two drones come
        within {metres(D_CRASH_M, 1)} and every drone lands with at least{" "}
        {soc(MIN_LANDING_SOC)} battery.
      </p>

      {startError && (
        <Alert
          type="error"
          showIcon
          title={`Could not start the stress test: ${startError}`}
        />
      )}
      {!running && noDevice && (
        <Alert
          type="error"
          showIcon
          title="No device to simulate on."
          description="The stress test runs on the graphics chip through its OpenCL driver, or on the processor with a CPU OpenCL runtime. Install either one, then use Check again on the Settings page."
        />
      )}
      {!running && isStale(run, part) && (
        <Alert
          type="warning"
          showIcon
          title="These results are from an earlier Stage 2 result. Run the stress test again to test the current paths."
        />
      )}

      {running ? (
        <McRunning job={job!} runId={run.run_id} devices={devices ?? []} />
      ) : (
        settings
      )}

      {(running || report) && (
        <McResults
          records={records}
          planned={planned}
          report={running ? null : report}
          onShowInReplay={onShowInReplay}
        />
      )}
      {!running &&
        (status === "failed_error" ||
          status === "failed_input" ||
          status === "cancelled") && (
          <Stopped
            runId={run.run_id}
            part="monte_carlo"
            status={status}
            message={part?.message}
            job={job}
          />
        )}
      {!running &&
        status === "failed_input" &&
        part?.config?.weather === "forecast" && (
          <p className="muted small">
            If the forecast can't be fetched, switch Weather to Random to test
            without it.
          </p>
        )}
    </section>
  );
}

function McRunning({
  job,
  runId,
  devices,
}: {
  job: Stage3Job;
  runId: string;
  devices: SimDevice[];
}) {
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
        {job.planned?.weather === "random" && (
          <p className="small tone-warn">
            Random weather, not the forecast for the show.
          </p>
        )}
      </div>
      <Button
        danger
        size="large"
        loading={job.cancelling}
        disabled={job.jobId < 0}
        onClick={() =>
          cancelStage3(runId, "monte_carlo").catch((e) =>
            setCancelError(String(e)),
          )
        }
      >
        {job.cancelling ? "Cancelling…" : "Cancel stress test"}
      </Button>
      {cancelError && <Alert type="error" showIcon title={cancelError} />}
    </div>
  );
}

/** "2026-10-11T19:30" -> "2026-10-11 19:30". */
const localTime = (iso: string) => iso.replace("T", " ").slice(0, 16);

function WeatherSource({ report }: { report: McReport }) {
  const w = report.weather_source;
  if (!w)
    return (
      <p className="small tone-warn">
        Weather: random, not the forecast for the show.
      </p>
    );
  return (
    <p className="muted small">
      Weather: Open-Meteo forecast for {localTime(w.show_start)}
      {w.site && (
        <>
          {" "}
          at {w.site[0].toFixed(4)}, {w.site[1].toFixed(4)}
        </>
      )}{" "}
      ({w.model}, {w.members} forecasts
      {w.fetched_at && <>, fetched {localTime(w.fetched_at)}</>}).
    </p>
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
  const tableRef = useRef<HTMLDivElement>(null);
  const byRun = useMemo(
    () => new Map(records.map((r) => [r.run, r])),
    [records],
  );
  const nominal = byRun.get(-1);

  const select = (runIndex: number) => {
    setSelected(runIndex);
    tableRef.current
      ?.querySelector(`[data-row-key="${runIndex}"]`)
      ?.scrollIntoView({ block: "nearest", behavior: "smooth" });
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
              <dd
                className={
                  (s.worst_min_separation_m ?? Infinity) < D_CRASH_M
                    ? "tone-bad"
                    : undefined
                }
              >
                {metres(s.worst_min_separation_m, 2)}
              </dd>
            </div>
            <div>
              <dt>Lowest battery at landing</dt>
              <dd
                className={
                  (s.worst_final_soc ?? 1) < MIN_LANDING_SOC
                    ? "tone-bad"
                    : undefined
                }
              >
                {soc(s.worst_final_soc)}
              </dd>
            </div>
            <div>
              <dt>Close calls (under {metres(1, 1)})</dt>
              <dd className={s.warning_runs.length ? "tone-warn" : undefined}>
                {s.warning_runs.length} flight
                {s.warning_runs.length === 1 ? "" : "s"}
              </dd>
            </div>
            <div>
              <dt>Voltage dips</dt>
              <dd className={s.brownout_runs.length ? "tone-warn" : undefined}>
                {s.brownout_runs.length} flight
                {s.brownout_runs.length === 1 ? "" : "s"}
              </dd>
            </div>
            {s.rain_alert_members !== undefined && (
              <div>
                <dt>Forecasts with a rain return</dt>
                <dd
                  className={s.rain_alert_members > 0 ? "tone-warn" : undefined}
                >
                  {Math.round(s.rain_alert_members * 100)} %
                </dd>
              </div>
            )}
          </dl>
          <WeatherSource report={report!} />
          <p className="muted small">
            Tested on {reportDevice(report!.device)} in{" "}
            {duration(report!.wall_time_sec)}.
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
          title={
            nominal
              ? `Calm air: ${recordResult(nominal)}, closest ${metres(nominal.min_separation_m, 2)}`
              : "Calm air: waiting"
          }
          aria-label={
            nominal ? `Calm air, ${recordResult(nominal)}` : "Calm air, waiting"
          }
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
              title={
                r
                  ? `Flight ${i + 1}: ${recordResult(r)}, closest ${metres(r.min_separation_m, 2)}`
                  : `Flight ${i + 1}: waiting`
              }
              aria-label={
                r
                  ? `Flight ${i + 1}, ${recordResult(r)}`
                  : `Flight ${i + 1}, waiting`
              }
              disabled={!r}
              onClick={() => select(i)}
            />
          );
        })}
      </div>
      <p className="run-grid-key muted small">
        <span
          className="run-cell run-cell-key tone-cell-ok"
          aria-hidden="true"
        />{" "}
        passed
        <span
          className="run-cell run-cell-key tone-cell-warn"
          aria-hidden="true"
        />{" "}
        passed with a close call or voltage dip
        <span
          className="run-cell run-cell-key tone-cell-bad"
          aria-hidden="true"
        />{" "}
        failed
        <span
          className="run-cell run-cell-key run-cell-nominal tone-cell-idle"
          aria-hidden="true"
        />{" "}
        the first light is the show in calm air
      </p>

      {records.length > 0 && (
        <div ref={tableRef}>
          <Table<McRecord>
            className="mc-table"
            size="small"
            style={FRAME}
            scroll={SCROLL}
            pagination={false}
            rowKey="run"
            dataSource={records}
            rowClassName={(r) => (selected === r.run ? "is-selected" : "")}
            onRow={(r) => ({ onClick: () => setSelected(r.run) })}
            columns={[
              {
                title: "Flight",
                key: "flight",
                render: (_, r) => flightName(r),
              },
              {
                title: "Result",
                key: "result",
                onCell: (r) => ({
                  className:
                    recordTone(r) === "ok"
                      ? undefined
                      : `tone-${recordTone(r)}`,
                }),
                render: (_, r) => recordResult(r),
              },
              {
                title: "Closest",
                key: "closest",
                align: "right",
                onCell: (r) => ({
                  className: `num ${(r.min_separation_m ?? Infinity) < D_CRASH_M ? "tone-bad" : ""}`,
                }),
                render: (_, r) => metres(r.min_separation_m, 2),
              },
              {
                title: "Lowest battery",
                key: "soc",
                align: "right",
                onCell: (r) => ({
                  className: `num ${r.min_final_soc < MIN_LANDING_SOC ? "tone-bad" : ""}`,
                }),
                render: (_, r) => soc(r.min_final_soc),
              },
              {
                title: "Wind",
                key: "wind",
                align: "right",
                className: "num",
                render: (_, r) => `${r.scenario.mean_wind_mps.toFixed(1)} m/s`,
              },
              {
                title: "Gust",
                key: "gust",
                align: "right",
                className: "num",
                render: (_, r) => `${r.scenario.gust_peak_mps.toFixed(1)} m/s`,
              },
              {
                title: "Sim speed",
                key: "speed",
                align: "right",
                className: "num",
                render: (_, r) => `${r.realtime_factor.toFixed(2)}×`,
              },
              {
                title: <span className="visually-hidden">Actions</span>,
                key: "action",
                render: (_, r) => {
                  const pair = closestPair(r);
                  return (
                    pair && (
                      <Button
                        type="link"
                        size="small"
                        onClick={(e) => {
                          e.stopPropagation();
                          onShowInReplay(
                            pair.time_sec,
                            [pair.drone_a, pair.drone_b],
                            REFERENCE_NOTE,
                          );
                        }}
                        title={`Drones ${pair.drone_a} and ${pair.drone_b}, ${metres(pair.min_distance_m, 2)} at ${formatTime(pair.time_sec)}`}
                      >
                        Show {pair.drone_a} and {pair.drone_b} in replay
                      </Button>
                    )
                  );
                },
              },
            ]}
          />
        </div>
      )}
      {records.length > 0 && (
        <p className="muted small">
          Sim speed is how much faster than real time the simulation ran; below
          1× a flight takes longer than the show. Flights simulated side by side
          share their batch's average speed.
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

function Pack({
  run,
  onFinished,
}: {
  run: RunRecord;
  onFinished: Props["onFinished"];
}) {
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
    startStage3(run.run_id, run.run_dir, "pack", [], (exit) =>
      onFinished(run.run_id, exit),
    ).catch((e) => setStartError(String(e)));
  }

  const verified = manifest?.files.filter((f) => f.verified).length ?? 0;
  return (
    <section className="card" aria-labelledby="pack-title">
      <h2 id="pack-title" className="stage3-title">
        Flight files
      </h2>
      <p className="muted stage3-about">
        One file per drone with its position and LED colour every 50 ms, in the
        format the drone's flight controller reads. Each file also carries the
        flights home planned under Return paths, and a table that tells the
        drone which one to fly when the return is called. Every file is read
        back and checked after it is written.
      </p>
      {startError && (
        <Alert
          type="error"
          showIcon
          title={`Could not start packing: ${startError}`}
        />
      )}
      {!running && isStale(run, part) && (
        <Alert
          type="warning"
          showIcon
          title="These files are from an earlier Stage 2 result. Pack again before loading them onto drones."
        />
      )}
      {!running &&
        status === "succeeded" &&
        !isStale(run, part) &&
        returnsChanged(run, part) && (
          <Alert
            type="warning"
            showIcon
            title="Return paths were planned after these files were packed, so the files don't carry them. Pack again to include them."
          />
        )}

      {running ? (
        <div className="running-head mc-running">
          <span className="light light-busy" aria-hidden="true" />
          <div>
            <p className="running-phase">
              {PACK_PHASE[job!.phase] ?? job!.phase}
            </p>
            <p className="muted">{run.input.fleet_size} drones</p>
          </div>
          <Button
            danger
            size="large"
            loading={job!.cancelling}
            disabled={job!.jobId < 0}
            onClick={() =>
              cancelStage3(run.run_id, "pack").catch((e) =>
                setCancelError(String(e)),
              )
            }
          >
            {job!.cancelling ? "Cancelling…" : "Cancel"}
          </Button>
          {cancelError && <Alert type="error" showIcon title={cancelError} />}
        </div>
      ) : (
        <div className="actions">
          <Button
            size="large"
            type={status === "not_run" ? "primary" : "default"}
            onClick={start}
          >
            {status === "not_run"
              ? "Pack flight files"
              : "Pack flight files again"}
          </Button>
          {status === "succeeded" && (
            <Button
              size="large"
              onClick={() => openRunFolder(run.run_id, "stage3/bin")}
            >
              Open the files' folder
            </Button>
          )}
        </div>
      )}

      {!running && status === "succeeded" && part && (
        <>
          <VerdictCard
            inner
            tone="ok"
            title={`${part.verified_files} of ${part.files} files written and checked`}
          >
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
              {part.return_tracks && (
                <div>
                  <dt>Flights home</dt>
                  <dd>{part.return_tracks.length}</dd>
                </div>
              )}
            </dl>
          </VerdictCard>
          <PackedReturns part={part} />
          {manifest && (
            <Collapse
              ghost
              size="small"
              className="manifest"
              items={[
                {
                  key: "files",
                  label: `File list (${manifest.files.length} files, ${verified} checked)`,
                  children: (
                    <>
                      {manifest.tracks && manifest.tracks.length > 1 && (
                        <p className="muted small">
                          Each file: the show (
                          {manifest.tracks[0].records.toLocaleString()} samples)
                          and {manifest.tracks.length - 1} flights home (
                          {manifest.tracks
                            .slice(1)
                            .reduce((n, t) => n + t.records, 0)
                            .toLocaleString()}{" "}
                          samples), pack {manifest.pack_id}.
                        </p>
                      )}
                      <Table<PackManifest["files"][number]>
                        size="small"
                        style={FRAME}
                        scroll={SCROLL}
                        pagination={false}
                        rowKey="drone_id"
                        dataSource={manifest.files}
                        columns={[
                          { title: "Drone", dataIndex: "drone_id" },
                          {
                            title: "File",
                            dataIndex: "file",
                            className: "path",
                          },
                          {
                            title: "Samples",
                            key: "samples",
                            align: "right",
                            className: "num",
                            render: () =>
                              manifest.records_per_file.toLocaleString(),
                          },
                          {
                            title: "Size",
                            key: "size",
                            align: "right",
                            className: "num",
                            render: (_, f) =>
                              `${f.size_bytes.toLocaleString()} B`,
                          },
                          {
                            title: "CRC-32",
                            dataIndex: "crc32",
                            className: "path",
                          },
                          {
                            title: "Checked",
                            key: "verified",
                            onCell: (f) => ({
                              className: f.verified ? "tone-ok" : "tone-bad",
                            }),
                            render: (_, f) => (f.verified ? "Yes" : "No"),
                          },
                        ]}
                      />
                    </>
                  ),
                },
              ]}
            />
          )}
        </>
      )}
      {!running &&
        (status === "failed_error" ||
          status === "failed_input" ||
          status === "cancelled") && (
          <Stopped
            runId={run.run_id}
            part="pack"
            status={status}
            message={part?.message}
            job={job}
          />
        )}
    </section>
  );
}

/** Return paths were planned (or planned again) after the files were packed. */
function returnsChanged(run: RunRecord, part: PackPart | undefined): boolean {
  const planned = run.stage2_returns?.ended_at ?? null;
  if (!part || !planned || run.stage2_returns?.status === "running")
    return false;
  if (part.returns_ended_at === undefined) return true; // packed before return paths went into the files
  return part.returns_ended_at !== planned;
}

/** Which flights home the files carry. */
function PackedReturns({ part }: { part: PackPart }) {
  if (!part.return_tracks) {
    return (
      <p className="muted small">
        These files were packed before flights home went into them: they hold
        the show only. Pack again to add them.
      </p>
    );
  }
  const formations = part.return_tracks.filter((t) => t.kind === "return");
  const points = part.return_tracks.filter((t) => t.kind === "abort_point");
  return (
    <div className="packed-returns">
      {part.returns_note && (
        <Alert
          type="warning"
          showIcon
          title={`Flights home left out: ${part.returns_note}.`}
        />
      )}
      {part.return_tracks.length === 0 ? (
        <p className="muted small">
          No flights home are planned, so if the return is called the drones
          keep flying the show to its own return leg (or have no planned way
          home when the show has none). Plan them under Stage 2 › Return paths,
          then pack again.
        </p>
      ) : (
        <p className="muted small">
          {formations.length > 0 && (
            <>
              Flights home from{" "}
              {formations.map((t) => t.formation_name).join(", ")}
            </>
          )}
          {formations.length > 0 && points.length > 0 && "; "}
          {points.length > 0 && (
            <>
              from inside a move at{" "}
              {points
                .map(
                  (t) => `${formatTime(t.start_sec)} (to ${t.formation_name})`,
                )
                .join(", ")}
            </>
          )}
          . Every drone has the same return table ({part.return_entries}{" "}
          entries) and pack id {part.pack_id}, so the whole fleet picks the same
          flight home.
        </p>
      )}
    </div>
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
      <Alert
        type={cancelled ? "info" : "error"}
        showIcon
        title={
          cancelled
            ? "Cancelled before it finished."
            : (message ?? "Stopped with an error.")
        }
      />
      {lines.length > 0 && (
        <LogPane
          title={`Output (${lines.length} lines)`}
          text={lines.slice(-200).join("\n")}
        />
      )}
    </div>
  );
}
