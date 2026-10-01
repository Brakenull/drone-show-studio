import { useCallback, useEffect, useState } from "react";
import { getSettings, listRuns, runJob } from "./bridge/api";
import type { DoctorCheck, RunRecord, Settings, SimDevice } from "./bridge/types";
import { STATUS, runName } from "./app/format";
import { isStage2Running } from "./app/stage2Jobs";
import { isStage3Running, useStage3Job } from "./app/stage3Jobs";
import { Sidebar } from "./features/Sidebar";
import { NewRun } from "./features/NewRun";
import { InputView } from "./features/InputView";
import { Stage2View } from "./features/Stage2View";
import { Stage3View } from "./features/Stage3View";
import { ReplayView, type ReplaySource } from "./features/ReplayView";
import { CompareView } from "./features/CompareView";
import { SettingsView } from "./features/SettingsView";
import type { ReplayFocus } from "./replay/types";
import "./styles.css";

type View = "new" | "run" | "settings";
type Tab = "input" | "stage2" | "stage3" | "replay" | "compare";

const TABS: { id: Tab; label: string }[] = [
  { id: "input", label: "Input" },
  { id: "stage2", label: "Stage 2" },
  { id: "stage3", label: "Stage 3" },
  { id: "replay", label: "Replay" },
  { id: "compare", label: "Compare" },
];

const runTone = (run: RunRecord) =>
  (isStage2Running(run.run_id) || isStage3Running(run.run_id) ? STATUS.running : STATUS[run.stage2.status]).tone;

export default function App() {
  const [settings, setSettings] = useState<Settings | null>(null);
  const [checks, setChecks] = useState<DoctorCheck[] | null>(null);
  /** Simulation devices from `doctor`; null until it has answered. */
  const [devices, setDevices] = useState<SimDevice[] | null>(null);
  const [doctorError, setDoctorError] = useState<string | null>(null);
  const [runs, setRuns] = useState<RunRecord[]>([]);
  const [view, setView] = useState<View>("new");
  const [selected, setSelected] = useState<string | null>(null);
  const [tab, setTab] = useState<Tab>("stage2");
  const [focus, setFocus] = useState<ReplayFocus | null>(null);
  const [replaySource, setReplaySource] = useState<ReplaySource>({ kind: "show" });

  const refreshRuns = useCallback(async () => {
    try {
      setRuns(await listRuns());
    } catch (e) {
      console.error(e);
    }
  }, []);

  const runDoctor = useCallback(async () => {
    setChecks(null);
    setDoctorError(null);
    try {
      const { events } = await runJob(["doctor"]);
      const d = events.find((e) => e.type === "doctor");
      if (d && d.type === "doctor") {
        setChecks(d.checks);
        setDevices(d.device_info ?? []);
      }
      else setDoctorError("The component check produced no result.");
    } catch (e) {
      setDoctorError(String(e));
    }
  }, []);

  useEffect(() => {
    getSettings().then(setSettings);
    void refreshRuns();
    void runDoctor();
  }, [refreshRuns, runDoctor]);

  const run = runs.find((r) => r.run_id === selected) ?? null;
  // Re-render when the selected run's Stage 3 jobs start or stop, so its status light follows them.
  useStage3Job(selected, "monte_carlo");
  useStage3Job(selected, "pack");
  const missing = checks?.filter((c) => !c.ok && ["stage2", "validate", "replay", "all"].includes(c.required_for));

  function openRun(runId: string, nextTab: Tab = "stage2") {
    setSelected(runId);
    setView("run");
    setTab(nextTab);
    setFocus(null);
    setReplaySource({ kind: "show" });
  }

  return (
    <div className="app">
      <Sidebar
        runs={runs}
        selectedRunId={selected}
        view={view}
        onNew={() => setView("new")}
        onSelect={(id) => openRun(id, id === selected ? tab : "stage2")}
        onSettings={() => setView("settings")}
      />
      <main className="main">
        {(doctorError || (missing && missing.length > 0)) && (
          <div className="banner" role="alert">
            {doctorError ? (
              <>Studio can't start its Python helper: {doctorError}. Check the Python path in Settings.</>
            ) : (
              <>
                Missing: {missing!.map((c) => c.name).join(", ")}. Steps that need them won't work.{" "}
                <button className="link" onClick={() => setView("settings")}>
                  See details
                </button>
              </>
            )}
          </div>
        )}

        {view === "settings" && settings && (
          <SettingsView
            settings={settings}
            checks={checks}
            onSaved={(saved) => {
              setSettings(saved);
              void refreshRuns();
              void runDoctor();
            }}
            onRecheck={runDoctor}
          />
        )}

        {view === "new" && settings && (
          <NewRun
            runsDir={settings.runs_dir}
            onCreated={async (id) => {
              await refreshRuns();
              openRun(id, "stage2");
            }}
          />
        )}

        {view === "run" && run && (
          <div className="run-view">
            <header className="run-head">
              <div className="run-title">
                <span className={`light light-${runTone(run)}`} aria-hidden="true" />
                <h1>{runName(run)}</h1>
                <span className="muted">
                  {run.input.fleet_size} drones, {run.input.keyframes.length}{" "}
                  {run.input.keyframes.length === 1 ? "formation" : "formations"}
                </span>
              </div>
              <div className="tabs" role="tablist">
                {TABS.map((t) => (
                  <button
                    key={t.id}
                    role="tab"
                    aria-selected={tab === t.id}
                    className={tab === t.id ? "is-current" : ""}
                    onClick={() => {
                      // The Replay tab itself always opens the show; a return path is opened from Stage 2.
                      if (t.id === "replay") {
                        setReplaySource({ kind: "show" });
                        setFocus(null);
                      }
                      setTab(t.id);
                    }}
                  >
                    {t.label}
                  </button>
                ))}
              </div>
            </header>
            <div className="run-body">
              {tab === "input" && <InputView run={run} />}
              {tab === "stage2" && (
                <Stage2View
                  run={run}
                  runsDir={settings?.runs_dir ?? ""}
                  onCreated={async (id) => {
                    await refreshRuns();
                    openRun(id, "stage2");
                  }}
                  onChanged={refreshRuns}
                  onFinished={() => void refreshRuns()}
                  onShowInReplay={(time, drones) => {
                    setReplaySource({ kind: "show" });
                    setFocus({ time, drones, key: Date.now() });
                    setTab("replay");
                  }}
                  onViewReturn={(keyframe, from, abortTime) => {
                    setReplaySource({ kind: "return", keyframe, from });
                    setFocus({ time: abortTime, drones: [], key: Date.now() });
                    setTab("replay");
                  }}
                />
              )}
              {tab === "stage3" && (
                <Stage3View
                  run={run}
                  devices={devices}
                  onFinished={() => void refreshRuns()}
                  onShowInReplay={(time, drones, note) => {
                    setReplaySource({ kind: "show" });
                    setFocus({ time, drones, note, key: Date.now() });
                    setTab("replay");
                  }}
                />
              )}
              {tab === "replay" && (
                <ReplayView
                  run={run}
                  focus={focus}
                  source={replaySource}
                  onShowPlayback={() => {
                    setReplaySource({ kind: "show" });
                    setFocus(null);
                  }}
                />
              )}
              {tab === "compare" && <CompareView key={run.run_id} run={run} runs={runs} />}
            </div>
          </div>
        )}
      </main>
    </div>
  );
}
