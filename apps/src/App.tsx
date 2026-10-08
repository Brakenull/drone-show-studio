import { useCallback, useEffect, useState } from "react";
import { Alert, Button, Flex, Tabs, Tag, Typography } from "antd";
import { getSettings, listRuns, runJob } from "./bridge/api";
import type { DoctorCheck, RunRecord, Settings, SimDevice } from "./bridge/types";
import { STATUS, TONE_TAG, TONE_TEXT, runCreated, runName } from "./app/format";
import { useStage3Job } from "./app/stage3Jobs";
import { useSimulateJob } from "./app/simulateJobs";
import { useReturnsJob } from "./app/returnsJobs";
import { compareState, inputState, replayState, stage2State, stage3State, type StageState } from "./app/stages";
import { Sidebar } from "./features/Sidebar";
import { NewRun } from "./features/NewRun";
import { InputView } from "./features/InputView";
import { Stage2View } from "./features/Stage2View";
import { Stage3Tab, type Stage3Section } from "./features/Stage3Tab";
import { ReplayView, type ReplaySource } from "./features/ReplayView";
import { CompareView } from "./features/CompareView";
import { SettingsView } from "./features/SettingsView";
import type { ReplayFocus } from "./replay/types";
import "./styles.css";

type View = "new" | "run";
type Tab = "input" | "stage2" | "stage3" | "replay" | "compare";

/** The stage cards in the run header: one per tab, each with its part's state. */
const TABS: { id: Tab; label: string; state: (run: RunRecord, runs: RunRecord[]) => StageState }[] = [
  { id: "input", label: "Input", state: inputState },
  { id: "stage2", label: "Stage 2", state: stage2State },
  { id: "stage3", label: "Stage 3", state: stage3State },
  { id: "replay", label: "Replay", state: replayState },
  { id: "compare", label: "Compare", state: compareState },
];

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
  const [stage3Section, setStage3Section] = useState<Stage3Section>("stress");
  const [settingsOpen, setSettingsOpen] = useState(false);
  /** The weather scenario Stage 3 › Weather scenarios opens on ("Edit this weather" in the Replay tab). */
  const [scenarioId, setScenarioId] = useState<string | null>(null);
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
  useSimulateJob(selected);
  useReturnsJob(selected);
  const missing = checks?.filter((c) => !c.ok && ["stage2", "validate", "replay", "all"].includes(c.required_for));

  function openRun(runId: string, nextTab: Tab = "stage2") {
    setSelected(runId);
    setView("run");
    setTab(nextTab);
    setFocus(null);
    setReplaySource({ kind: "show" });
    setScenarioId(null);
  }

  return (
    <div className="app">
      <Sidebar
        runs={runs}
        selectedRunId={selected}
        view={view}
        settingsOpen={settingsOpen}
        onNew={() => setView("new")}
        onSelect={(id) => openRun(id, id === selected ? tab : "stage2")}
        onSettings={() => setSettingsOpen(true)}
      />
      <main className="main">
        {(doctorError || (missing && missing.length > 0)) && (
          <Alert
            banner
            type={doctorError ? "error" : "warning"}
            title={
              doctorError
                ? `Studio can't start its Python helper: ${doctorError}. Check the Python path in Settings.`
                : `Missing: ${missing!.map((c) => c.name).join(", ")}. Steps that need them won't work.`
            }
            action={
              !doctorError && (
                <Button type="link" size="small" onClick={() => setSettingsOpen(true)}>
                  See details
                </Button>
              )
            }
          />
        )}

        {settingsOpen && settings && (
          <SettingsView
            settings={settings}
            checks={checks}
            onSaved={(saved) => {
              setSettings(saved);
              void refreshRuns();
              void runDoctor();
            }}
            onRecheck={runDoctor}
            onClose={() => setSettingsOpen(false)}
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
              <Flex align="center" gap={12} style={{ minWidth: 0 }}>
                <Typography.Title level={3} ellipsis style={{ margin: 0 }}>
                  {runName(run)}
                </Typography.Title>
                <Tag color={TONE_TAG[stage2State(run).tone]} variant="filled">
                  {stage2State(run).tone === "busy" ? "Running" : STATUS[run.stage2.status].label}
                </Tag>
              </Flex>
              <Flex wrap gap="4px 20px">
                <Typography.Text type="secondary">
                  {run.input.fleet_size} drones, {run.input.keyframes.length}{" "}
                  {run.input.keyframes.length === 1 ? "formation" : "formations"}
                </Typography.Text>
                <Typography.Text type="secondary">Created {runCreated(run)}</Typography.Text>
                {run.copied_from && <Typography.Text type="secondary">Copy of {runName({ run_id: run.copied_from })}</Typography.Text>}
                <Typography.Text type="secondary" copyable>
                  {run.run_id}
                </Typography.Text>
              </Flex>
              <Tabs
                className="run-tabs"
                activeKey={tab}
                tabBarStyle={{ marginBottom: 0 }}
                // onTabClick, not onChange: clicking Replay again still resets it to the show.
                onTabClick={(key) => {
                  // The Replay tab itself always opens the show; a return path is opened from Stage 2.
                  if (key === "replay") {
                    setReplaySource({ kind: "show" });
                    setFocus(null);
                  }
                  setTab(key as Tab);
                }}
                items={TABS.map((t, i) => {
                  const st = t.state(run, runs);
                  return {
                    key: t.id,
                    label: (
                      <Flex vertical align="start" style={{ minWidth: 120, lineHeight: 1.4 }}>
                        <Typography.Text type="secondary" code>
                          {String(i + 1).padStart(2, "0")}
                        </Typography.Text>
                        <Typography.Text strong style={{ color: "inherit" }}>
                          {t.label}
                        </Typography.Text>
                        <Typography.Text
                          type={TONE_TEXT[st.tone]}
                          ellipsis
                          className="run-tab-status"
                          style={{ fontSize: 12, maxWidth: 160 }}
                        >
                          {st.text}
                        </Typography.Text>
                      </Flex>
                    ),
                  };
                })}
              />
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
                <Stage3Tab
                  run={run}
                  section={stage3Section}
                  onSection={setStage3Section}
                  devices={devices}
                  onFinished={() => void refreshRuns()}
                  onConditionsFinished={() => void refreshRuns()}
                  scenarioId={scenarioId}
                  onPlayScenario={(id, name, time, drones) => {
                    setReplaySource({ kind: "scenario", id, name });
                    setFocus({ time, drones, key: Date.now() });
                    setTab("replay");
                  }}
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
                  onSource={(s) => {
                    setReplaySource(s);
                    setFocus(null);
                  }}
                  onEditScenario={(id) => {
                    setScenarioId(id);
                    setStage3Section("conditions");
                    setTab("stage3");
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
