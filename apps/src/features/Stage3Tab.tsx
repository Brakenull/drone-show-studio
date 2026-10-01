// Stage 3, the digital twin: the Monte Carlo stress test, weather scenarios (Conditions) and the flight
// files, as three sections of one tab (docs/5-studio_gui.md §6.3, §6.7).

import type { JobExit, RunRecord, SimDevice } from "../bridge/types";
import { conditionsState, packState, stressState, type StageState } from "../app/stages";
import { Stage3View } from "./Stage3View";
import { ConditionsView } from "./ConditionsView";

export type Stage3Section = "stress" | "conditions" | "pack";

interface Props {
  run: RunRecord;
  section: Stage3Section;
  onSection: (s: Stage3Section) => void;
  devices: SimDevice[] | null;
  onFinished: (runId: string, exit: JobExit) => void;
  /** A weather simulation finished: refresh the run list. */
  onConditionsFinished: () => void;
  onShowInReplay: (time: number, drones: number[], note: string) => void;
  /** Play a simulated weather scenario in the Replay tab. */
  onPlayScenario: (id: string, name: string, time: number, drones: number[]) => void;
  /** The weather scenario to show first. */
  scenarioId: string | null;
}

export function Stage3Tab({
  run,
  section,
  onSection,
  devices,
  onFinished,
  onConditionsFinished,
  onShowInReplay,
  onPlayScenario,
  scenarioId,
}: Props) {
  const sections: [Stage3Section, string, StageState][] = [
    ["stress", "Random weather (stress test)", stressState(run)],
    ["conditions", "Weather scenarios", conditionsState(run)],
    ["pack", "Flight files", packState(run)],
  ];
  return (
    <>
      <div className="subtabs" role="tablist" aria-label="Stage 3">
        {sections.map(([id, label, st]) => (
          <button
            key={id}
            role="tab"
            aria-selected={section === id}
            className={`subtab ${section === id ? "is-current" : ""}`}
            onClick={() => onSection(id)}
          >
            <span className={`light light-${st.tone}`} aria-hidden="true" />
            <span className="subtab-label">{label}</span>
            <span className={`subtab-status tone-${st.tone}`}>{st.text}</span>
          </button>
        ))}
      </div>
      {section === "conditions" ? (
        <ConditionsView run={run} onFinished={onConditionsFinished} onPlay={onPlayScenario} scenarioId={scenarioId} />
      ) : (
        <Stage3View
          run={run}
          part={section}
          devices={devices}
          onFinished={onFinished}
          onShowInReplay={onShowInReplay}
        />
      )}
    </>
  );
}
