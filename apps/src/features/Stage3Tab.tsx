// Stage 3, the digital twin: the Monte Carlo stress test, weather scenarios (Conditions) and the flight
// files, as three sections of one tab.

import { Flex, Segmented, Typography } from "antd";
import type { JobExit, RunRecord, SimDevice } from "../bridge/types";
import { TONE_TEXT } from "../app/format";
import {
  conditionsState,
  packState,
  stressState,
  type StageState,
} from "../app/stages";
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
  onPlayScenario: (
    id: string,
    name: string,
    time: number,
    drones: number[],
  ) => void;
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
    ["stress", "Stress test", stressState(run)],
    ["conditions", "Weather scenarios", conditionsState(run)],
    ["pack", "Flight files", packState(run)],
  ];
  return (
    <>
      <div className="subtabs">
        <Segmented<Stage3Section>
          aria-label="Stage 3"
          size="large"
          value={section}
          onChange={onSection}
          options={sections.map(([id, label, st]) => ({
            value: id,
            label: (
              <Flex align="center" gap={8}>
                <Typography.Text strong style={{ color: "inherit" }}>
                  {label}
                </Typography.Text>
                <Typography.Text type={TONE_TEXT[st.tone]} style={{ fontSize: 12 }}>
                  {st.text}
                </Typography.Text>
              </Flex>
            ),
          }))}
        />
      </div>
      {section === "conditions" ? (
        <ConditionsView
          run={run}
          onFinished={onConditionsFinished}
          onPlay={onPlayScenario}
          scenarioId={scenarioId}
        />
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
