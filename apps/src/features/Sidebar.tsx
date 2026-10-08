import { Button, Empty, Flex, Layout, Menu, Progress, Tag, Tooltip, Typography, theme } from "antd";
import type { RunRecord } from "../bridge/types";
import { STATUS, TONE_TAG, runCreated, runName, type Tone } from "../app/format";
import { packState, stage2State, stressState } from "../app/stages";

interface Props {
  runs: RunRecord[];
  selectedRunId: string | null;
  view: "new" | "run";
  settingsOpen: boolean;
  onNew: () => void;
  onSelect: (runId: string) => void;
  onSettings: () => void;
}

export function Sidebar({ runs, selectedRunId, view, settingsOpen, onNew, onSelect, onSettings }: Props) {
  const { token } = theme.useToken();
  const pipeColor: Record<Tone, string> = {
    ok: token.colorSuccess,
    bad: token.colorError,
    warn: token.colorWarning,
    busy: token.colorInfo,
    idle: token.colorBorder,
  };

  const items = runs.map((run) => {
    const status = STATUS[run.stage2.status] ?? STATUS.not_run;
    const s2 = stage2State(run);
    const pipe = [
      ["Stage 2", s2],
      ["Stress test", stressState(run)],
      ["Flight files", packState(run)],
    ] as const;
    return {
      key: run.run_id,
      label: (
        <Flex vertical gap={4}>
          <Flex align="center" gap={8}>
            <Typography.Text strong ellipsis style={{ flex: 1 }}>
              {runName(run)}
            </Typography.Text>
            <Tag color={TONE_TAG[s2.tone]} variant="filled" style={{ marginInlineEnd: 0 }}>
              {s2.tone === "busy" ? "Running" : status.label}
            </Tag>
          </Flex>
          <Typography.Text type="secondary" style={{ fontSize: token.fontSizeSM }}>
            {runCreated(run)} · {run.input.fleet_size} drones
            {run.copied_from && " · copy"}
          </Typography.Text>
          <Tooltip title={pipe.map(([name, st]) => `${name}: ${st.text}`).join(" · ")}>
            <Progress
              className="run-pipe"
              percent={100}
              steps={3}
              size={[68, 4]}
              showInfo={false}
              strokeColor={pipe.map(([, st]) => pipeColor[st.tone])}
            />
          </Tooltip>
        </Flex>
      ),
    };
  });

  return (
    <Layout.Sider width={296} theme="light" aria-label="Runs" style={{ borderRight: `1px solid ${token.colorBorder}` }}>
      <Flex vertical style={{ height: "100%" }}>
        <Flex justify="space-between" align="center" style={{ padding: "24px 20px 12px" }}>
          <div>
            <Typography.Text type="secondary">Drone Show Studio</Typography.Text>
            <Typography.Title level={3} style={{ margin: 0 }}>
              Runs
            </Typography.Title>
          </div>
          <Flex align="center" gap={4}>
            <Tag variant="filled">{runs.length}</Tag>
            <Button
              color={settingsOpen ? "primary" : "default"}
              variant="text"
              onClick={onSettings}
              title="Settings"
              aria-label="Settings"
              icon={
                <svg width="20" height="20" viewBox="0 0 20 20" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" aria-hidden="true">
                  <path d="M3 6h14M3 14h14" />
                  <circle cx="13" cy="6" r="2.2" fill={token.colorBgContainer} />
                  <circle cx="7" cy="14" r="2.2" fill={token.colorBgContainer} />
                </svg>
              }
            />
          </Flex>
        </Flex>
        {runs.length === 0 ? (
          <Empty image={Empty.PRESENTED_IMAGE_SIMPLE} description="Runs you create appear here, newest first." style={{ flex: 1 }} />
        ) : (
          <Menu
            className="run-menu"
            mode="inline"
            items={items}
            selectedKeys={view === "run" && selectedRunId ? [selectedRunId] : []}
            onClick={({ key }) => onSelect(key)}
            style={{ flex: 1, overflowY: "auto", borderInlineEnd: 0 }}
          />
        )}
        <Button
          type="primary"
          size="large"
          block
          onClick={onNew}
          style={{ margin: 20, width: "auto" }}
          icon={
            <svg width="16" height="16" viewBox="0 0 20 20" fill="none" stroke="currentColor" strokeWidth="2.4" strokeLinecap="round" aria-hidden="true">
              <path d="M10 4v12M4 10h12" />
            </svg>
          }
        >
          New run
        </Button>
      </Flex>
    </Layout.Sider>
  );
}
