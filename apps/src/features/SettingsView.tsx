// Where Studio finds the repo, its Python and the run folders, in a modal
// dialog over the current page.

import { useState } from "react";
import { Button, Form, Input, Modal, Space, Table, Typography } from "antd";
import { saveSettings } from "../bridge/api";
import type { DoctorCheck, Settings } from "../bridge/types";

interface Props {
  settings: Settings;
  checks: DoctorCheck[] | null;
  onSaved: (s: Settings) => void;
  onRecheck: () => void;
  onClose: () => void;
}

const FIELDS: { key: keyof Settings; label: string; help: string }[] = [
  { key: "repo_root", label: "Repository folder", help: "The folder that contains stage2_core_engine." },
  { key: "python", label: "Python", help: "The interpreter drone_core was built for (the repo's .venv)." },
  { key: "runs_dir", label: "Run folders", help: "Where each run's input, results and replay are kept." },
];

export function SettingsView({ settings, checks, onSaved, onRecheck, onClose }: Props) {
  const [message, setMessage] = useState<{ tone: "success" | "danger"; text: string } | null>(null);

  async function save(values: Settings) {
    try {
      const saved = await saveSettings(values);
      onSaved(saved);
      setMessage({ tone: "success", text: "Settings saved." });
    } catch (e) {
      setMessage({ tone: "danger", text: String(e) });
    }
  }

  return (
    <Modal open title="Settings" onCancel={onClose} footer={null} width={760} centered>
      <Form layout="vertical" initialValues={settings} onFinish={save} requiredMark={false}>
        {FIELDS.map((f, i) => (
          <Form.Item key={f.key} name={f.key} label={f.label} extra={f.help}>
            {/* Start in the first field rather than on the close button. */}
            <Input className="mono-input" spellCheck={false} autoFocus={i === 0} />
          </Form.Item>
        ))}
        <Space size="middle">
          <Button type="primary" htmlType="submit">
            Save settings
          </Button>
          {message && <Typography.Text type={message.tone}>{message.text}</Typography.Text>}
        </Space>
      </Form>

      <h2 className="section-title">Components</h2>
      <p className="muted">What Studio found when it started. Missing pieces disable the steps that need them.</p>
      <Table<DoctorCheck>
        size="small"
        showHeader={false}
        pagination={false}
        rowKey="name"
        loading={!checks}
        dataSource={checks ?? []}
        columns={[
          {
            key: "name",
            render: (_, c) => <Typography.Text type={c.ok ? "success" : "danger"} style={{ whiteSpace: "nowrap" }}>{c.name}</Typography.Text>,
          },
          { dataIndex: "required_for", className: "muted" },
          {
            key: "detail",
            className: "path",
            render: (_, c) => (c.ok ? c.detail : <Typography.Text type="danger">{c.detail}</Typography.Text>),
          },
        ]}
      />
      <div className="actions">
        <Button onClick={onRecheck}>Check again</Button>
      </div>
    </Modal>
  );
}
