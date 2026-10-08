// Planner settings for a Stage 2 run, and the buttons that run it.

import { useEffect, useMemo, useState } from "react";
import { Alert, Button, Checkbox, Collapse, InputNumber, Select, Table } from "antd";
import { runJob } from "../bridge/api";
import { LogPane } from "./JobOutput";
import type { ConfigField, Overrides, RunRecord } from "../bridge/types";
import { changed, flatten, nest, parse, warningsFor, type Value, type Values } from "../app/overrides";

interface Props {
  run: RunRecord;
  runLabel: string;
  primary: boolean;
  /** Why the run button is off, e.g. a Stage 3 job is reading this run's output. */
  blocked: string | null;
  onRun: (overrides: Overrides) => void;
  onCopy: (overrides: Overrides) => Promise<void>;
}

/** Groups most people need; the solver-tuning ones sit folded under them. */
const MAIN_GROUPS = new Set(["Safety check", "Motion limits", "Takeoff and landing"]);

const show = (v: Value) => (typeof v === "boolean" ? (v ? "On" : "Off") : v === "disabled" ? "Off" : String(v));

// Flush headers and bodies, so the tables line up with the card's edge.
const FLUSH = { header: { paddingInline: 0 }, body: { paddingInline: 0 } };

export function PlannerSettings({ run, runLabel, primary, blocked, onRun, onCopy }: Props) {
  const [fields, setFields] = useState<ConfigField[] | null>(null);
  const [loadError, setLoadError] = useState<string | null>(null);
  const [values, setValues] = useState<Values>({});
  const [drafts, setDrafts] = useState<Record<string, string>>({});
  const [errors, setErrors] = useState<Record<string, string>>({});
  const [accepted, setAccepted] = useState(false);
  const [copying, setCopying] = useState(false);
  const [copyError, setCopyError] = useState<string | null>(null);

  // The saved settings change when Stage 2 runs, so reload with its end time.
  useEffect(() => {
    let live = true;
    setFields(null);
    setLoadError(null);
    runJob(["config", run.run_dir])
      .then(({ events }) => {
        const e = events.find((x) => x.type === "config");
        if (!live) return;
        if (e && e.type === "config") {
          setFields(e.fields);
          setValues(flatten(e.overrides));
          setDrafts({});
          setErrors({});
        } else setLoadError("Couldn't read the planner settings.");
      })
      .catch((e) => live && setLoadError(String(e)));
    return () => {
      live = false;
    };
  }, [run.run_id, run.run_dir, run.stage2.ended_at]);

  const overrides = useMemo(() => (fields ? changed(fields, values) : {}), [fields, values]);
  const warnings = useMemo(() => (fields ? warningsFor(fields, overrides) : []), [fields, overrides]);
  const count = Object.keys(overrides).length;
  const invalid = Object.keys(errors).length > 0;
  useEffect(() => setAccepted(false), [warnings.length]);

  const groups = useMemo(() => {
    const out = new Map<string, ConfigField[]>();
    for (const f of fields ?? []) out.set(f.group, [...(out.get(f.group) ?? []), f]);
    return [...out];
  }, [fields]);

  function set(f: ConfigField, text: string) {
    setDrafts((d) => ({ ...d, [f.path]: text }));
    const v = parse(f, text);
    if (typeof v === "object") {
      setErrors((e) => ({ ...e, [f.path]: v.error }));
      return;
    }
    setErrors(({ [f.path]: _, ...rest }) => rest);
    setValues((vals) => ({ ...vals, [f.path]: v }));
  }

  function revert(path: string) {
    setValues(({ [path]: _, ...rest }) => rest);
    setDrafts(({ [path]: _, ...rest }) => rest);
    setErrors(({ [path]: _, ...rest }) => rest);
  }

  function resetAll() {
    setValues({});
    setDrafts({});
    setErrors({});
  }

  async function copy() {
    setCopying(true);
    setCopyError(null);
    try {
      await onCopy(nest(overrides));
    } catch (e) {
      setCopyError(String(e));
    } finally {
      setCopying(false);
    }
  }

  const input = (f: ConfigField) => {
    const current = f.path in values ? values[f.path] : f.baseline;
    const status = errors[f.path] || warnings.some((w) => w.path === f.path) ? "error" : undefined;
    if (f.kind === "boolean" || f.kind === "choice") {
      const options = f.kind === "boolean" ? ["true", "false"] : (f.choices ?? []);
      return (
        <Select
          aria-label={f.label}
          className="setting-input"
          value={String(current)}
          status={status}
          onChange={(v) => set(f, v)}
          options={options.map((o) => ({
            value: o,
            label: f.kind === "boolean" ? (o === "true" ? "On" : "Off") : o === "4_sector_discrete" ? "On (4 sectors)" : "Off",
          }))}
        />
      );
    }
    // No min here: clamping on blur would hide the "At least" message.
    return (
      <InputNumber<string>
        stringMode
        controls={false}
        aria-label={f.label}
        className="setting-input"
        status={status}
        value={drafts[f.path] ?? String(current)}
        onChange={(v) => set(f, v ?? "")}
      />
    );
  };

  const table = (group: string, gf: ConfigField[]) => (
    <Table<ConfigField>
      key={group}
      className="settings-table"
      size="small"
      pagination={false}
      rowKey="path"
      title={() => <strong>{group}</strong>}
      dataSource={gf}
      rowClassName={(f) => (f.path in overrides ? "is-changed" : "")}
      columns={[
        {
          title: "Setting",
          key: "label",
          width: "45%",
          render: (_, f) => (
            <>
              {f.label}
              {f.help && <span className="setting-help">{f.help}</span>}
            </>
          ),
        },
        {
          title: "Default",
          key: "default",
          align: "right",
          className: "num muted",
          render: (_, f) => (
            <>
              {show(f.baseline)}
              {f.baseline !== f.default && <span className="setting-help">from the show file</span>}
            </>
          ),
        },
        {
          title: "This run",
          key: "value",
          render: (_, f) => (
            <>
              {input(f)}
              {errors[f.path] && <span className="setting-help tone-bad">{errors[f.path]}</span>}
            </>
          ),
        },
        {
          title: <span className="visually-hidden">Undo</span>,
          key: "undo",
          width: "4.5rem",
          render: (_, f) =>
            (f.path in overrides || errors[f.path]) && (
              <Button type="link" size="small" onClick={() => revert(f.path)}>
                Undo
              </Button>
            ),
        },
      ]}
    />
  );

  const main = groups.filter(([g]) => MAIN_GROUPS.has(g));
  const tuning = groups.filter(([g]) => !MAIN_GROUPS.has(g));

  return (
    <section className="planner card planner-card" aria-label="Run Stage 2">
      {loadError && <Alert type="error" showIcon title={loadError} />}
      {fields && (
        <Collapse
          ghost
          styles={FLUSH}
          classNames={{ title: "planner-settings-title" }}
          defaultActiveKey={count > 0 ? ["settings"] : []}
          items={[
            {
              key: "settings",
              label: (
                <>
                  Planner settings{" "}
                  <span className="muted">{count ? `${count} changed from the defaults` : "all at their defaults"}</span>
                </>
              ),
              children: (
                <>
                  <p className="muted small planner-about">
                    Defaults come from the planner's configuration file; motion limits can come from the show file. Only
                    the settings you change are sent to Stage 2.
                  </p>
                  {main.map(([group, gf]) => table(group, gf))}
                  <Collapse
                    ghost
                    styles={FLUSH}
                    className="settings-advanced"
                    defaultActiveKey={tuning.some(([, gf]) => gf.some((f) => f.path in overrides)) ? ["tuning"] : []}
                    items={[
                      {
                        key: "tuning",
                        label: (
                          <>
                            <strong>Solver tuning</strong>{" "}
                            <span className="muted">({tuning.reduce((n, [, gf]) => n + gf.length, 0)} settings)</span>
                          </>
                        ),
                        children: tuning.map(([group, gf]) => table(group, gf)),
                      },
                    ]}
                  />
                  <div className="actions">
                    <Button onClick={resetAll} disabled={!count && !invalid}>
                      Reset all to defaults
                    </Button>
                  </div>
                  <LogPane
                    title="What Stage 2 receives"
                    text={count ? JSON.stringify(nest(overrides), null, 2) : "{}  (no changes: the defaults apply)"}
                  />
                </>
              ),
            },
          ]}
        />
      )}

      {warnings.length > 0 && (
        <Alert
          type="error"
          showIcon
          className="planner-warnings"
          title="These settings make the safety check weaker than the defaults."
          description={
            <>
              <p>A show that passes with them may not be safe to fly.</p>
              <ul>
                {warnings.map((w, i) => (
                  <li key={i}>{w.message}</li>
                ))}
              </ul>
              <Checkbox checked={accepted} onChange={(e) => setAccepted(e.target.checked)}>
                Run with these settings anyway
              </Checkbox>
            </>
          }
        />
      )}

      <div className="actions">
        <Button
          type={primary ? "primary" : "default"}
          size="large"
          disabled={!fields || invalid || (warnings.length > 0 && !accepted) || !!blocked}
          onClick={() => onRun(nest(overrides))}
        >
          {runLabel}
        </Button>
        {run.stage2.status !== "not_run" && (
          <Button size="large" onClick={copy} disabled={!fields || invalid} loading={copying}>
            Try these settings in a new run
          </Button>
        )}
        {blocked && <span className="muted small">{blocked}</span>}
        {invalid && <span className="tone-bad small">Fix the highlighted settings first.</span>}
      </div>
      {copyError && <Alert type="error" showIcon title={copyError} />}
      {run.stage2.status !== "not_run" && (
        <p className="muted small">
          Running again here replaces this run's result. A new run keeps it, so you can compare the two.
        </p>
      )}
    </section>
  );
}
