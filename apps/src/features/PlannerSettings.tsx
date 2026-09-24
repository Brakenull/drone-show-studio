// Planner settings for a Stage 2 run, and the buttons that run it (docs/5-studio_gui.md §6.2).

import { useEffect, useMemo, useState } from "react";
import { runJob } from "../bridge/api";
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
const MAIN_GROUPS = new Set(["Safety check", "Motion limits", "Takeoff"]);

const show = (v: Value) => (typeof v === "boolean" ? (v ? "On" : "Off") : v === "disabled" ? "Off" : String(v));

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
    const worse = warnings.some((w) => w.path === f.path);
    if (f.kind === "boolean" || f.kind === "choice") {
      const options = f.kind === "boolean" ? ["true", "false"] : (f.choices ?? []);
      return (
        <select
          aria-label={f.label}
          value={String(current)}
          className={worse ? "is-worse" : undefined}
          onChange={(e) => set(f, e.target.value)}
        >
          {options.map((o) => (
            <option key={o} value={o}>
              {f.kind === "boolean" ? (o === "true" ? "On" : "Off") : o === "4_sector_discrete" ? "On (4 sectors)" : "Off"}
            </option>
          ))}
        </select>
      );
    }
    return (
      <input
        type="number"
        aria-label={f.label}
        aria-invalid={!!errors[f.path]}
        className={worse ? "is-worse" : undefined}
        min={f.min ?? undefined}
        step="any"
        value={drafts[f.path] ?? String(current)}
        onChange={(e) => set(f, e.target.value)}
      />
    );
  };

  const table = (group: string, gf: ConfigField[]) => (
    <table key={group} className="table settings-table">
      <caption>{group}</caption>
      <thead>
        <tr>
          <th scope="col">Setting</th>
          <th scope="col" className="num">Default</th>
          <th scope="col">This run</th>
          <th scope="col">
            <span className="visually-hidden">Undo</span>
          </th>
        </tr>
      </thead>
      <tbody>
        {gf.map((f) => {
          const isChanged = f.path in overrides;
          return (
            <tr key={f.path} className={isChanged ? "is-changed" : undefined}>
              <th scope="row">
                {f.label}
                {f.help && <span className="setting-help">{f.help}</span>}
              </th>
              <td className="num muted">
                {show(f.baseline)}
                {f.baseline !== f.default && <span className="setting-help">from the show file</span>}
              </td>
              <td>
                {input(f)}
                {errors[f.path] && <span className="setting-help tone-bad">{errors[f.path]}</span>}
              </td>
              <td>
                {(isChanged || errors[f.path]) && (
                  <button className="link" onClick={() => revert(f.path)}>
                    Undo
                  </button>
                )}
              </td>
            </tr>
          );
        })}
      </tbody>
    </table>
  );

  return (
    <section className="planner" aria-label="Run Stage 2">
      {loadError && <p className="notice notice-bad">{loadError}</p>}
      {fields && (
        <details className="planner-settings" open={count > 0 || undefined}>
          <summary>
            Planner settings{" "}
            <span className="muted">{count ? `${count} changed from the defaults` : "all at their defaults"}</span>
          </summary>
          <p className="muted small planner-about">
            Defaults come from the planner's configuration file; motion limits can come from the show file. Only the
            settings you change are sent to Stage 2.
          </p>
          {groups.filter(([g]) => MAIN_GROUPS.has(g)).map(([group, gf]) => table(group, gf))}
          <details
            className="settings-advanced"
            open={groups.some(([g, gf]) => !MAIN_GROUPS.has(g) && gf.some((f) => f.path in overrides)) || undefined}
          >
            <summary>
              Solver tuning{" "}
              <span className="muted">
                ({groups.filter(([g]) => !MAIN_GROUPS.has(g)).reduce((n, [, gf]) => n + gf.length, 0)} settings)
              </span>
            </summary>
            {groups.filter(([g]) => !MAIN_GROUPS.has(g)).map(([group, gf]) => table(group, gf))}
          </details>
          <div className="actions">
            <button onClick={resetAll} disabled={!count && !invalid}>
              Reset all to defaults
            </button>
          </div>
          <details className="log">
            <summary>What Stage 2 receives</summary>
            <pre>{count ? JSON.stringify(nest(overrides), null, 2) : "{}  (no changes: the defaults apply)"}</pre>
          </details>
        </details>
      )}

      {warnings.length > 0 && (
        <div className="notice notice-bad planner-warnings" role="alert">
          <p>
            <strong>These settings make the safety check weaker than the defaults.</strong> A show that passes with
            them may not be safe to fly.
          </p>
          <ul>
            {warnings.map((w, i) => (
              <li key={i}>{w.message}</li>
            ))}
          </ul>
          <label className="check">
            <input type="checkbox" checked={accepted} onChange={(e) => setAccepted(e.target.checked)} /> Run with these
            settings anyway
          </label>
        </div>
      )}

      <div className="actions">
        <button
          className={primary ? "primary" : ""}
          disabled={!fields || invalid || (warnings.length > 0 && !accepted) || !!blocked}
          onClick={() => onRun(nest(overrides))}
        >
          {runLabel}
        </button>
        {run.stage2.status !== "not_run" && (
          <button onClick={copy} disabled={!fields || invalid || copying}>
            {copying ? "Copying…" : "Try these settings in a new run"}
          </button>
        )}
        {blocked && <span className="muted small">{blocked}</span>}
        {invalid && <span className="tone-bad small">Fix the highlighted settings first.</span>}
      </div>
      {copyError && <p className="notice notice-bad">{copyError}</p>}
      {run.stage2.status !== "not_run" && (
        <p className="muted small">
          Running again here replaces this run's result. A new run keeps it, so you can compare the two.
        </p>
      )}
    </section>
  );
}
