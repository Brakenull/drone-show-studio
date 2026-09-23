// Where Studio finds the repo, its Python and the run folders (docs/5-studio_gui.md §2.1).

import { useState } from "react";
import { saveSettings } from "../bridge/api";
import type { DoctorCheck, Settings } from "../bridge/types";

interface Props {
  settings: Settings;
  checks: DoctorCheck[] | null;
  onSaved: (s: Settings) => void;
  onRecheck: () => void;
}

const FIELDS: { key: keyof Settings; label: string; help: string }[] = [
  { key: "repo_root", label: "Repository folder", help: "The folder that contains stage2_core_engine." },
  { key: "python", label: "Python", help: "The interpreter drone_core was built for (the repo's .venv)." },
  { key: "runs_dir", label: "Run folders", help: "Where each run's input, results and replay are kept." },
];

export function SettingsView({ settings, checks, onSaved, onRecheck }: Props) {
  const [draft, setDraft] = useState(settings);
  const [message, setMessage] = useState<{ tone: "ok" | "bad"; text: string } | null>(null);

  async function save() {
    try {
      const saved = await saveSettings(draft);
      onSaved(saved);
      setMessage({ tone: "ok", text: "Settings saved." });
    } catch (e) {
      setMessage({ tone: "bad", text: String(e) });
    }
  }

  return (
    <div className="page">
      <header className="page-head">
        <h1>Settings</h1>
      </header>
      <form
        className="form"
        onSubmit={(e) => {
          e.preventDefault();
          void save();
        }}
      >
        {FIELDS.map((f) => (
          <label key={f.key} className="field">
            <span className="field-label">{f.label}</span>
            <input
              value={draft[f.key]}
              spellCheck={false}
              onChange={(e) => setDraft({ ...draft, [f.key]: e.target.value })}
            />
            <span className="muted small">{f.help}</span>
          </label>
        ))}
        <div className="actions">
          <button className="primary" type="submit">
            Save settings
          </button>
          {message && <span className={`tone-${message.tone}`}>{message.text}</span>}
        </div>
      </form>

      <h2 className="section-title">Components</h2>
      <p className="muted">What Studio found when it started. Missing pieces disable the steps that need them.</p>
      {checks ? (
        <table className="table">
          <tbody>
            {checks.map((c) => (
              <tr key={c.name}>
                <td>
                  <span className={`light light-${c.ok ? "ok" : "bad"}`} aria-hidden="true" /> {c.name}
                </td>
                <td className="muted">{c.required_for}</td>
                <td className="path">{c.ok ? c.detail : <span className="tone-bad">{c.detail}</span>}</td>
              </tr>
            ))}
          </tbody>
        </table>
      ) : (
        <p className="status-line">Checking…</p>
      )}
      <div className="actions">
        <button onClick={onRecheck}>Check again</button>
      </div>
    </div>
  );
}
