// Where Studio finds the repo, its Python and the run folders (docs/5-studio_gui.md §2.1), in a modal
// dialog over the current page.

import { useEffect, useRef, useState } from "react";
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
  const [draft, setDraft] = useState(settings);
  const [message, setMessage] = useState<{ tone: "ok" | "bad"; text: string } | null>(null);
  const ref = useRef<HTMLDialogElement>(null);

  useEffect(() => {
    const d = ref.current;
    if (d && !d.open) d.showModal();
    // Start in the first field rather than on the close button.
    d?.querySelector("input")?.focus();
  }, []);

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
    <dialog
      ref={ref}
      className="modal"
      aria-labelledby="settings-title"
      // Escape closes the dialog natively; keep React's state in step.
      onClose={onClose}
      onClick={(e) => e.target === e.currentTarget && ref.current?.close()}
    >
      <button className="icon-button modal-close" aria-label="Close settings" onClick={() => ref.current?.close()}>
        <svg width="18" height="18" viewBox="0 0 18 18" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" aria-hidden="true">
          <path d="M4 4l10 10M14 4L4 14" />
        </svg>
      </button>
      <header className="page-head">
        <h1 id="settings-title">Settings</h1>
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
              className="mono-input"
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
        <table className="table data checks-table">
          <tbody>
            {checks.map((c) => (
              <tr key={c.name}>
                <td>
                  <span className="check-name">
                    <span className={`light light-${c.ok ? "ok" : "bad"}`} aria-hidden="true" /> {c.name}
                  </span>
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
    </dialog>
  );
}
