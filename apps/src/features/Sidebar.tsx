import type { RunRecord } from "../bridge/types";
import { STATUS, runCreated, runName } from "../app/format";
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
  return (
    <nav className="sidebar" aria-label="Runs">
      <div className="sidebar-head">
        <div className="sidebar-title">
          <span>Drone Show Studio</span>
          <h2>Runs</h2>
        </div>
        <span className="sidebar-tools">
          <span className="count-pill">{runs.length}</span>
          <button
            className={`icon-button settings-button ${settingsOpen ? "is-current" : ""}`}
            onClick={onSettings}
            title="Settings"
            aria-label="Settings"
          >
            <svg width="20" height="20" viewBox="0 0 20 20" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" aria-hidden="true">
              <path d="M3 6h14M3 14h14" />
              <circle cx="13" cy="6" r="2.2" fill="var(--surface)" />
              <circle cx="7" cy="14" r="2.2" fill="var(--surface)" />
            </svg>
          </button>
        </span>
      </div>
      {runs.length === 0 ? (
        <p className="sidebar-empty">Runs you create appear here, newest first.</p>
      ) : (
        <ul className="run-list">
          {runs.map((run) => {
            const status = STATUS[run.stage2.status] ?? STATUS.not_run;
            const s2 = stage2State(run);
            const current = view === "run" && run.run_id === selectedRunId;
            const pipe = [
              ["Stage 2", s2],
              ["Stress test", stressState(run)],
              ["Flight files", packState(run)],
            ] as const;
            return (
              <li key={run.run_id}>
                <button
                  className={`run-card ${current ? "is-current" : ""}`}
                  aria-current={current ? "page" : undefined}
                  onClick={() => onSelect(run.run_id)}
                >
                  <span className="run-card-top">
                    <span className={`light light-${s2.tone}`} aria-hidden="true" />
                    <span className="run-name">{runName(run)}</span>
                    <span className={`pill pill-${s2.tone}`}>{s2.tone === "busy" ? "Running" : status.label}</span>
                  </span>
                  <span className="run-meta">
                    {runCreated(run)} · {run.input.fleet_size} drones
                    {run.copied_from && " · copy"}
                  </span>
                  <span className="run-pipe" aria-hidden="true">
                    {pipe.map(([name, st]) => (
                      <span key={name} className={`pipe-${st.tone}`} title={`${name}: ${st.text}`} />
                    ))}
                  </span>
                </button>
              </li>
            );
          })}
        </ul>
      )}
      <button className={`fab ${view === "new" ? "is-current" : ""}`} onClick={onNew}>
        <svg width="20" height="20" viewBox="0 0 20 20" fill="none" stroke="currentColor" strokeWidth="2.4" strokeLinecap="round" aria-hidden="true">
          <path d="M10 4v12M4 10h12" />
        </svg>
        New run
      </button>
    </nav>
  );
}
