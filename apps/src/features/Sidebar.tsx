import type { RunRecord } from "../bridge/types";
import { STATUS, runCreated, runName } from "../app/format";

interface Props {
  runs: RunRecord[];
  selectedRunId: string | null;
  view: "new" | "run" | "settings";
  onNew: () => void;
  onSelect: (runId: string) => void;
  onSettings: () => void;
}

export function Sidebar({ runs, selectedRunId, view, onNew, onSelect, onSettings }: Props) {
  return (
    <nav className="sidebar" aria-label="Runs">
      <div className="brand">
        <span className="brand-mark" aria-hidden="true" />
        <span>Drone Show Studio</span>
      </div>
      <button className={`new-run ${view === "new" ? "is-current" : ""}`} onClick={onNew}>
        New run
      </button>
      {runs.length === 0 ? (
        <p className="sidebar-empty">Runs you create appear here, newest first.</p>
      ) : (
        <ul className="run-list">
          {runs.map((run) => {
            const status = STATUS[run.stage2.status] ?? STATUS.not_run;
            const current = view === "run" && run.run_id === selectedRunId;
            return (
              <li key={run.run_id}>
                <button
                  className={`run-item ${current ? "is-current" : ""}`}
                  aria-current={current ? "page" : undefined}
                  onClick={() => onSelect(run.run_id)}
                >
                  <span className={`light light-${status.tone}`} aria-hidden="true" />
                  <span className="run-name">{runName(run)}</span>
                  <span className="run-meta">
                    {runCreated(run)}, {run.input.fleet_size} drones
                  </span>
                  <span className={`run-status tone-${status.tone}`}>{status.label}</span>
                </button>
              </li>
            );
          })}
        </ul>
      )}
      <button className={`settings-link ${view === "settings" ? "is-current" : ""}`} onClick={onSettings}>
        Settings
      </button>
    </nav>
  );
}
