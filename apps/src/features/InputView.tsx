// The run's own copy of the Phase 1 file, re-validated on open.

import { useEffect, useState } from "react";
import { openRunFolder, runJob } from "../bridge/api";
import type { RunRecord, Validation } from "../bridge/types";
import { ValidationReport } from "./ValidationReport";

const cache = new Map<string, Validation>();

export function InputView({ run }: { run: RunRecord }) {
  const [validation, setValidation] = useState<Validation | null>(cache.get(run.run_id) ?? null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (cache.has(run.run_id)) {
      setValidation(cache.get(run.run_id)!);
      return;
    }
    setValidation(null);
    setError(null);
    const inputPath = `${run.run_dir}/input/phase1.json`;
    runJob(["validate", inputPath])
      .then(({ events }) => {
        const v = events.find((e) => e.type === "validation");
        if (v && v.type === "validation") {
          cache.set(run.run_id, v);
          setValidation(v);
        } else setError("Validation produced no result.");
      })
      .catch((e) => setError(String(e)));
  }, [run.run_id, run.run_dir]);

  return (
    <div className="page">
      <header className="page-head">
        <h1>Input</h1>
        <p className="lede">
          The Phase 1 export this run uses. Source: <span className="path">{run.input.source_path}</span>
        </p>
      </header>
      {error && <p className="notice notice-bad">{error}</p>}
      {!validation && !error && <p className="status-line">Checking the file…</p>}
      {validation && <ValidationReport validation={validation} />}
      <div className="actions">
        <button onClick={() => openRunFolder(run.run_id)}>Open run folder</button>
      </div>
    </div>
  );
}
