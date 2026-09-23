// Pick a Phase 1 export, validate it, create a run folder (docs/5-studio_gui.md §6.1).

import { useEffect, useRef, useState } from "react";
import { getCurrentWebview } from "@tauri-apps/api/webview";
import { runJob, stashImport } from "../bridge/api";
import type { Validation } from "../bridge/types";
import { ValidationReport } from "./ValidationReport";

interface Props {
  runsDir: string;
  onCreated: (runId: string) => void;
}

const baseName = (p: string) => p.split(/[\\/]/).pop() ?? p;

export function NewRun({ runsDir, onCreated }: Props) {
  const [path, setPath] = useState<string | null>(null);
  const [validation, setValidation] = useState<Validation | null>(null);
  const [busy, setBusy] = useState<"validating" | "creating" | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [dragOver, setDragOver] = useState(false);
  const fileInput = useRef<HTMLInputElement>(null);

  async function check(filePath: string) {
    setPath(filePath);
    setValidation(null);
    setError(null);
    setBusy("validating");
    try {
      const { events } = await runJob(["validate", filePath]);
      const v = events.find((e) => e.type === "validation");
      const err = events.find((e) => e.type === "error");
      if (v && v.type === "validation") setValidation(v);
      else setError(err && err.type === "error" ? err.message : "Validation produced no result.");
    } catch (e) {
      setError(String(e));
    } finally {
      setBusy(null);
    }
  }

  // Native drag-and-drop gives the real path (the HTML file input only gives contents).
  const checkRef = useRef(check);
  checkRef.current = check;
  useEffect(() => {
    let unlisten: (() => void) | undefined;
    getCurrentWebview()
      .onDragDropEvent((event) => {
        const p = event.payload;
        if (p.type === "over" || p.type === "enter") setDragOver(true);
        else if (p.type === "leave") setDragOver(false);
        else if (p.type === "drop") {
          setDragOver(false);
          const file = p.paths.find((f) => f.toLowerCase().endsWith(".json"));
          if (file) void checkRef.current(file);
          else setError("Drop a .json file exported from the Blender add-on.");
        }
      })
      .then((fn) => (unlisten = fn));
    return () => unlisten?.();
  }, []);

  async function onFileChosen(e: React.ChangeEvent<HTMLInputElement>) {
    const file = e.target.files?.[0];
    e.target.value = "";
    if (!file) return;
    try {
      const stashed = await stashImport(file.name, await file.text());
      await check(stashed);
    } catch (err) {
      setError(String(err));
    }
  }

  async function create() {
    if (!path) return;
    setBusy("creating");
    setError(null);
    try {
      const { events } = await runJob(["new-run", path, "--runs-dir", runsDir]);
      const created = events.find((e) => e.type === "run_created");
      if (created && created.type === "run_created") onCreated(created.run_id);
      else {
        const v = events.find((e) => e.type === "validation");
        if (v && v.type === "validation") setValidation(v);
        const err = events.find((e) => e.type === "error");
        setError(err && err.type === "error" ? err.message : "The run was not created.");
      }
    } catch (e) {
      setError(String(e));
    } finally {
      setBusy(null);
    }
  }

  return (
    <div className="page">
      <header className="page-head">
        <h1>New run</h1>
        <p className="lede">
          Start from a Phase 1 export. Studio checks the file, then keeps its own copy in a run folder so later
          changes to the original don't affect results.
        </p>
      </header>

      <div
        className={`dropzone ${dragOver ? "is-over" : ""}`}
        onClick={() => fileInput.current?.click()}
        role="button"
        tabIndex={0}
        onKeyDown={(e) => (e.key === "Enter" || e.key === " ") && fileInput.current?.click()}
      >
        <svg className="dropzone-art" viewBox="0 0 120 48" aria-hidden="true">
          {Array.from({ length: 5 }, (_, r) =>
            Array.from({ length: 12 }, (_, c) => (
              <circle key={`${r}-${c}`} cx={6 + c * 10} cy={6 + r * 9} r={(r + c) % 4 === 0 ? 2.2 : 1.4} />
            )),
          )}
        </svg>
        <p className="dropzone-title">{path ? baseName(path) : "Drop a Phase 1 JSON file here"}</p>
        <p className="muted">{path ? "Drop another file or click to choose a different one." : "or click to choose one"}</p>
        <input ref={fileInput} type="file" accept=".json,application/json" hidden onChange={onFileChosen} />
      </div>

      {busy === "validating" && <p className="status-line">Checking the file…</p>}
      {error && (
        <p className="notice notice-bad" role="alert">
          {error}
        </p>
      )}

      {validation && (
        <>
          <ValidationReport validation={validation} />
          <div className="actions">
            <button className="primary" disabled={!validation.ok || busy !== null} onClick={create}>
              {busy === "creating" ? "Creating run…" : "Create run"}
            </button>
            {!validation.ok && <span className="muted">Fix the problems above, then export again.</span>}
          </div>
        </>
      )}
    </div>
  );
}
