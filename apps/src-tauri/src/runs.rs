//! Run folders (docs/5-studio_gui.md §3): list them, read their files, repair stale "running" records.

use std::fs;
use std::path::{Component, Path, PathBuf};

use serde_json::Value;
use tauri::{AppHandle, Manager, State};

use crate::bridge::Jobs;
use crate::settings;

const RUN_FILE: &str = "run.json";

/// The run.json sections a job can own: (name the UI passes, JSON pointer, log file in the run folder).
const SECTIONS: [(&str, &str, &str); 5] = [
    ("stage2", "/stage2", "stage2/log.ndjson"),
    ("stage2_returns", "/stage2_returns", "stage2/returns/log.ndjson"),
    ("monte_carlo", "/stage3/monte_carlo", "stage3/monte_carlo_log.ndjson"),
    ("pack", "/stage3/pack", "stage3/pack_log.ndjson"),
    ("simulate", "/conditions/simulate", "stage3/scenarios/simulate_log.ndjson"),
];

/// (JSON pointer, log file) for a section name.
pub fn section(name: &str) -> Result<(&'static str, &'static str), String> {
    SECTIONS
        .iter()
        .find(|(n, _, _)| *n == name)
        .map(|(_, pointer, log)| (*pointer, *log))
        .ok_or_else(|| format!("unknown run section {name}"))
}

/// UTC ISO-8601 (the bridge writes local-offset ISO; both parse in JS). Avoids a date-time crate.
fn now_iso() -> String {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs() as i64)
        .unwrap_or(0);
    let (days, rem) = (secs.div_euclid(86_400), secs.rem_euclid(86_400));
    // Civil-from-days (Howard Hinnant's algorithm).
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!(
        "{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}Z",
        rem / 3_600,
        rem % 3_600 / 60,
        rem % 60
    )
}

fn write_json_atomic(path: &Path, value: &Value) -> Result<(), String> {
    let tmp = path.with_extension("json.tmp");
    fs::write(&tmp, serde_json::to_string_pretty(value).map_err(|e| e.to_string())?)
        .map_err(|e| e.to_string())?;
    fs::rename(&tmp, path).map_err(|e| e.to_string())
}

/// Rewrite the section at `pointer` (e.g. "/stage3/pack") from "running" to "cancelled".
pub fn mark_cancelled(run_dir: &Path, pointer: &str) -> Result<(), String> {
    let path = run_dir.join(RUN_FILE);
    let mut record: Value =
        serde_json::from_str(&fs::read_to_string(&path).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
    if let Some(part) = record.pointer_mut(pointer).and_then(Value::as_object_mut) {
        if part.get("status").and_then(Value::as_str) == Some("running") {
            part.insert("status".into(), "cancelled".into());
            part.insert("ended_at".into(), now_iso().into());
            part.insert("pid".into(), Value::Null);
            part.insert("message".into(), "cancelled".into());
        }
    }
    write_json_atomic(&path, &record)
}

fn runs_dir(app: &AppHandle) -> PathBuf {
    PathBuf::from(settings::load(app).runs_dir)
}

/// Every run record, newest first. A "running" record with no live bridge process is a job that died
/// with the app (or was killed outside it) and is rewritten to "cancelled".
#[tauri::command]
pub fn list_runs(app: AppHandle, jobs: State<'_, Jobs>) -> Result<Vec<Value>, String> {
    let dir = runs_dir(&app);
    if !dir.is_dir() {
        return Ok(vec![]);
    }
    let busy: Vec<PathBuf> = jobs.busy_run_dirs().iter().filter_map(|p| p.canonicalize().ok()).collect();
    let mut out = vec![];
    for entry in fs::read_dir(&dir).map_err(|e| e.to_string())?.flatten() {
        let run_dir = entry.path();
        let Ok(text) = fs::read_to_string(run_dir.join(RUN_FILE)) else { continue };
        let Ok(mut record) = serde_json::from_str::<Value>(&text) else { continue };
        let attached = run_dir.canonicalize().map(|p| busy.contains(&p)).unwrap_or(false);
        let stale: Vec<&str> = SECTIONS
            .iter()
            .map(|(_, pointer, _)| *pointer)
            .filter(|p| record.pointer(&format!("{p}/status")).and_then(Value::as_str) == Some("running"))
            .collect();
        if !attached && !stale.is_empty() {
            for pointer in stale {
                mark_cancelled(&run_dir, pointer)?;
            }
            record = serde_json::from_str(&fs::read_to_string(run_dir.join(RUN_FILE)).map_err(|e| e.to_string())?)
                .map_err(|e| e.to_string())?;
        }
        record["run_dir"] = run_dir.display().to_string().into();
        out.push(record);
    }
    out.sort_by(|a, b| b["run_id"].as_str().cmp(&a["run_id"].as_str()));
    Ok(out)
}

/// `rel` must stay inside the run folder: no absolute paths, no "..".
fn run_file(app: &AppHandle, run_id: &str, rel: &str) -> Result<PathBuf, String> {
    let rel_path = Path::new(rel);
    let safe = |p: &Path| p.components().all(|c| matches!(c, Component::Normal(_)));
    if !safe(Path::new(run_id)) || !safe(rel_path) {
        return Err(format!("invalid path {run_id}/{rel}"));
    }
    Ok(runs_dir(app).join(run_id).join(rel_path))
}

#[tauri::command]
pub fn read_run_text(app: AppHandle, run_id: String, rel: String) -> Result<Option<String>, String> {
    let path = run_file(&app, &run_id, &rel)?;
    if !path.exists() {
        return Ok(None);
    }
    fs::read_to_string(&path).map(Some).map_err(|e| format!("{}: {e}", path.display()))
}

/// Raw bytes (replay arrays) without JSON encoding; the UI receives an ArrayBuffer.
#[tauri::command]
pub fn read_run_bytes(app: AppHandle, run_id: String, rel: String) -> Result<tauri::ipc::Response, String> {
    let path = run_file(&app, &run_id, &rel)?;
    let bytes = fs::read(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    Ok(tauri::ipc::Response::new(bytes))
}

/// A file picked through the HTML file input has no path; keep a copy so the bridge can read it.
#[tauri::command]
pub fn stash_import(app: AppHandle, name: String, text: String) -> Result<String, String> {
    let file_name = Path::new(&name)
        .file_name()
        .and_then(|n| n.to_str())
        .filter(|n| !n.is_empty())
        .unwrap_or("show.json")
        .to_string();
    let stamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis())
        .unwrap_or(0);
    let dir = app.path().app_cache_dir().map_err(|e| e.to_string())?.join("imports").join(stamp.to_string());
    fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
    let path = dir.join(file_name);
    fs::write(&path, text).map_err(|e| e.to_string())?;
    Ok(path.display().to_string())
}

/// Opens the run folder, or a folder inside it (`rel`, e.g. "stage3/bin").
#[tauri::command]
pub fn open_run_folder(app: AppHandle, run_id: String, rel: Option<String>) -> Result<(), String> {
    let path = run_file(&app, &run_id, rel.as_deref().filter(|r| !r.is_empty()).unwrap_or("."))?;
    #[cfg(windows)]
    std::process::Command::new("explorer").arg(&path).spawn().map_err(|e| e.to_string())?;
    #[cfg(not(windows))]
    let _ = path;
    Ok(())
}
