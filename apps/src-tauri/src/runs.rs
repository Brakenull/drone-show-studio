//! Run folders (docs/5-studio_gui.md §3): list them, read their files, repair stale "running" records.

use std::fs;
use std::path::{Component, Path, PathBuf};

use serde_json::Value;
use tauri::{AppHandle, Manager, State};

use crate::bridge::Jobs;
use crate::settings;

const RUN_FILE: &str = "run.json";

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

pub fn mark_stage2_cancelled(run_dir: &Path) -> Result<(), String> {
    let path = run_dir.join(RUN_FILE);
    let mut record: Value =
        serde_json::from_str(&fs::read_to_string(&path).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
    if let Some(stage2) = record.get_mut("stage2").and_then(Value::as_object_mut) {
        if stage2.get("status").and_then(Value::as_str) == Some("running") {
            stage2.insert("status".into(), "cancelled".into());
            stage2.insert("ended_at".into(), now_iso().into());
            stage2.insert("pid".into(), Value::Null);
            stage2.insert("message".into(), "cancelled".into());
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
        let running = record.pointer("/stage2/status").and_then(Value::as_str) == Some("running");
        let attached = run_dir.canonicalize().map(|p| busy.contains(&p)).unwrap_or(false);
        if running && !attached {
            mark_stage2_cancelled(&run_dir)?;
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

#[tauri::command]
pub fn open_run_folder(app: AppHandle, run_id: String) -> Result<(), String> {
    let path = run_file(&app, &run_id, ".")?;
    #[cfg(windows)]
    std::process::Command::new("explorer").arg(&path).spawn().map_err(|e| e.to_string())?;
    #[cfg(not(windows))]
    let _ = path;
    Ok(())
}
