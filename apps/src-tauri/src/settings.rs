//! App settings: where the repo, its Python and the run folders are.

use std::fs;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};
use tauri::{AppHandle, Manager};

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Settings {
    pub repo_root: String,
    pub python: String,
    pub runs_dir: String,
}

fn is_repo_root(dir: &Path) -> bool {
    dir.join("stage2_core_engine").is_dir() && dir.join("tools").is_dir()
}

/// Walk up from the executable (dev builds live in src-tauri/target/...) and the working directory.
fn detect_repo_root() -> Option<PathBuf> {
    let starts = [std::env::current_exe().ok(), std::env::current_dir().ok()];
    starts
        .into_iter()
        .flatten()
        .flat_map(|start| start.ancestors().map(Path::to_path_buf).collect::<Vec<_>>())
        .find(|dir| is_repo_root(dir))
}

fn defaults_for(repo_root: &Path) -> Settings {
    Settings {
        repo_root: repo_root.display().to_string(),
        python: repo_root.join(".venv").join("Scripts").join("python.exe").display().to_string(),
        runs_dir: repo_root.join("runs").display().to_string(),
    }
}

fn settings_path(app: &AppHandle) -> Result<PathBuf, String> {
    let dir = app.path().app_config_dir().map_err(|e| e.to_string())?;
    Ok(dir.join("settings.json"))
}

pub fn load(app: &AppHandle) -> Settings {
    let saved = settings_path(app)
        .ok()
        .and_then(|p| fs::read_to_string(p).ok())
        .and_then(|text| serde_json::from_str::<Settings>(&text).ok());
    match (saved, detect_repo_root()) {
        (Some(s), _) if is_repo_root(Path::new(&s.repo_root)) => s,
        (_, Some(root)) => defaults_for(&root),
        (Some(s), None) => s,
        (None, None) => Settings::default(),
    }
}

#[tauri::command]
pub fn get_settings(app: AppHandle) -> Settings {
    load(&app)
}

#[tauri::command]
pub fn save_settings(app: AppHandle, settings: Settings) -> Result<Settings, String> {
    if !is_repo_root(Path::new(&settings.repo_root)) {
        return Err(format!("{} does not contain stage2_core_engine/ and tools/", settings.repo_root));
    }
    let path = settings_path(&app)?;
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    let text = serde_json::to_string_pretty(&settings).map_err(|e| e.to_string())?;
    fs::write(&path, text).map_err(|e| e.to_string())?;
    Ok(settings)
}
