//! Bridge processes (docs/5-studio_gui.md §2, §4): spawn `python -m tools.studio_bridge`,
//! forward its NDJSON stdout as Tauri events, and cancel by killing the process tree.

use std::collections::HashMap;
use std::fs::OpenOptions;
use std::io::{BufRead, BufReader, Write};
use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;

use serde::Serialize;
use serde_json::Value;
use tauri::{AppHandle, Emitter, State};

use crate::{runs, settings};

#[cfg(windows)]
use std::os::windows::process::CommandExt;
#[cfg(windows)]
const CREATE_NO_WINDOW: u32 = 0x0800_0000;

struct Job {
    pid: u32,
    run_dir: Option<PathBuf>,
    /// The run.json section this job owns (JSON pointer from runs::section); "" without a run_dir.
    pointer: &'static str,
    cancelled: bool,
}

#[derive(Default)]
pub struct Jobs {
    next_id: AtomicU64,
    live: Arc<Mutex<HashMap<u64, Job>>>,
}

impl Jobs {
    /// Run folders that have a bridge process attached right now.
    pub fn busy_run_dirs(&self) -> Vec<PathBuf> {
        self.live.lock().unwrap().values().filter_map(|j| j.run_dir.clone()).collect()
    }

    /// On app exit: Windows doesn't end child processes with their parent, so a solve would keep
    /// running unseen. Kill every job and mark its run cancelled.
    pub fn kill_all(&self) {
        let jobs: Vec<(u32, Option<PathBuf>, &str)> = self
            .live
            .lock()
            .unwrap()
            .values_mut()
            .map(|j| {
                j.cancelled = true;
                (j.pid, j.run_dir.clone(), j.pointer)
            })
            .collect();
        for (pid, run_dir, pointer) in jobs {
            let _ = kill_tree(pid);
            if let Some(dir) = run_dir {
                let _ = runs::mark_cancelled(&dir, pointer);
            }
        }
    }
}

fn kill_tree(pid: u32) -> Result<(), String> {
    #[cfg(windows)]
    {
        let status = Command::new("taskkill")
            .args(["/PID", &pid.to_string(), "/T", "/F"])
            .creation_flags(CREATE_NO_WINDOW)
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .map_err(|e| e.to_string())?;
        if !status.success() {
            return Err(format!("taskkill failed for pid {pid}"));
        }
    }
    #[cfg(not(windows))]
    {
        Command::new("kill").args(["-9", &pid.to_string()]).status().map_err(|e| e.to_string())?;
    }
    Ok(())
}

#[derive(Clone, Serialize)]
struct BridgeEvent {
    job_id: u64,
    event: Value,
}

#[derive(Clone, Serialize)]
struct BridgeLog {
    job_id: u64,
    line: String,
}

#[derive(Clone, Serialize)]
struct BridgeExit {
    job_id: u64,
    code: Option<i32>,
    cancelled: bool,
}

fn append_log(path: &Option<PathBuf>, entry: &Value) {
    if let Some(path) = path {
        if let Ok(mut file) = OpenOptions::new().create(true).append(true).open(path) {
            let _ = writeln!(file, "{entry}");
        }
    }
}

/// `args` are the bridge's own arguments (e.g. ["stage2", "<run_dir>"]). `run_dir` ties the job to a
/// run folder and `section` ("stage2", "stage2_returns", "monte_carlo" or "pack") to one part of its run.json: events are
/// appended to that section's log file, and a cancel marks that section cancelled.
#[tauri::command]
pub fn start_job(
    app: AppHandle,
    jobs: State<'_, Jobs>,
    args: Vec<String>,
    run_dir: Option<String>,
    section: Option<String>,
) -> Result<u64, String> {
    let cfg = settings::load(&app);
    if cfg.repo_root.is_empty() {
        return Err("repo root is not set".into());
    }
    let run_dir = run_dir.map(PathBuf::from);
    let (pointer, log_rel) = match (&run_dir, section.as_deref()) {
        (Some(_), Some(name)) => runs::section(name)?,
        (Some(_), None) => return Err("a job tied to a run folder needs a section".into()),
        (None, _) => ("", ""),
    };
    let log_path = run_dir.as_ref().map(|d| d.join(log_rel));
    if let Some(p) = &log_path {
        let _ = std::fs::create_dir_all(p.parent().unwrap());
    }

    let mut cmd = Command::new(&cfg.python);
    cmd.args(["-u", "-m", "tools.studio_bridge"])
        .args(&args)
        .current_dir(&cfg.repo_root)
        .env("PYTHONIOENCODING", "utf-8")
        .env("PYTHONUNBUFFERED", "1")
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    #[cfg(windows)]
    cmd.creation_flags(CREATE_NO_WINDOW);
    let mut child = cmd.spawn().map_err(|e| format!("cannot start {}: {e}", cfg.python))?;

    let job_id = jobs.next_id.fetch_add(1, Ordering::SeqCst) + 1;
    jobs.live.lock().unwrap().insert(
        job_id,
        Job { pid: child.id(), run_dir: run_dir.clone(), pointer, cancelled: false },
    );

    let stdout = child.stdout.take().unwrap();
    let stderr = child.stderr.take().unwrap();

    let (app_out, log_out) = (app.clone(), log_path.clone());
    let out_thread = thread::spawn(move || {
        for line in BufReader::new(stdout).lines().map_while(Result::ok) {
            let event = serde_json::from_str::<Value>(&line)
                .unwrap_or_else(|_| serde_json::json!({ "type": "stdout", "line": line }));
            append_log(&log_out, &event);
            let _ = app_out.emit("bridge-event", BridgeEvent { job_id, event });
        }
    });
    let (app_err, log_err) = (app.clone(), log_path);
    let err_thread = thread::spawn(move || {
        for line in BufReader::new(stderr).lines().map_while(Result::ok) {
            append_log(&log_err, &serde_json::json!({ "type": "stderr", "line": line }));
            let _ = app_err.emit("bridge-log", BridgeLog { job_id, line });
        }
    });

    let live = jobs.live.clone();
    thread::spawn(move || {
        let status = child.wait();
        let _ = out_thread.join();
        let _ = err_thread.join();
        let job = live.lock().unwrap().remove(&job_id);
        let cancelled = job.as_ref().map(|j| j.cancelled).unwrap_or(false);
        if let Some(Job { run_dir: Some(dir), pointer, cancelled: true, .. }) = job {
            let _ = runs::mark_cancelled(&dir, pointer);
        }
        let code = status.ok().and_then(|s| s.code());
        let _ = app.emit("bridge-exit", BridgeExit { job_id, code, cancelled });
    });

    Ok(job_id)
}

/// Kill the whole tree: the .venv python.exe is a launcher whose child is the real interpreter.
#[tauri::command]
pub fn cancel_job(jobs: State<'_, Jobs>, job_id: u64) -> Result<(), String> {
    let pid = {
        let mut live = jobs.live.lock().unwrap();
        let job = live.get_mut(&job_id).ok_or("job is not running")?;
        job.cancelled = true;
        job.pid
    };
    kill_tree(pid)
}
