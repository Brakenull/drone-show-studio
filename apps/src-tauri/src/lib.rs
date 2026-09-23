//! Drone Show Studio desktop shell (docs/5-studio_gui.md §2): no domain logic lives here.

mod bridge;
mod runs;
mod settings;

use tauri::Manager;

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    tauri::Builder::default()
        .plugin(tauri_plugin_opener::init())
        .manage(bridge::Jobs::default())
        .invoke_handler(tauri::generate_handler![
            settings::get_settings,
            settings::save_settings,
            bridge::start_job,
            bridge::cancel_job,
            runs::list_runs,
            runs::read_run_text,
            runs::read_run_bytes,
            runs::stash_import,
            runs::open_run_folder,
        ])
        .build(tauri::generate_context!())
        .expect("error while building tauri application")
        .run(|app, event| {
            if let tauri::RunEvent::Exit = event {
                app.state::<bridge::Jobs>().kill_all();
            }
        });
}
