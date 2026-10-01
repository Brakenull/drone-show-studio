// Shapes of the bridge's NDJSON events and run files (docs/5-studio_gui.md §3, §4).

export type Vec3 = [number, number, number];

export type RunStatus =
  | "not_run"
  | "running"
  | "succeeded"
  | "failed_safety"
  | "failed_input"
  | "failed_error"
  | "cancelled";

export interface Settings {
  repo_root: string;
  python: string;
  runs_dir: string;
}

export interface Transition {
  index: number;
  from_keyframe: string;
  to_keyframe: string;
  start_time_sec: number;
  duration_sec: number;
}

/** run.json's stage3.monte_carlo / stage3.pack (tools/studio_bridge/stage3_job.py). */
export interface Stage3Part {
  status: RunStatus;
  started_at?: string;
  ended_at?: string | null;
  message?: string | null;
  wall_time_sec?: number | null;
  /** Stage 2's ended_at when this ran; differs from stage2.ended_at once Stage 2 was run again. */
  stage2_ended_at?: string | null;
}

export interface MonteCarloPart extends Stage3Part {
  /** `batch` since the OpenCL twin; `workers` in runs made before it (one process per CPU core). */
  config?: { runs: number; device: string; seed: number; batch?: number; workers?: number };
  passed?: boolean;
  crash_rate?: number;
  worst_min_separation_m?: number | null;
  worst_final_soc?: number | null;
}

export interface PackSummary {
  fleet_size: number;
  files: number;
  verified_files: number;
  records_per_file: number;
  file_size_bytes: number;
  sampling_dt_ms: number;
  total_bytes: number;
  verify_message: string;
}

export type PackPart = Stage3Part & Partial<PackSummary>;

export interface RunRecord {
  run_id: string;
  run_dir: string;
  created_at: string;
  input: { source_path: string; sha256: string; fleet_size: number; keyframes: string[] };
  stage2: {
    status: RunStatus;
    started_at?: string;
    ended_at?: string | null;
    message?: string | null;
    wall_time_sec?: number | null;
    total_duration_sec?: number;
    worst_separation_m?: number;
    required_separation_m?: number;
    transition?: Transition;
    /** Planner settings of the last Stage 2 run that made it less safe than its baseline. */
    config_warnings?: ConfigWarning[];
  };
  /** Set on a run made with "Copy to a new run": the run it copies. */
  copied_from?: string;
  stage3?: { monte_carlo?: MonteCarloPart; pack?: PackPart };
  /** Return paths from abort points (tools/studio_bridge/returns_job.py, docs/4-condition_simulator.md B4). */
  stage2_returns?: Stage3Part & { formations?: number[]; planned?: number[] };
  /** Condition simulator (tools/studio_bridge/conditions_job.py): the `simulate` job and each scenario's last outcome. */
  conditions?: {
    simulate?: Stage3Part & { scenario?: string };
    scenarios?: Record<string, Stage3Part & { passed?: boolean }>;
  };
}

// ---- Condition simulator (docs/4-condition_simulator.md §3, §6) ----

export type RtkState = "fixed" | "float" | "gps";
export interface WindKey {
  t: number;
  speed_mps: number;
  from_deg: number;
  turbulence: number;
}
export interface GustKey {
  t: number;
  peak_mps: number;
  duration_s: number;
  from_deg: number;
}
export interface RtkKey {
  t: number;
  state: RtkState;
}
export interface RainKey {
  t: number;
  mm_h: number;
}
export interface RainRule {
  alert_mm_h: number;
  limit_mm_h: number;
  reaction_s: number;
  margin_s: number;
}
/** stage3/scenarios/<id>/scenario.json */
export interface Scenario {
  name: string;
  seed: number;
  wind: WindKey[];
  gusts: GustKey[];
  rtk: RtkKey[];
  rain: RainKey[];
  rain_rule: RainRule;
}

/** stage3/scenarios/<id>/result.json (twin_sim/scenario_runner.py). */
export interface ScenarioResult {
  scenario: Scenario;
  device: string;
  fleet_size: number;
  show_duration_sec: number;
  sim_duration_sec: number;
  wall_time_sec: number;
  realtime_factor: number;
  criteria: { d_crash_m: number; d_warning_m: number; min_landing_soc: number };
  passed: boolean;
  closest: { distance_m: number; a: number; b: number; time_sec: number } | null;
  largest_deviation: { distance_m: number; drone: number; time_sec: number };
  lowest_battery: { soc: number; drone: number };
  crash_pairs: McPair[];
  warning_pairs: McPair[];
  low_soc_drones: number[];
  brownout_drones: number[];
  min_voltage_v: number;
  rain: {
    alert_mm_h: number;
    limit_mm_h: number;
    alert_time_sec: number | null;
    limit_time_sec: number | null;
    peak_mm_h: number;
  };
  stage2_ended_at: string | null;
  simulated_at: string;
}

export interface ScenarioEntry {
  id: string;
  scenario: Scenario;
  result: ScenarioResult | null;
  playback: boolean;
}

export interface ShowTransition {
  index: number;
  from_keyframe: string;
  to_keyframe: string;
  start_time_sec: number;
  end_time_sec: number;
}

export interface ConditionsInfo {
  show: {
    duration_sec: number;
    keyframes: string[];
    transitions: ShowTransition[];
    legs: { takeoff?: { start_time_sec: number; end_time_sec: number } | null; return?: { start_time_sec: number; end_time_sec: number } | null };
  };
  scenarios: ScenarioEntry[];
  defaults: { rain_rule: RainRule; rtk_states: RtkState[]; limits: Record<string, [number, number]> };
}

/** One formation's return path in stage2/returns/index.json. */
export interface ReturnEntry {
  keyframe_index: number;
  from_keyframe: string;
  /** "reversed_takeoff": the first formation with staggered takeoff on, the takeoff flown backwards. */
  method?: "planned" | "reversed_takeoff";
  status: RunStatus;
  message: string | null;
  abort_time_sec: number;
  target_duration_sec: number | null;
  planned_duration_sec: number | null;
  flown_duration_sec: number | null;
  attempts: number;
  worst_separation_m: number | null;
  required_separation_m?: number;
  wall_time_sec: number;
  /** stage2/returns/replay_<k>/ was built (the show up to the formation, then the flight home). */
  replay?: boolean;
}

export interface ReturnIndex {
  /** Stage 2's ended_at the returns were planned from. */
  stage2_ended_at: string | null;
  returns: ReturnEntry[];
}

/** One Stage 2 planner setting (tools/studio_bridge/config_fields.py). `path` is dotted, e.g.
 *  "solver.continuous_gatekeeper.min_allowable_distance_m". */
export interface ConfigField {
  path: string;
  label: string;
  help: string;
  kind: "number" | "integer" | "boolean" | "choice";
  group: string;
  /** The direction that makes the setting less safe. */
  risky: "lower" | "higher" | "off" | null;
  min: number | null;
  choices: string[] | null;
  default: number | boolean | string;
  /** What this run uses without an override: default, or the Phase 1 file's motion limits. */
  baseline: number | boolean | string;
}

export interface ConfigWarning {
  path: string;
  label: string;
  message: string;
}

/** optional_config_overrides: a nested object shaped like core_config.json. */
export type Overrides = { [key: string]: Overrides | number | boolean | string };

/** Two drones' closest approach in one simulated flight. */
export interface McPair {
  drone_a: number;
  drone_b: number;
  min_distance_m: number;
  time_sec: number;
}

/** One Monte Carlo flight (monte_carlo_runner.evaluate_run); run -1 is the undisturbed nominal flight. */
export interface McRecord {
  run: number;
  seed: number;
  passed: boolean;
  scenario: { mean_wind_mps: number; gust_peak_mps: number; ambient_c: number };
  min_separation_m: number | null;
  crash_pairs: McPair[];
  warning_pairs: McPair[];
  min_final_soc: number;
  low_soc_drones: number[];
  brownout_drones: number[];
  min_voltage_v: number;
  max_tracking_error_m: number;
  sim_duration_sec: number;
  wall_time_sec: number;
  realtime_factor: number;
}

export interface McSummary {
  runs: number;
  passed: boolean;
  crash_rate: number;
  crash_runs: number[];
  low_soc_runs: number[];
  warning_runs: number[];
  brownout_runs: number[];
  worst_min_separation_m: number | null;
  worst_final_soc: number | null;
  worst_tracking_error_m: number | null;
  mean_realtime_factor: number | null;
  nominal?: McRecord;
}

/** stage3/monte_carlo_report.json (the part the UI reads). */
export interface McReport {
  device: string;
  fleet_size: number;
  show_duration_sec: number;
  criteria: { d_crash_m: number; d_warning_m: number; min_landing_soc: number };
  summary: McSummary;
  wall_time_sec: number;
  runs: McRecord[];
}

/** stage3/bin/manifest.json, written by pack_to_binary. */
export interface PackManifest {
  source: string;
  fleet_size: number;
  sampling_dt_ms: number;
  records_per_file: number;
  file_size_bytes: number;
  files: { drone_id: number; file: string; size_bytes: number; crc32: string; verified: boolean }[];
}

export interface Issue {
  path: string;
  message: string;
}

export interface KeyframeSummary {
  shape_name: string;
  time_sec: number;
  points: number;
  min_spacing_m: number;
  bbox_min: Vec3;
  bbox_max: Vec3;
}

export interface ValidationSummary {
  fleet_size: number;
  version: string;
  total_duration_sec: number;
  min_distance_m: number;
  keyframes: KeyframeSummary[];
  holding_area: {
    center: Vec3;
    size: [number, number];
    max_height: number;
    grid_spacing_m: number;
    layers: number;
    /** Phase 1's layout rules applied to the declared area (tools/studio_bridge/validate.py). */
    capacity: {
      per_layer: number;
      max_layers: number;
      capacity: number;
      layers_used: number;
      widened: boolean;
      width_used_m: number;
    };
    gatekeeper_floor_m: number | null;
  };
  first_formation_targets_in_holding_area: number;
  /** Drones the first formation doesn't use, left parked on their holding slots (not an overlap). */
  first_formation_parked?: number;
}

export interface Validation {
  ok: boolean;
  errors: Issue[];
  warnings: Issue[];
  summary: ValidationSummary | null;
}

/** An OpenCL device the digital twin can run on (`doctor`); `id` is what `--device` takes. */
export interface SimDevice {
  id: string;
  name: string;
  kind: "gpu" | "cpu" | "other";
  compute_units: number;
  memory_mib: number;
}

export interface DoctorCheck {
  name: string;
  ok: boolean;
  detail: string;
  required_for: string;
}

export interface Violation {
  drone_a: number;
  drone_b: number;
  time_sec: number;
  distance_m: number;
  position_a: Vec3;
  position_b: Vec3;
}

export interface Attempt {
  attempt: number;
  duration_sec: number;
  worst_separation_m: number;
}

/** The part of failure.json the UI reads (the trajectories stay on disk). */
export interface FailureSummary {
  transition: Transition;
  worst_separation_m: number;
  required_separation_m: number;
  attempts: Attempt[];
  violating_pair_count: number;
  violations_truncated: boolean;
  violations: Violation[];
}

interface SolveTransition {
  transition: number; // 0 = holding area -> first formation
  transition_count: number;
  from_keyframe: string;
  to_keyframe: string;
  /** Only in a `stage2-returns` job: which return of the job, and its formation. */
  return_index?: number;
  return_count?: number;
  keyframe_index?: number;
}

interface SolveAttempt extends SolveTransition {
  attempt: number; // 1-based
  max_attempts: number;
  duration_sec: number;
}

/** drone_core progress_callback events (docs/5-studio_gui.md §5.2), forwarded as `solve_progress`. */
export type SolveProgress =
  | (SolveTransition & { event: "transition_start" | "transition_end"; show_time_sec: number })
  | (SolveAttempt & { event: "attempt_start" })
  | (SolveAttempt & {
      event: "scp_iteration";
      substage: number;
      substage_count: number;
      iteration: number;
      max_iterations: number;
      conflict_pairs: number;
      max_delta_m: number;
      min_separation_m: number | null;
      converged: boolean;
    })
  | (SolveAttempt & {
      event: "attempt_end";
      worst_separation_m: number | null;
      required_separation_m: number;
      passed: boolean;
    });

export type BridgeEvent =
  | { type: "phase"; name: string; detail: string }
  | { type: "progress"; stage: string; done: number; total: number }
  | ({ type: "solve_progress" } & SolveProgress)
  | { type: "validation"; ok: boolean; errors: Issue[]; warnings: Issue[]; summary: ValidationSummary | null }
  | {
      type: "doctor";
      repo_root: string;
      extension_dir: string;
      checks: DoctorCheck[];
      devices?: string[];
      device_info?: SimDevice[];
    }
  | { type: "run_created"; run_id: string; run_dir: string }
  | { type: "stage2_result"; wall_time_sec: number; total_duration_sec: number; fleet_size: number }
  | ({ type: "stage2_failure"; message: string; wall_time_sec: number } & Omit<FailureSummary, "violations">)
  | { type: "replay_ready"; frames: number; fleet_size: number }
  | { type: "mc_start"; runs: number; device: string; batch: number; seed: number }
  | ({ type: "mc_run" } & McRecord)
  | { type: "mc_result"; summary: McSummary; device: string; wall_time_sec: number }
  | ({ type: "pack_result"; ok: boolean } & PackSummary)
  | { type: "config"; fields: ConfigField[]; overrides: Overrides; warnings: ConfigWarning[] }
  | { type: "config_warnings"; warnings: ConfigWarning[] }
  | ({ type: "return_result" } & ReturnEntry)
  | ({ type: "conditions" } & ConditionsInfo)
  | { type: "scenario_saved"; id: string; scenario: Scenario }
  | { type: "scenario_deleted"; id: string }
  | { type: "sim_result"; id: string; result: ScenarioResult }
  | { type: "error"; code: string; message: string; errors?: string[] }
  | { type: "done"; status: RunStatus; exit_code: number }
  | { type: "stdout"; line: string };

export interface JobExit {
  code: number | null;
  cancelled: boolean;
}
