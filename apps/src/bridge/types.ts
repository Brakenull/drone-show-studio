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
  };
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
  };
  first_formation_targets_in_holding_area: number;
}

export interface Validation {
  ok: boolean;
  errors: Issue[];
  warnings: Issue[];
  summary: ValidationSummary | null;
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
  | { type: "doctor"; repo_root: string; extension_dir: string; checks: DoctorCheck[] }
  | { type: "run_created"; run_id: string; run_dir: string }
  | { type: "stage2_result"; wall_time_sec: number; total_duration_sec: number; fleet_size: number }
  | ({ type: "stage2_failure"; message: string; wall_time_sec: number } & Omit<FailureSummary, "violations">)
  | { type: "replay_ready"; frames: number; fleet_size: number }
  | { type: "error"; code: string; message: string }
  | { type: "done"; status: RunStatus; exit_code: number }
  | { type: "stdout"; line: string };

export interface JobExit {
  code: number | null;
  cancelled: boolean;
}
