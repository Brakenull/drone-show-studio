// Replay data. Built by tools/studio_bridge/replay_builder.py.
// This folder has no Tauri imports so the Phase 4 dashboard can reuse it.

export type V3 = [number, number, number];

/** A holding area in the 3D views: its own drones' slots. */
export interface HoldingOverlay {
  center: V3;
  size: [number, number];
  grid_spacing_m: number;
  slots: V3[];
}

export interface ReplayViolation {
  drone_a: number;
  drone_b: number;
  time_sec: number;
  distance_m: number;
  position_a: V3;
  position_b: V3;
}

export interface ReplayHeader {
  fleet_size: number;
  files?: { positions: string; colors: string; separation: string; reference?: string };
  fps: number;
  frames: number;
  t0: number;
  t1: number;
  bounds_min: V3;
  bounds_max: V3;
  ground_z_m: number;
  below_ground: { drone: number; min_z_m: number; time_sec: number }[];
  min_distance_enforced_m: number | null;
  /** Where each formation falls in the planned show; absent from replays built before 2026-10-01. */
  timeline?: ShowTimeline;
  overlays: {
    /** Replays built before several holding areas: the one area. */
    holding_area?: HoldingOverlay;
    /** Each holding area with its own drones' slots. */
    holding_areas?: HoldingOverlay[];
    /** Per drone: its home holding area (0-based). */
    home_area?: number[];
    /** Places in the air where spare drones wait (schema 1.7.0); absent or empty without any. */
    waiting_areas?: { center: V3; size: [number, number]; grid_spacing_m: number; slots: V3[] }[];
    keyframes?: { shape_name: string; time_sec?: number }[];
    nominal_min_distance_m?: number;
    gatekeeper_floor_m?: number | null;
    failure?: {
      transition: {
        index: number;
        from_keyframe: string;
        to_keyframe: string;
        start_time_sec: number;
        duration_sec: number;
      };
      required_separation_m: number;
      worst_separation_m: number;
      violations: ReplayViolation[];
    };
    /** A return path's replay: the show until abort_time_sec, then the flight home. */
    return_path?: {
      keyframe_index: number;
      from_keyframe: string;
      abort_time_sec: number;
      duration_sec: number;
      method: "planned" | "reversed_takeoff";
      /** From a moment inside the move into the formation (an abort point). */
      inside_transition?: boolean;
    };
    /** A condition-simulator flight: the weather to draw. */
    simulation?: SimWeather;
  };
}

/** tools/studio_bridge/replay_builder.py show_timeline(). */
export interface FormationMark {
  /** Index in the Phase 1 file's keyframes. */
  index: number;
  name: string;
  /** When the planned show reaches it. */
  reached_sec: number;
  /** When the next transition leaves it (later than reached_sec when the formation is held), or null. */
  leaves_sec: number | null;
  /** Its time_sec in the Blender export. */
  designed_sec: number | null;
  /** The target of a rejected transition: the rejected attempt ends here. */
  rejected: boolean;
}

export interface ShowTimeline {
  formations: FormationMark[];
  legs: { takeoff?: { start_sec: number; end_sec: number }; return?: { start_sec: number; end_sec: number } };
}

/** The scenario's weather sampled at `hz` from t = 0 (tools/studio_bridge/conditions_job.py). */
export interface SimWeather {
  hz: number;
  speed_mps: number[];
  from_deg: number[];
  turbulence: number[];
  rain_mm_h: number[];
  rtk: number[];
  rtk_states: string[];
  gusts: {
    t: number;
    /** When the front passes the ENU origin; it reaches the field centre at `t`. */
    start_sec: number;
    duration_s: number;
    peak_mps: number;
    from_deg: number;
    speed_mps: number;
    /** Unit direction the front moves in (ENU x, y). */
    dir: [number, number];
  }[];
  field_center: [number, number];
  rain_rule: { alert_mm_h: number; limit_mm_h: number; reaction_s: number; margin_s: number };
  alert_time_sec?: number | null;
  /** The rain rule's return to the holding area, when the rain reached the alert level. */
  rain_return?: {
    command_sec: number;
    formation_name: string | null;
    method: string;
    start_sec: number;
    planned_home_sec: number | null;
    deadline_sec: number | null;
    last_landing_sec: number | null;
  } | null;
}

export interface Separation {
  times: number[];
  min_m: (number | null)[];
  a: number[];
  b: number[];
  worst: { frame: number; time_sec: number; distance_m: number | null; a: number; b: number };
  crash_m: number;
  sampled_fps: number;
  /** Simulated flights only: per frame, the largest gap between a drone and its planned position. */
  deviation_m?: number[];
  deviation_drone?: number[];
}

export interface ReplayData {
  header: ReplayHeader;
  separation: Separation;
  positions: Float32Array; // frames x n x 3, ENU metres
  colors: Uint8Array; // frames x n x 3
  /** Simulated flights only: planned positions at the same frames. */
  reference?: Float32Array;
}

/** A moment to jump to, e.g. a violation picked in the Stage 2 failure panel. */
export interface ReplayFocus {
  time: number;
  drones: number[];
  /** Shown with the replay's label, e.g. that a Monte Carlo crash is drawn on the planned paths. */
  note?: string;
  key: number; // changes on every request so the same moment can be focused twice
}
