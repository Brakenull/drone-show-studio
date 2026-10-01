// Replay data (docs/5-studio_gui.md §6.4). Built by tools/studio_bridge/replay_builder.py.
// This folder has no Tauri imports so the Phase 4 dashboard can reuse it.

export type V3 = [number, number, number];

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
  fps: number;
  frames: number;
  t0: number;
  t1: number;
  bounds_min: V3;
  bounds_max: V3;
  ground_z_m: number;
  below_ground: { drone: number; min_z_m: number; time_sec: number }[];
  min_distance_enforced_m: number | null;
  overlays: {
    holding_area?: { center: V3; size: [number, number]; grid_spacing_m: number; slots: V3[] };
    keyframes?: { shape_name: string }[];
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
    };
  };
}

export interface Separation {
  times: number[];
  min_m: (number | null)[];
  a: number[];
  b: number[];
  worst: { frame: number; time_sec: number; distance_m: number | null; a: number; b: number };
  crash_m: number;
  sampled_fps: number;
}

export interface ReplayData {
  header: ReplayHeader;
  separation: Separation;
  positions: Float32Array; // frames x n x 3, ENU metres
  colors: Uint8Array; // frames x n x 3
}

/** A moment to jump to, e.g. a violation picked in the Stage 2 failure panel. */
export interface ReplayFocus {
  time: number;
  drones: number[];
  /** Shown with the replay's label, e.g. that a Monte Carlo crash is drawn on the planned paths. */
  note?: string;
  key: number; // changes on every request so the same moment can be focused twice
}
