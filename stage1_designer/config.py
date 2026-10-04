"""Global default configuration for the Phase 1 Blender add-on.

Values mirror the defaults defined in `.claude/docs/1-phase_1.md` section 3.1-3.4
and the intermediate schema at `schemas/project_intermediate.schema.json`.
"""

SCHEMA_VERSION = "1.7.0"  # 1.7.0: holding_area.layer_spacing_m honoured, staggered_layers (section 3.2)

# --- Safety distance (section 3.1) ---
SAFETY_RADIUS_M = 0.75          # R_safe
MIN_DISTANCE_M = 2.0 * SAFETY_RADIUS_M  # d_min = 2 * R_safe = 1.5 m

# --- Fleet defaults ---
DEFAULT_FLEET_SIZE = 200

# --- Sampling modes (section 3.4) ---
SAMPLING_MODE_KEYFRAME_ONLY = "KEYFRAME_ONLY"
SAMPLING_MODE_DENSE_SAMPLED = "DENSE_SAMPLED"
SAMPLING_MODES = (SAMPLING_MODE_KEYFRAME_ONLY, SAMPLING_MODE_DENSE_SAMPLED)
DEFAULT_DENSE_FPS = 10

# --- Holding area defaults (section 3.2, Rev 1.5) ---
# d_launch: the holding-area grid pitch, deliberately >= MIN_DISTANCE_M with a
# 0.5 m margin. Rest-to-rest launch control points can't bend to dodge a
# neighbor the way an in-flight formation point can, so parking drones at
# exactly d_min (1.5 m) left zero slack and produced real sub-d_min dips
# (down to 1.418 m, per the Phase 2 findings this revision exists to fix)
# during the first seconds of liftoff.
DEFAULT_GRID_SPACING_M = 2.0     # d_launch
MIN_GRID_SPACING_M = 1.8         # hard floor enforced by the UI property
# Stacked layers (section 3.2.3, Rev 1.7, option C):
# 4 m between layers and odd layers shifted half a slot, so no slot sits
# straight above another (downwash) and a pad's vertical path to its hover
# point stays clear of the slot above.
DEFAULT_LAYER_SPACING_M = 4.0
DEFAULT_STAGGERED_LAYERS = True
# Mirrors stage2_core_engine/config/core_config.json
# (solver.landing_approach_height_m): the hover point above a pad that Stage 2's
# vertical pad moves climb to / descend from (2-phase_2.md sections 1.26-1.28).
STAGE2_HOVER_HEIGHT_M = 2.0
# Stage 2 plans to min_distance_m x this factor (enforced_min_distance_m).
STAGE2_PLANNING_DISTANCE_FACTOR = 1.05
DEFAULT_HOLDING_AREA = {
    "center": (0.0, -30.0, 5.0),
    "size": (40.0, 10.0),
    "max_height": 15.0,
    "grid_spacing_m": DEFAULT_GRID_SPACING_M,
    "layer_spacing_m": DEFAULT_LAYER_SPACING_M,
    "staggered_layers": DEFAULT_STAGGERED_LAYERS,
}
Z_HOLD_MAX_DEFAULT = 15.0
# Minimum distance from any formation point to the holding region (the
# parked grid padded by half a grid step) before the add-on raises a caution.
# Drones leaving the upper layers need room to climb out past the formation;
# P1-01's failing file had targets ~2 m from the parked grid.
DEFAULT_SHOW_CLEARANCE_M = 5.0

# --- Ground level (section 3.9) ---
# ENU height of the ground. The heading offset only rotates about Z, so this is
# also Blender Z. 0.0 = the ENU origin plane (origin_gps.altitude_amsl).
DEFAULT_GROUND_Z_M = 0.0

# --- Takeoff and return legs (section 3.8) ---
LEG_MODE_AUTO = "AUTO"      # Stage 2 flies the leg in its own minimum time
LEG_MODE_TARGET = "TARGET"  # Stage 2 flies max(target, its minimum)
DEFAULT_LEG_DURATION_SEC = 30.0
# Mirrors stage2_core_engine/config/core_config.json (solver.enable_staggered_takeoff
# = true, solver.staggered_wave_delay_s = 1.2), used only for the panel's takeoff
# estimate. Stage 2's own config is what actually applies.
STAGE2_STAGGER_WAVE_DELAY_S = 1.2

# --- Kinematic pre-validator (section 3.5) ---
# Status thresholds as a fraction of v_max, and the same slack fraction /
# peak-velocity constant (1.875 = 15/8, the quintic minimum-snap rest-to-rest
# peak-velocity factor) that Phase 2's T_min auto-scaling uses, so Auto-Fix
# here and the SCP solver's own duration floor agree on what "safe" means.
KINEMATIC_WARNING_FRACTION = 0.75
KINEMATIC_SLACK_FRACTION = 0.25
QUINTIC_PEAK_VELOCITY_FACTOR = 1.875

# --- Coordinate system / heading (section 3.3) ---
COORDINATE_SYSTEM = "ENU"
DEFAULT_HEADING_OFFSET_DEG = 0.0

# --- Kinematic constraints (optional, null means Phase 2 fallback) ---
DEFAULT_KINEMATIC_CONSTRAINTS = {
    "v_max_mps": 6.0,
    "a_max_mps2": 3.0,
    "j_max_mps3": 5.0,
}

# --- Volumetric sampling (section 3.1) ---
import math as _math

VOLUMETRIC_CELL_SIZE_FACTOR = 1.0 / _math.sqrt(3.0)  # s = d_min / sqrt(3)
VOLUMETRIC_JITTER_FACTOR = 0.2  # delta in [-0.2s, 0.2s]

# --- Viewport overlay colors (section 3.5/3.6) ---
COLOR_VALID_RGBA = (0.0, 1.0, 0.0, 1.0)
COLOR_VIOLATION_RGBA = (1.0, 0.0, 0.0, 1.0)
COLOR_HOLDING_RGBA = (0.2, 0.6, 1.0, 1.0)  # parked/holding-area drones (guaranteed collision-free)
COLOR_KINEMATIC_WARNING_RGBA = (1.0, 0.8, 0.0, 1.0)  # yellow: transition needs attention
COLOR_GROUND_RGBA = (0.45, 0.35, 0.25, 1.0)  # brown: ground grid
COLOR_WAITING_RGBA = (0.3, 0.9, 0.8, 1.0)  # teal: waiting areas (section 3.10)

# --- Waiting areas (section 3.10) ---
# New areas start here (ENU); there is no sensible universal place, the
# designer moves them next to the show.
DEFAULT_WAITING_AREA = {
    "center": (40.0, 0.0, 10.0),
    "size": (10.0, 10.0),
    "grid_spacing_m": DEFAULT_GRID_SPACING_M,
    "show_clearance_m": DEFAULT_SHOW_CLEARANCE_M,
}
