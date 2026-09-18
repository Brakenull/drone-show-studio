"""Global default configuration for the Phase 1 Blender add-on.

Values mirror the defaults defined in `.claude/docs/1-phase_1.md` section 3.1-3.4
and the intermediate schema at `schemas/project_intermediate.schema.json`.
"""

SCHEMA_VERSION = "1.5.0"

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
DEFAULT_HOLDING_AREA = {
    "center": (0.0, -30.0, 5.0),
    "size": (40.0, 10.0),
    "max_height": 15.0,
    "grid_spacing_m": DEFAULT_GRID_SPACING_M,
    "layer_spacing_m": DEFAULT_GRID_SPACING_M,
}
Z_HOLD_MAX_DEFAULT = 15.0

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
