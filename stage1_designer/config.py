"""Global default configuration for the Phase 1 Blender add-on.

Values mirror the defaults defined in `.claude/docs/1-phase_1.md` section 3.1-3.4
and the intermediate schema at `schemas/project_intermediate.schema.json`.
"""

SCHEMA_VERSION = "1.4.0"

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

# --- Holding area defaults (section 3.2) ---
DEFAULT_HOLDING_AREA = {
    "center": (0.0, -30.0, 5.0),
    "size": (40.0, 10.0),
    "max_height": 15.0,
    "layer_spacing_m": MIN_DISTANCE_M,
}
Z_HOLD_MAX_DEFAULT = 15.0

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

# --- Viewport overlay colors (section 3.5) ---
COLOR_VALID_RGBA = (0.0, 1.0, 0.0, 1.0)
COLOR_VIOLATION_RGBA = (1.0, 0.0, 0.0, 1.0)
COLOR_HOLDING_RGBA = (0.2, 0.6, 1.0, 1.0)  # parked/holding-area drones (guaranteed collision-free)
