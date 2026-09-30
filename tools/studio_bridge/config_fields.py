"""Stage 2 planner settings for the overrides editor (docs/5-studio_gui.md §6.2).

Mirrors what drone_core reads (stage2_core_engine/include/config.hpp, apply_json_overrides) and how it
resolves them: core_config.json, then the Phase 1 file's kinematic_constraints (kinematics only), then
optional_config_overrides. drone_core silently ignores keys it doesn't know, so `override_errors` rejects
them here: a typo must not look like an applied setting.

`risky` says which direction makes a setting less safe ("lower", "higher", or "off" for a switch); an
override that moves it that way from the run's baseline gets a warning before the run and in run.json.
"""

from __future__ import annotations

import copy
import json
from typing import Any

from .paths import REPO_ROOT

CORE_CONFIG = REPO_ROOT / "stage2_core_engine" / "config" / "core_config.json"
KINEMATICS = "kinematics_default"


def _f(path: str, label: str, help: str, kind: str = "number", *, group: str, risky: str | None = None,
       minimum: float | None = None, choices: list[str] | None = None,
       builtin: Any = None) -> dict[str, Any]:
    # `builtin`: config.hpp's value for a key core_config.json doesn't list.
    return {"path": path, "label": label, "help": help, "kind": kind, "group": group, "risky": risky,
            "min": minimum, "choices": choices, "builtin": builtin}


FIELDS: list[dict[str, Any]] = [
    # Safety: what "too close" means.
    _f("solver.continuous_gatekeeper.min_allowable_distance_m", "Required distance",
       "Stage 2 rejects the show if any two drones get closer than this.", group="Safety check",
       risky="lower", minimum=0.0),
    _f("safety.min_distance_m", "Planning distance",
       "The distance the planner aims to keep between drones (plus the margin below).", group="Safety check",
       risky="lower", minimum=0.0),
    _f("solver.collision_margin_fraction", "Planning margin",
       "Extra fraction added to the planning distance, e.g. 0.05 = 5 %.", group="Safety check", risky="lower",
       minimum=0.0),
    _f("safety.safety_radius_m", "Drone radius", "Physical radius used to pad each drone's box when searching "
       "for close pairs.", group="Safety check", risky="lower", minimum=0.0),
    _f("solver.continuous_gatekeeper.verification_frequency_hz", "Check rate",
       "Samples per second of the final safety check. Fewer samples can miss a short close pass.",
       group="Safety check", risky="lower", minimum=1.0),
    _f("solver.continuous_gatekeeper.auto_retry_with_expansion", "Retry rejected transitions",
       "Retry a rejected transition with more time and stronger spreading.", "boolean", group="Safety check"),
    _f("solver.continuous_gatekeeper.max_retry_count", "Retries", "How many times to retry a rejected "
       "transition.", "integer", group="Safety check", minimum=0),
    _f("solver.continuous_gatekeeper.expansion_factor", "Retry time factor",
       "Each retry multiplies the transition time by this.", group="Safety check", minimum=1.0),
    _f("solver.continuous_gatekeeper.min_retry_separation_m", "Retry only near misses",
       "Metres. A transition whose closest pass is below this is not retried: more time can't fix a miss "
       "that deep. 0 retries every miss.", group="Safety check", minimum=0.0),
    # Motion limits.
    _f(f"{KINEMATICS}.v_max_mps", "Top speed", "m/s.", group="Motion limits", risky="higher", minimum=0.0),
    _f(f"{KINEMATICS}.a_max_mps2", "Max acceleration", "m/s².", group="Motion limits", risky="higher",
       minimum=0.0),
    _f(f"{KINEMATICS}.j_max_mps3", "Max jerk", "m/s³.", group="Motion limits", risky="higher", minimum=0.0),
    _f("solver.auto_scale_transition_time", "Stretch transitions that are too short",
       "Give a transition more time when the drones can't make it within the motion limits.", "boolean",
       group="Motion limits", risky="off"),
    _f("solver.kinematic_slack_fraction", "Speed headroom",
       "Fraction of the limits kept free for swerving around other drones.", group="Motion limits",
       minimum=0.0),
    # Takeoff.
    _f("solver.enable_staggered_takeoff", "Staggered takeoff", "Launch rows one after another instead of all "
       "at once.", "boolean", group="Takeoff"),
    _f("solver.staggered_wave_delay_s", "Delay between rows", "Seconds.", group="Takeoff", minimum=0.0),
    # Planner.
    _f("solver.max_scp_iterations", "Refining passes", "Maximum refining passes per transition part.",
       "integer", group="Planner", minimum=1),
    _f("solver.convergence_tol", "Stop when moves are below", "Metres.", group="Planner", minimum=0.0),
    _f("solver.scp_stall_iterations", "Stop after passes without progress",
       "Ends a transition part after this many refining passes in a row that don't improve it. 0 = off.",
       "integer", group="Planner", minimum=0),
    _f("solver.scp_stall_tol_m", "Progress threshold", "Metres. Smaller gains count as no progress.",
       group="Planner", minimum=0.0),
    _f("solver.centered_formation_velocity", "Formation speed from both legs",
       "Drones pass a formation with the velocity of the formation's centre between the previous and "
       "the next formation, instead of full speed along the way they arrived.", "boolean", group="Planner"),
    _f("solver.shared_substage_velocity", "Shared velocity between transition parts",
       "Long transitions are split into parts; all drones pass each split with the same velocity, so "
       "drones passing each other there can't come closer than planned.", "boolean", group="Planner"),
    _f("solver.repair_seed", "Start from flyable paths", "Move each drone's starting path to the closest one "
       "within the speed, acceleration and jerk limits before planning.", "boolean", group="Planner"),
    _f("solver.trust_region_delta_m", "Largest move per pass", "Metres.", group="Planner", minimum=0.0),
    _f("solver.num_control_points_min", "Path detail (minimum)", "Control points per path segment.",
       "integer", group="Planner", minimum=6),
    _f("solver.adaptive_control_points", "More detail for longer paths", "", "boolean", group="Planner"),
    _f("solver.max_substage_duration_s", "Longest transition part", "Seconds. Longer transitions are split.",
       group="Planner", minimum=1.0),
    _f("solver.adaptive_time_bucketing", "Adaptive time windows", "", "boolean", group="Planner"),
    _f("solver.target_time_windows", "Time windows per transition", "", "integer", group="Planner", minimum=1),
    _f("solver.jitter_magnitude_m", "Nudge when stuck", "Metres. How far a stuck drone is nudged sideways "
       "(built into the planner, not in core_config.json).", group="Planner", minimum=0.0, builtin=0.3),
    _f("solver.w_slack_collision", "Collision penalty", "How hard the planner pushes drones apart.",
       group="Planner", minimum=0.0),
    _f("solver.enable_graph_coloring", "Solve neighbours in parallel", "", "boolean", group="Planner"),
    _f("solver.cluster_distance_threshold_m", "Cluster distance", "Metres (not used while disabled in the "
       "solver).", group="Planner", minimum=0.0),
    _f("solver.max_cluster_size", "Cluster size", "(Not used while disabled in the solver.)", "integer",
       group="Planner", minimum=1),
    _f("solver.cutting_plane.enabled", "Extra checks between samples", "Adds constraints where two drones "
       "pass close between the planner's samples.", "boolean", group="Planner"),
    _f("solver.cutting_plane.max_dynamic_collocations_per_pair", "Extra checks per pair",
       "Most extra constraints per drone pair and pass, at its closest points. 0 = all of them.", "integer",
       group="Planner", minimum=0),
    _f("solver.cutting_plane.detection_frequency_hz", "Extra-check scan rate", "Samples per second.",
       group="Planner", minimum=1.0),
    # Spreading at the start of planning.
    _f("solver.apf_seeding.enabled", "Spread start paths apart", "Start planning from paths pushed apart "
       "by a repulsion simulation.", "boolean", group="Start paths"),
    _f("solver.apf_seeding.k_repulsion", "Repulsion strength", "", group="Start paths", minimum=0.0),
    _f("solver.apf_seeding.detection_radius_m", "Repulsion range", "Metres.", group="Start paths",
       minimum=0.0),
    _f("solver.apf_seeding.num_euler_steps", "Repulsion steps", "", "integer", group="Start paths", minimum=0),
    _f("solver.apf_seeding.euler_dt", "Repulsion step size", "", group="Start paths", minimum=0.0),
    _f("solver.apf_seeding.z_stratification_mode", "Height layers by direction",
       "Send drones heading in different directions to different heights.", "choice", group="Start paths",
       choices=["4_sector_discrete", "disabled"]),
    _f("solver.apf_seeding.z_layer_step_m", "Height layer step", "Metres.", group="Start paths", minimum=0.0),
    # Assignment and smoothness weights.
    _f("solver_weights.w_distance", "Weight: distance", "", group="Weights", minimum=0.0),
    _f("solver_weights.w_vertical_climb", "Weight: climbing", "", group="Weights", minimum=0.0),
    _f("solver_weights.w_heading_change", "Weight: turning", "", group="Weights", minimum=0.0),
    _f("solver_weights.w_smoothness_snap", "Weight: smoothness (snap)", "", group="Weights", minimum=0.0),
    _f("solver_weights.w_smoothness_jerk", "Weight: smoothness (jerk)", "", group="Weights", minimum=0.0),
]
BY_PATH = {f["path"]: f for f in FIELDS}


def get(tree: dict[str, Any], path: str) -> Any:
    node: Any = tree
    for key in path.split("."):
        if not isinstance(node, dict) or key not in node:
            return None
        node = node[key]
    return node


def _set(tree: dict[str, Any], path: str, value: Any) -> None:
    *parents, leaf = path.split(".")
    for key in parents:
        tree = tree.setdefault(key, {})
    tree[leaf] = value


def leaves(tree: dict[str, Any], prefix: str = "") -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key, value in tree.items():
        path = f"{prefix}{key}"
        if isinstance(value, dict):
            out.update(leaves(value, path + "."))
        else:
            out[path] = value
    return out


def load_defaults() -> dict[str, Any]:
    """core_config.json, plus the built-in value of any setting the file leaves out."""
    with CORE_CONFIG.open("r", encoding="utf-8") as fh:
        defaults = json.load(fh)
    for field in FIELDS:
        if field["builtin"] is not None and get(defaults, field["path"]) is None:
            _set(defaults, field["path"], field["builtin"])
    return defaults


def baseline(phase1_meta: dict[str, Any] | None, defaults: dict[str, Any] | None = None) -> dict[str, Any]:
    """What a run uses without overrides: core_config.json plus the Phase 1 kinematic_constraints."""
    base = copy.deepcopy(defaults if defaults is not None else load_defaults())
    kc = (phase1_meta or {}).get("kinematic_constraints") or {}
    for key in ("v_max_mps", "a_max_mps2", "j_max_mps3"):
        if kc.get(key) is not None:
            _set(base, f"{KINEMATICS}.{key}", kc[key])
    return base


def override_errors(overrides: Any) -> list[str]:
    """Unknown keys and wrong value types. drone_core would ignore the first and throw on the second."""
    if not isinstance(overrides, dict):
        return ["overrides must be a JSON object"]
    errors = []
    for path, value in leaves(overrides).items():
        field = BY_PATH.get(path.replace("weights.", "solver_weights.", 1) if path.startswith("weights.") else path)
        if field is None:
            errors.append(f"{path}: not a planner setting")
            continue
        kind = field["kind"]
        ok = {
            "boolean": isinstance(value, bool),
            "integer": isinstance(value, int) and not isinstance(value, bool),
            "number": isinstance(value, (int, float)) and not isinstance(value, bool),
            "choice": isinstance(value, str),  # drone_core treats any other mode as "off"
        }[kind]
        if not ok:
            errors.append(f"{path}: expected {kind}, got {json.dumps(value)}")
        elif field["min"] is not None and kind in ("integer", "number") and value < field["min"]:
            errors.append(f"{path}: must be at least {field['min']}")
    return errors


def safety_warnings(base: dict[str, Any], overrides: dict[str, Any]) -> list[dict[str, str]]:
    """Overrides that make the run less safe than its baseline, plus settings the check can't enforce."""
    effective = copy.deepcopy(base)
    for path, value in leaves(overrides).items():
        _set(effective, path.replace("weights.", "solver_weights.", 1) if path.startswith("weights.") else path,
             value)
    warnings = []
    for field in FIELDS:
        old, new = get(base, field["path"]), get(effective, field["path"])
        if old is None or new is None or old == new or field["risky"] is None:
            continue
        worse = {"lower": lambda: new < old, "higher": lambda: new > old, "off": lambda: old and not new}
        if worse[field["risky"]]():
            verb = {"lower": "lowered", "higher": "raised"}.get(field["risky"])
            change = f"{verb} from {old} to {new}" if verb else "turned off"
            warnings.append({"path": field["path"], "label": field["label"],
                             "message": f"{field['label']} {change}."})

    # The check only finds pairs whose planning-distance boxes touch (2-phase_2.md §5, known limit).
    floor = get(effective, "solver.continuous_gatekeeper.min_allowable_distance_m")
    enforced = get(effective, "safety.min_distance_m") * (1 + get(effective, "solver.collision_margin_fraction"))
    if floor is not None and floor > 2 * enforced:
        warnings.append({"path": "solver.continuous_gatekeeper.min_allowable_distance_m",
                         "label": BY_PATH["solver.continuous_gatekeeper.min_allowable_distance_m"]["label"],
                         "message": f"Required distance {floor} m is more than twice the planning distance "
                                    f"({enforced:.2f} m). The check can't see pairs that far apart, so it "
                                    "may pass a show that breaks it. Raise the planning distance too."})
    return warnings


def describe(phase1_meta: dict[str, Any] | None, overrides: dict[str, Any]) -> dict[str, Any]:
    """Everything the editor needs: field metadata with each field's default and baseline value."""
    defaults = load_defaults()
    base = baseline(phase1_meta, defaults)
    fields = [{**f, "default": get(defaults, f["path"]), "baseline": get(base, f["path"])} for f in FIELDS]
    return {"fields": fields, "overrides": overrides, "warnings": safety_warnings(base, overrides)}
