"""Keyframe / timeline sampling and ENU coordinate mapping (spec section 3.3-3.4).

`enu_transform` is pure numpy/math and unit-testable. The scene/keyframe
readers are `bpy`-dependent and import it locally.
"""

from __future__ import annotations

import math
from typing import List, Optional

import numpy as np

from ..config import (
    SAMPLING_MODE_DENSE_SAMPLED,
    SAMPLING_MODE_KEYFRAME_ONLY,
)


def enu_transform(points: np.ndarray, heading_offset_deg: float) -> np.ndarray:
    """Rotate Blender-space points into the ENU frame per the heading offset.

    theta = heading_offset_deg (clockwise from +Y_blender to True North, looking
    down); rotation matrix as defined in `.claude/docs/1-phase_1.md` section 3.3:

        [X_enu]   [ cos(t)  sin(t)  0] [X_blender]
        [Y_enu] = [-sin(t)  cos(t)  0] [Y_blender]
        [Z_enu]   [   0        0    1] [Z_blender]
    """
    theta = math.radians(heading_offset_deg)
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    rotation = np.array(
        [
            [cos_t, sin_t, 0.0],
            [-sin_t, cos_t, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    pts = np.atleast_2d(np.asarray(points, dtype=float))
    return pts @ rotation.T


# --------------------------------------------------------------------------
# bpy-dependent readers (only usable inside Blender)
# --------------------------------------------------------------------------

def _iter_action_fcurves(action, slot=None):
    """Yield every F-Curve of `action`, handling both the legacy flat layout
    (`action.fcurves`, pre-4.4) and the layered layout (Blender >= 4.4:
    `action.layers[].strips[].channelbags[].fcurves`).

    When `slot` (the owning object's `animation_data.action_slot`) is given,
    only channelbags bound to that slot are considered - an action can hold
    channels for several objects/slots at once in the layered system.
    """
    if action is None:
        return

    if getattr(action, "is_action_legacy", True) and hasattr(action, "fcurves"):
        yield from action.fcurves
        return

    slot_handle = getattr(slot, "handle", None)
    for layer in action.layers:
        for strip in layer.strips:
            for channelbag in getattr(strip, "channelbags", ()):
                if slot_handle is not None and channelbag.slot_handle != slot_handle:
                    continue
                yield from channelbag.fcurves


def get_keyframe_times_seconds(obj, fps: float) -> List[float]:
    """Collect the sorted, de-duplicated keyframe times (seconds) of an object's action."""
    anim = obj.animation_data
    if anim is None or anim.action is None:
        return []

    frames = set()
    action_slot = getattr(anim, "action_slot", None)
    for fcurve in _iter_action_fcurves(anim.action, action_slot):
        for keyframe in fcurve.keyframe_points:
            frames.add(keyframe.co.x)

    return sorted(frame / fps for frame in frames)


def get_sample_times_seconds(
    obj,
    scene,
    sampling_mode: str,
    fps: Optional[float] = None,
) -> List[float]:
    """Return the list of times (seconds) at which to sample `obj`, per mode."""
    scene_fps = scene.render.fps / scene.render.fps_base

    if sampling_mode == SAMPLING_MODE_KEYFRAME_ONLY:
        return get_keyframe_times_seconds(obj, scene_fps)

    if sampling_mode == SAMPLING_MODE_DENSE_SAMPLED:
        if not fps:
            raise ValueError("DENSE_SAMPLED mode requires a positive `fps` value")
        start_sec = scene.frame_start / scene_fps
        end_sec = scene.frame_end / scene_fps
        step = 1.0 / fps
        n_steps = int(math.floor((end_sec - start_sec) / step)) + 1
        return [start_sec + i * step for i in range(max(n_steps, 0))]

    raise ValueError(f"Unknown sampling_mode: {sampling_mode!r}")


def evaluate_object_at_time(obj, scene, time_sec: float):
    """Move the playhead to `time_sec` and return the object evaluated at that frame."""
    import bpy

    scene_fps = scene.render.fps / scene.render.fps_base
    scene.frame_set(int(round(time_sec * scene_fps)))
    depsgraph = bpy.context.evaluated_depsgraph_get()
    return obj.evaluated_get(depsgraph)
