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


def shift_keyframes_to_frames(obj, frame_mapping: dict) -> int:
    """Move every keyframe point at an `old_frame` in `frame_mapping` (int ->
    int) to its `new_frame`, translating Bezier handles by the same delta so
    handle shape (hence in/out tangent) is preserved (spec section 3.5's
    Auto-Fix Timeline Timing operator).

    Processes old frames in descending `new_frame` order: since Auto-Fix only
    ever stretches (each new_frame >= its old_frame) and preserves relative
    ordering, moving the keyframe that ends up latest first guarantees no
    not-yet-processed keyframe is ever overwritten or crossed.

    Returns the number of keyframe points moved (0 if the object has no
    animation data).
    """
    anim = obj.animation_data
    if anim is None or anim.action is None:
        return 0

    action_slot = getattr(anim, "action_slot", None)
    ordered_old_frames = sorted(frame_mapping, key=lambda f: frame_mapping[f], reverse=True)

    moved = 0
    for fcurve in _iter_action_fcurves(anim.action, action_slot):
        for old_frame in ordered_old_frames:
            new_frame = frame_mapping[old_frame]
            delta = new_frame - old_frame
            if delta == 0:
                continue
            for kp in fcurve.keyframe_points:
                if abs(kp.co.x - old_frame) < 0.5:
                    kp.co.x += delta
                    kp.handle_left.x += delta
                    kp.handle_right.x += delta
                    moved += 1
        fcurve.update()
    return moved


def shift_timeline_markers_to_frames(scene, frame_mapping: dict) -> int:
    """Move every timeline marker at an `old_frame` in `frame_mapping` to its
    `new_frame`, so shape names (looked up by marker frame) stay attached to
    the keyframe they named after Auto-Fix Timeline Timing shifts it.
    Returns the number of markers moved."""
    moved = 0
    for old_frame in sorted(frame_mapping, key=lambda f: frame_mapping[f], reverse=True):
        new_frame = frame_mapping[old_frame]
        if new_frame == old_frame:
            continue
        for marker in scene.timeline_markers:
            if marker.frame == old_frame:
                marker.frame = new_frame
                moved += 1
    return moved
