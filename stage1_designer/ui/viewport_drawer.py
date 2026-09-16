"""Real-time viewport safety overlay (spec section 3.5).

Draws a wireframe sphere (three orthogonal great-circles) around every cached
sample point, colored green when its nearest-neighbour distance satisfies
`min_distance_m`, or blinking red otherwise. The point cache is populated by
`DSS_OT_PreviewSample` (running the actual sampler is too expensive to redo on
every viewport redraw), and the overlay itself just visualizes that cache.
"""

import math
import time

import bpy
import gpu
import numpy as np
from gpu_extras.batch import batch_for_shader

from .. import config

_draw_handle = None

_state = {
    "points": np.zeros((0, 3), dtype=float),
    "violation": np.zeros((0,), dtype=bool),
    "radius": config.SAFETY_RADIUS_M,
    "holding_points": np.zeros((0, 3), dtype=float),
}


def set_preview_points(points: np.ndarray, min_distance_m: float, safety_radius_m: float) -> None:
    """Update the cached point cloud used by the overlay, flagging violations."""
    points = np.asarray(points, dtype=float)
    n = len(points)
    violation = np.zeros((n,), dtype=bool)
    if n > 1:
        diffs = points[:, None, :] - points[None, :, :]
        dists = np.linalg.norm(diffs, axis=-1)
        np.fill_diagonal(dists, np.inf)
        nearest = dists.min(axis=1)
        violation = nearest < min_distance_m

    _state["points"] = points
    _state["violation"] = violation
    _state["radius"] = safety_radius_m


def set_holding_preview_points(points: np.ndarray) -> None:
    """Update the cached holding-area point cloud (drawn in a neutral color;
    these are guaranteed collision-free by construction, so no violation check)."""
    _state["holding_points"] = np.asarray(points, dtype=float)


def _circle_points(center, radius, axis, segments=24):
    pts = []
    for i in range(segments + 1):
        angle = 2.0 * math.pi * i / segments
        c, s = math.cos(angle), math.sin(angle)
        if axis == "xy":
            offset = (radius * c, radius * s, 0.0)
        elif axis == "xz":
            offset = (radius * c, 0.0, radius * s)
        else:  # yz
            offset = (0.0, radius * c, radius * s)
        pts.append((center[0] + offset[0], center[1] + offset[1], center[2] + offset[2]))
    return pts


def _wireframe_sphere_lines(center, radius):
    lines = []
    for axis in ("xy", "xz", "yz"):
        ring = _circle_points(center, radius, axis)
        for a, b in zip(ring[:-1], ring[1:]):
            lines.append(a)
            lines.append(b)
    return lines


def draw_callback_3d():
    points = _state["points"]
    holding_points = _state["holding_points"]
    if len(points) == 0 and len(holding_points) == 0:
        return

    radius = _state["radius"]
    violation = _state["violation"]

    blink = 0.5 + 0.5 * math.sin(time.time() * 8.0)

    shader = gpu.shader.from_builtin("UNIFORM_COLOR")
    gpu.state.line_width_set(1.5)
    gpu.state.blend_set("ALPHA")

    green_lines = []
    red_lines = []
    for i, p in enumerate(points):
        lines = _wireframe_sphere_lines(p, radius)
        if violation[i]:
            red_lines.extend(lines)
        else:
            green_lines.extend(lines)

    if green_lines:
        batch = batch_for_shader(shader, "LINES", {"pos": green_lines})
        shader.uniform_float("color", config.COLOR_VALID_RGBA)
        batch.draw(shader)

    if red_lines:
        batch = batch_for_shader(shader, "LINES", {"pos": red_lines})
        r, g, b, a = config.COLOR_VIOLATION_RGBA
        shader.uniform_float("color", (r, g, b, a * blink))
        batch.draw(shader)

    if len(holding_points) > 0:
        holding_lines = []
        for p in holding_points:
            holding_lines.extend(_wireframe_sphere_lines(p, radius))
        batch = batch_for_shader(shader, "LINES", {"pos": holding_lines})
        shader.uniform_float("color", config.COLOR_HOLDING_RGBA)
        batch.draw(shader)

    gpu.state.blend_set("NONE")


def enable():
    global _draw_handle
    if _draw_handle is not None:
        return
    _draw_handle = bpy.types.SpaceView3D.draw_handler_add(draw_callback_3d, (), "WINDOW", "POST_VIEW")


def disable():
    global _draw_handle
    if _draw_handle is None:
        return
    bpy.types.SpaceView3D.draw_handler_remove(_draw_handle, "WINDOW")
    _draw_handle = None


def register():
    pass


def unregister():
    disable()
