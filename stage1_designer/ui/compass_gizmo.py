"""True North compass gizmo overlay."""

import math

import blf
import bpy
import gpu
from gpu_extras.batch import batch_for_shader

_draw_handle = None
_MARGIN = 70
_ARROW_LENGTH = 40


def _rotated(dx, dy, theta_rad):
    c, s = math.cos(theta_rad), math.sin(theta_rad)
    return dx * c - dy * s, dx * s + dy * c


def draw_callback_2d():
    context = bpy.context
    scene = context.scene
    settings = getattr(scene, "drone_show_settings", None)
    if settings is None or not settings.show_compass_gizmo:
        return

    region = context.region
    if region is None:
        return

    origin_x = region.width - _MARGIN
    origin_y = region.height - _MARGIN

    # heading_offset_deg: clockwise angle from Blender +Y to True North.
    theta = math.radians(settings.heading_offset_deg)

    shader = gpu.shader.from_builtin("UNIFORM_COLOR")
    gpu.state.line_width_set(2.0)
    gpu.state.blend_set("ALPHA")

    # North arrow (red): points from Blender +Y rotated clockwise by theta.
    nx, ny = _rotated(0.0, _ARROW_LENGTH, theta)
    north_line = [(origin_x, origin_y), (origin_x + nx, origin_y + ny)]
    batch = batch_for_shader(shader, "LINES", {"pos": north_line})
    shader.uniform_float("color", (1.0, 0.2, 0.2, 1.0))
    batch.draw(shader)

    # East arrow (blue): 90 deg clockwise from North.
    ex, ey = _rotated(_ARROW_LENGTH * 0.6, 0.0, theta)
    east_line = [(origin_x, origin_y), (origin_x + ex, origin_y + ey)]
    batch = batch_for_shader(shader, "LINES", {"pos": east_line})
    shader.uniform_float("color", (0.2, 0.4, 1.0, 1.0))
    batch.draw(shader)

    gpu.state.blend_set("NONE")

    blf.position(0, origin_x + nx * 1.1 - 4, origin_y + ny * 1.1 - 4, 0)
    blf.size(0, 14)
    blf.color(0, 1.0, 0.2, 0.2, 1.0)
    blf.draw(0, "N")

    blf.position(0, origin_x + ex * 1.2 - 4, origin_y + ey * 1.2 - 4, 0)
    blf.color(0, 0.2, 0.4, 1.0, 1.0)
    blf.draw(0, "E")


def enable():
    global _draw_handle
    if _draw_handle is not None:
        return
    _draw_handle = bpy.types.SpaceView3D.draw_handler_add(draw_callback_2d, (), "WINDOW", "POST_PIXEL")


def disable():
    global _draw_handle
    if _draw_handle is None:
        return
    bpy.types.SpaceView3D.draw_handler_remove(_draw_handle, "WINDOW")
    _draw_handle = None


def register():
    enable()


def unregister():
    disable()
