"""Ground level as a scene object (spec section 3.9, "Show in Scene").

  DSS Ground (collection)
  └── DSS_Ground   wire grid at the ground level, 10 m cells, covering the
                   holding area and the last sampled show plus a margin

The extent is worked out in ENU and the vertices are placed back in Blender
space with the heading offset, like the holding area (holding_area_scene.py).
The grid is display-only: not selectable, not rendered.
"""

import bpy
import numpy as np

from .. import config
from ..core import holding_area, timeline_sampler

COLLECTION_NAME = "DSS Ground"
OBJECT_NAME = "DSS_Ground"
_CELL_M = 10.0
_MARGIN_M = 20.0
_MIN_HALF_SPAN_M = 50.0

# ENU (x, y) bounds of every formation point from the last full sampling pass,
# so the grid also covers the show, not just the holding area.
_show_extent = {"lo": None, "hi": None}


def set_show_extent(point_sets) -> None:
    pts = [np.asarray(p, dtype=float)[:, :2] for p in point_sets if len(p)]
    if not pts:
        _show_extent.update(lo=None, hi=None)
        return
    stacked = np.vstack(pts)
    _show_extent.update(lo=stacked.min(axis=0), hi=stacked.max(axis=0))


def _grid_extent(settings):
    from .holding_area_scene import layout_args

    lo, hi = holding_area.holding_region_bounds(*layout_args(settings))
    lo, hi = lo[:2], hi[:2]
    if _show_extent["lo"] is not None:
        lo = np.minimum(lo, _show_extent["lo"])
        hi = np.maximum(hi, _show_extent["hi"])
    center = (lo + hi) / 2.0
    half = np.maximum((hi - lo) / 2.0 + _MARGIN_M, _MIN_HALF_SPAN_M)
    # Whole cells, so the grid lines land on round numbers from the center.
    half = np.ceil(half / _CELL_M) * _CELL_M
    return center - half, center + half


def _build_grid(mesh, settings) -> None:
    lo, hi = _grid_extent(settings)
    nx = int(round((hi[0] - lo[0]) / _CELL_M))
    ny = int(round((hi[1] - lo[1]) / _CELL_M))
    xs = np.linspace(lo[0], hi[0], nx + 1)
    ys = np.linspace(lo[1], hi[1], ny + 1)
    gx, gy = np.meshgrid(xs, ys)  # row-major: vertex (j, i) = j * (nx + 1) + i
    enu = np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, settings.ground_z_m)])
    verts = timeline_sampler.blender_from_enu(enu, settings.heading_offset_deg)

    faces = []
    for j in range(ny):
        for i in range(nx):
            a = j * (nx + 1) + i
            faces.append((a, a + 1, a + nx + 2, a + nx + 1))

    mesh.clear_geometry()
    mesh.from_pydata(verts.tolist(), [], faces)
    mesh.update()


def sync(scene) -> None:
    settings = getattr(scene, "drone_show_settings", None)
    if settings is None:
        return
    if not settings.show_ground_object:
        remove()
        return

    coll = bpy.data.collections.get(COLLECTION_NAME)
    if coll is None:
        coll = bpy.data.collections.new(COLLECTION_NAME)
    if coll.name not in scene.collection.children:
        scene.collection.children.link(coll)
    obj = bpy.data.objects.get(OBJECT_NAME)
    if obj is None or obj.type != "MESH":
        obj = bpy.data.objects.new(OBJECT_NAME, bpy.data.meshes.new(OBJECT_NAME))
    if obj.name not in coll.objects:
        coll.objects.link(obj)

    obj.display_type = "WIRE"
    obj.hide_render = True
    obj.hide_select = True
    obj.color = config.COLOR_GROUND_RGBA
    obj.location = (0.0, 0.0, 0.0)
    obj.rotation_euler = (0.0, 0.0, 0.0)
    obj.scale = (1.0, 1.0, 1.0)
    _build_grid(obj.data, settings)


def remove() -> None:
    obj = bpy.data.objects.get(OBJECT_NAME)
    if obj is not None:
        mesh = obj.data
        bpy.data.objects.remove(obj, do_unlink=True)
        if mesh is not None and mesh.users == 0:
            bpy.data.meshes.remove(mesh)
    coll = bpy.data.collections.get(COLLECTION_NAME)
    if coll is not None and len(coll.all_objects) == 0:
        bpy.data.collections.remove(coll)


def on_settings_changed(self, context):
    if context is not None and context.scene is not None:
        sync(context.scene)

