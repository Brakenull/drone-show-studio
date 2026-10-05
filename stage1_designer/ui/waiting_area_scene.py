"""Waiting areas as real scene objects ("Create in Scene").

One set of objects per waiting area `n` (1-based, in list order):

  DSS Waiting Areas (collection)
  ├── DSS_WaitingArea_<n>_Volume     wire box: the waiting region (the slot
  │                                  layer padded by half a grid step); grab
  │                                  it (G) to move the area
  ├── DSS_WaitingArea_<n>_Slots      one sphere of R_safe per waiting slot,
  │                                  parented to the box
  └── DSS_WaitingArea_<n>_Clearance  yellow wire box: the region grown by the
                                     area's safe distance to the show

Same conventions as `holding_area_scene`: areas are declared in ENU, the box
sits at `blender_from_enu(center)` turned by the heading offset, moving it
writes the new center back (settings otherwise win), rotation and scale are
locked. The slot count of each area comes from the last sampling pass (the
most spare drones at any keyframe, `set_spare_needed`).
"""

import bpy
import numpy as np

from .. import config
from ..core import timeline_sampler, waiting_area
from .holding_area_scene import _ICO_FACES, _ICO_VERTS, _box_mesh, _set_mesh

COLLECTION_NAME = "DSS Waiting Areas"
_GEOMETRY_KEY = "dss_geometry_key"
_EPS = 1e-4
_PREFIX = "DSS_WaitingArea_"

_syncing = False
# The most spare drones at any keyframe, from the last full sampling pass
# (Check Kinematics / Auto-Fix / Export). 0 until then: every area shows its
# declared grid.
_needed = {"spare": 0}


def set_spare_needed(spare: int) -> None:
    _needed["spare"] = max(int(spare), 0)


def spare_needed() -> int:
    return _needed["spare"]


def _names(i: int):
    base = f"{_PREFIX}{i + 1}_"
    return base + "Volume", base + "Slots", base + "Clearance"


def areas_of(settings):
    return [
        waiting_area.WaitingArea(
            tuple(float(v) for v in a.center),
            tuple(float(v) for v in a.size),
            float(a.grid_spacing_m),
            float(a.show_clearance_m),
        )
        for a in settings.waiting_areas
    ]


def slot_counts(settings, spare=None):
    """Slots each area lays out (exported as `slot_count`)."""
    return waiting_area.allocate_slot_counts(areas_of(settings), spare_needed() if spare is None else spare)


def _geometry_key(settings, i: int, count: int) -> str:
    a = settings.waiting_areas[i]
    return repr(
        (
            round(a.center[2], 6),
            tuple(round(v, 6) for v in a.size),
            round(a.grid_spacing_m, 6),
            round(a.show_clearance_m, 6),
            int(count),
            round(settings.safety_radius_m, 6),
        )
    )


def _build_meshes(volume, slots, clearance, area, count, safety_radius) -> None:
    center = np.asarray(area.center, dtype=float)
    lo, hi = waiting_area.waiting_region_bounds(count, area)
    lo, hi = lo - center, hi - center
    _box_mesh(volume.data, lo, hi)
    c = area.show_clearance_m
    if c > 0.0:
        _box_mesh(clearance.data, lo - c, hi + c)
    else:
        clearance.data.clear_geometry()
        clearance.data.update()
    local = waiting_area.compute_waiting_positions(count, area) - center
    verts = (local[:, None, :] + _ICO_VERTS[None, :, :] * safety_radius).reshape(-1, 3)
    faces = (_ICO_FACES[None, :, :] + (np.arange(len(local)) * len(_ICO_VERTS))[:, None, None]).reshape(-1, 3)
    _set_mesh(slots.data, verts, faces)


def _ensure_collection(scene):
    coll = bpy.data.collections.get(COLLECTION_NAME)
    if coll is None:
        coll = bpy.data.collections.new(COLLECTION_NAME)
    if coll.name not in scene.collection.children:
        scene.collection.children.link(coll)
    return coll


def _ensure_object(name, coll):
    obj = bpy.data.objects.get(name)
    if obj is None or obj.type != "MESH":
        obj = bpy.data.objects.new(name, bpy.data.meshes.new(name))
    if obj.name not in coll.objects:
        coll.objects.link(obj)
    return obj


def _ensure_area_objects(scene, i: int):
    coll = _ensure_collection(scene)
    volume_name, slots_name, clearance_name = _names(i)
    volume = _ensure_object(volume_name, coll)
    slots = _ensure_object(slots_name, coll)
    clearance = _ensure_object(clearance_name, coll)
    color = config.COLOR_WAITING_RGBA
    volume.display_type = "WIRE"
    volume.show_in_front = True
    volume.hide_render = True
    volume.color = color
    volume.lock_rotation = (True, True, True)
    volume.lock_scale = (True, True, True)
    for child in (slots, clearance):
        if child.parent is not volume:
            child.parent = volume
            child.matrix_parent_inverse.identity()
        child.location = (0.0, 0.0, 0.0)
        child.rotation_euler = (0.0, 0.0, 0.0)
        child.scale = (1.0, 1.0, 1.0)
        child.hide_render = True
        child.hide_select = True
    slots.color = color
    slots.display_type = "SOLID"
    clearance.color = config.COLOR_KINEMATIC_WARNING_RGBA
    clearance.display_type = "WIRE"
    return volume, slots, clearance


def _existing_indices():
    found = set()
    for obj in bpy.data.objects:
        if obj.name.startswith(_PREFIX):
            head = obj.name[len(_PREFIX):].split("_", 1)[0]
            if head.isdigit():
                found.add(int(head) - 1)
    return found


def _remove_area(i: int) -> None:
    for name in _names(i)[::-1]:
        obj = bpy.data.objects.get(name)
        if obj is not None:
            mesh = obj.data
            bpy.data.objects.remove(obj, do_unlink=True)
            if mesh is not None and mesh.users == 0:
                bpy.data.meshes.remove(mesh)


def remove_objects() -> None:
    for i in sorted(_existing_indices()):
        _remove_area(i)
    coll = bpy.data.collections.get(COLLECTION_NAME)
    if coll is not None and len(coll.all_objects) == 0:
        bpy.data.collections.remove(coll)


def sync_objects(scene) -> None:
    """Rebuild / re-place every area's objects from the settings (settings win)."""
    global _syncing
    settings = getattr(scene, "drone_show_settings", None)
    if settings is None or _syncing:
        return
    _syncing = True
    try:
        if not settings.show_waiting_area_objects or not len(settings.waiting_areas):
            remove_objects()
            return
        areas = areas_of(settings)
        counts = slot_counts(settings)
        for i in _existing_indices() - set(range(len(areas))):
            _remove_area(i)
        heading = settings.heading_offset_deg
        rot_z = np.radians(heading)
        for i, (area, count) in enumerate(zip(areas, counts)):
            volume, slots, clearance = _ensure_area_objects(scene, i)
            key = _geometry_key(settings, i, count)
            if volume.get(_GEOMETRY_KEY) != key:
                _build_meshes(volume, slots, clearance, area, count, settings.safety_radius_m)
                volume[_GEOMETRY_KEY] = key
            target = timeline_sampler.blender_from_enu(area.center, heading)[0]
            if np.abs(np.asarray(volume.location) - target).max() > _EPS:
                volume.location = target
            if abs(volume.rotation_euler.z - rot_z) > 1e-6 or volume.rotation_euler.x or volume.rotation_euler.y:
                volume.rotation_euler = (0.0, 0.0, rot_z)
    finally:
        _syncing = False


def on_settings_changed(self, context):
    """`update=` callback for every property the waiting-area objects use."""
    if context is not None and context.scene is not None:
        sync_objects(context.scene)


def _pull_from_objects(scene) -> None:
    """A box was moved in the viewport: write its center back (objects win).
    A deleted box turns the toggle off and clears the rest, like the holding
    area's."""
    global _syncing
    settings = getattr(scene, "drone_show_settings", None)
    if settings is None or not settings.show_waiting_area_objects or _syncing:
        return
    for i, area in enumerate(settings.waiting_areas):
        volume = bpy.data.objects.get(_names(i)[0])
        if volume is None or volume.name not in scene.objects:
            _syncing = True
            try:
                settings.show_waiting_area_objects = False
                remove_objects()
            finally:
                _syncing = False
            return
        new_center = timeline_sampler.enu_transform(tuple(volume.location), settings.heading_offset_deg)[0]
        if np.abs(new_center - np.asarray(area.center, dtype=float)).max() <= _EPS:
            continue
        _syncing = True
        try:
            area.center = tuple(float(v) for v in new_center)
            volume[_GEOMETRY_KEY] = _geometry_key(settings, i, slot_counts(settings)[i])
        finally:
            _syncing = False


@bpy.app.handlers.persistent
def _on_depsgraph_update_post(scene, depsgraph):
    if not depsgraph.id_type_updated("OBJECT"):
        return
    _pull_from_objects(scene)


def register():
    if _on_depsgraph_update_post not in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.append(_on_depsgraph_update_post)


def unregister():
    if _on_depsgraph_update_post in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.remove(_on_depsgraph_update_post)
