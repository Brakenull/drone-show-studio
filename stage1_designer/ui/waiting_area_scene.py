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
from .holding_area_scene import (
    area_names,
    build_area_meshes,
    ensure_area_objects,
    existing_indices,
    place_box,
    remove_area_objects,
    remove_named,
)

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
    lo, hi = waiting_area.waiting_region_bounds(count, area)
    slot_positions = waiting_area.compute_waiting_positions(count, area)
    build_area_meshes(volume, slots, clearance, area.center, lo, hi, area.show_clearance_m, slot_positions, safety_radius)


def _names(i: int):
    return area_names(_PREFIX, i)


def remove_objects() -> None:
    remove_area_objects(_PREFIX, COLLECTION_NAME)


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
        for i in existing_indices(_PREFIX) - set(range(len(areas))):
            remove_named(_names(i)[::-1])
        for i, (area, count) in enumerate(zip(areas, counts)):
            volume, slots, clearance = ensure_area_objects(scene, COLLECTION_NAME, _names(i), config.COLOR_WAITING_RGBA)
            key = _geometry_key(settings, i, count)
            if volume.get(_GEOMETRY_KEY) != key:
                _build_meshes(volume, slots, clearance, area, count, settings.safety_radius_m)
                volume[_GEOMETRY_KEY] = key
            place_box(volume, area.center, settings.heading_offset_deg)
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
