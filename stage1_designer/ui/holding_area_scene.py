"""Holding area as real scene objects (spec section 3.2, "Create in Scene").

The GPU overlay (`viewport_drawer`) only previews the leftover parked drones.
This module builds editable geometry instead, so a designer can see the
takeoff volume next to the formations and keep them out of it (bug-report
P1-01):

  DSS Holding Area (collection)
  ├── DSS_HoldingArea_Volume     wire box: the holding region the clearance
  │                              check uses (parked grid padded by half a grid
  │                              step); grab it (G) to move the holding area
  ├── DSS_HoldingArea_Slots      one sphere of R_safe per launch slot, for the
  │                              whole fleet (Stage 2 launches every drone
  │                              from here), parented to the box
  └── DSS_HoldingArea_Clearance  yellow wire box: the region grown by the safe
                                 distance to the show (section 3.2.2); formation
                                 points should stay outside it

The holding area is declared in ENU, so the box sits at
`blender_from_enu(center)` and is turned by the heading offset. Moving the box
writes the new center (and shifts `max_height` by the same dz) back into the
settings; rotation and scale are locked because they have no meaning in the
settings (size is edited in the panel).
"""

import bpy
import numpy as np

from .. import config
from ..core import holding_area, timeline_sampler

COLLECTION_NAME = "DSS Holding Area"
VOLUME_NAME = "DSS_HoldingArea_Volume"
SLOTS_NAME = "DSS_HoldingArea_Slots"
CLEARANCE_NAME = "DSS_HoldingArea_Clearance"
_GEOMETRY_KEY = "dss_geometry_key"
_EPS = 1e-4

# Set while this module writes to the settings or the objects, so the update
# callbacks and the depsgraph handler don't feed back into each other.
_syncing = False

# Unit icosahedron: 12 vertices, 20 faces - enough to read as a sphere at the
# scale of a whole fleet while staying cheap for 1000+ drones.
_PHI = (1.0 + 5.0 ** 0.5) / 2.0
_ICO_VERTS = np.array(
    [
        (-1, _PHI, 0), (1, _PHI, 0), (-1, -_PHI, 0), (1, -_PHI, 0),
        (0, -1, _PHI), (0, 1, _PHI), (0, -1, -_PHI), (0, 1, -_PHI),
        (_PHI, 0, -1), (_PHI, 0, 1), (-_PHI, 0, -1), (-_PHI, 0, 1),
    ],
    dtype=float,
)
_ICO_VERTS /= np.linalg.norm(_ICO_VERTS[0])
_ICO_FACES = np.array(
    [
        (0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11),
        (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
        (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9),
        (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1),
    ],
    dtype=int,
)


def _layout_args(settings):
    ha = settings.holding_area
    return (
        settings.fleet_size,
        tuple(ha.center),
        tuple(ha.size),
        ha.max_height,
        ha.grid_spacing_m,
    )


def layout_for(settings) -> holding_area.HoldingLayout:
    return holding_area.compute_holding_layout(*_layout_args(settings))


def _geometry_key(settings) -> str:
    """Everything the meshes depend on. The center is left out on purpose:
    the meshes are built relative to it, so moving the box needs no rebuild."""
    ha = settings.holding_area
    return repr(
        (
            settings.fleet_size,
            round(ha.center[2], 6),
            tuple(round(v, 6) for v in ha.size),
            round(ha.max_height, 6),
            round(ha.grid_spacing_m, 6),
            round(settings.safety_radius_m, 6),
            round(ha.show_clearance_m, 6),
        )
    )


def _set_mesh(mesh, verts, faces) -> None:
    mesh.clear_geometry()
    mesh.from_pydata(np.asarray(verts).tolist(), [], np.asarray(faces).tolist())
    mesh.update()


def _box_mesh(mesh, lo, hi) -> None:
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    verts = [
        (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
        (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1),
    ]
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    _set_mesh(mesh, verts, faces)


def _build_region_meshes(volume_mesh, clearance_mesh, settings) -> None:
    """Holding region (and its safe-distance zone) in the box's local frame,
    i.e. ENU offsets from the center."""
    lo, hi = holding_area.holding_region_bounds(*_layout_args(settings))
    center = np.asarray(settings.holding_area.center, dtype=float)
    lo, hi = lo - center, hi - center
    _box_mesh(volume_mesh, lo, hi)
    c = settings.holding_area.show_clearance_m
    if c > 0.0:
        # The true zone has rounded edges/corners; the box around it is the
        # simple, conservative picture of it.
        _box_mesh(clearance_mesh, lo - c, hi + c)
    else:
        clearance_mesh.clear_geometry()
        clearance_mesh.update()


def _build_slots_mesh(mesh, settings) -> None:
    fleet_size, center, size, max_height, spacing = _layout_args(settings)
    slots = holding_area.compute_holding_positions(fleet_size, center, size, max_height, spacing)
    local = slots - np.asarray(center, dtype=float)  # ENU offsets; the box carries the heading
    verts = (local[:, None, :] + _ICO_VERTS[None, :, :] * settings.safety_radius_m).reshape(-1, 3)
    faces = (_ICO_FACES[None, :, :] + (np.arange(len(local)) * len(_ICO_VERTS))[:, None, None]).reshape(-1, 3)
    _set_mesh(mesh, verts, faces)


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


def _create(scene, settings):
    coll = _ensure_collection(scene)
    volume = _ensure_object(VOLUME_NAME, coll)
    slots = _ensure_object(SLOTS_NAME, coll)
    clearance = _ensure_object(CLEARANCE_NAME, coll)

    color = config.COLOR_HOLDING_RGBA
    volume.display_type = "WIRE"
    volume.show_in_front = True
    volume.hide_render = True
    volume.color = color
    volume.lock_rotation = (True, True, True)
    volume.lock_scale = (True, True, True)

    for child in (slots, clearance):
        child.parent = volume
        child.matrix_parent_inverse.identity()
        child.location = (0.0, 0.0, 0.0)
        child.rotation_euler = (0.0, 0.0, 0.0)
        child.scale = (1.0, 1.0, 1.0)
        child.hide_render = True
        child.hide_select = True  # clicks go to the box, which is what moves the area
    slots.color = color
    slots.display_type = "SOLID"
    clearance.color = config.COLOR_KINEMATIC_WARNING_RGBA
    clearance.display_type = "WIRE"

    settings.holding_area_object = volume
    return volume


def sync_objects(scene) -> None:
    """Rebuild / re-place the scene objects from the settings (settings win)."""
    global _syncing
    settings = getattr(scene, "drone_show_settings", None)
    if settings is None or _syncing:
        return
    _syncing = True
    try:
        if not settings.show_holding_area_object:
            remove_objects(settings)
            return

        volume = settings.holding_area_object
        if volume is None or volume.name not in scene.objects:
            volume = _create(scene, settings)
        slots = _child(volume, SLOTS_NAME)
        clearance = _child(volume, CLEARANCE_NAME)
        if slots is None or clearance is None:
            volume = _create(scene, settings)
            slots = _child(volume, SLOTS_NAME)
            clearance = _child(volume, CLEARANCE_NAME)
            volume.pop(_GEOMETRY_KEY, None)

        key = _geometry_key(settings)
        if volume.get(_GEOMETRY_KEY) != key:
            _build_region_meshes(volume.data, clearance.data, settings)
            _build_slots_mesh(slots.data, settings)
            volume[_GEOMETRY_KEY] = key

        heading = settings.heading_offset_deg
        target = timeline_sampler.blender_from_enu(tuple(settings.holding_area.center), heading)[0]
        if np.abs(np.asarray(volume.location) - target).max() > _EPS:
            volume.location = target
        rot_z = np.radians(heading)
        if abs(volume.rotation_euler.z - rot_z) > 1e-6 or volume.rotation_euler.x or volume.rotation_euler.y:
            volume.rotation_euler = (0.0, 0.0, rot_z)
        if tuple(volume.scale) != (1.0, 1.0, 1.0):
            volume.scale = (1.0, 1.0, 1.0)
    finally:
        _syncing = False


def _child(volume, name):
    return next((c for c in volume.children if c.name.startswith(name)), None)


def remove_objects(settings) -> None:
    for name in (SLOTS_NAME, CLEARANCE_NAME, VOLUME_NAME):
        obj = bpy.data.objects.get(name)
        if obj is not None:
            mesh = obj.data
            bpy.data.objects.remove(obj, do_unlink=True)
            if mesh is not None and mesh.users == 0:
                bpy.data.meshes.remove(mesh)
    coll = bpy.data.collections.get(COLLECTION_NAME)
    if coll is not None and len(coll.all_objects) == 0:
        bpy.data.collections.remove(coll)
    if settings.holding_area_object is not None:
        settings.holding_area_object = None


def on_settings_changed(self, context):
    """`update=` callback for every property the holding-area objects use."""
    if context is not None and context.scene is not None:
        sync_objects(context.scene)


def _pull_from_objects(scene) -> None:
    """The box was moved in the viewport: write its position back to the
    settings (objects win). Moving on Z carries `max_height` along."""
    global _syncing
    settings = getattr(scene, "drone_show_settings", None)
    if settings is None or not settings.show_holding_area_object or _syncing:
        return
    volume = settings.holding_area_object
    if volume is None or volume.name not in scene.objects:
        # Deleted by the user: switch the toggle off instead of recreating it,
        # and clear what's left (the slots, the collection).
        _syncing = True
        try:
            settings.show_holding_area_object = False
            remove_objects(settings)
        finally:
            _syncing = False
        return

    ha = settings.holding_area
    new_center = timeline_sampler.enu_transform(tuple(volume.location), settings.heading_offset_deg)[0]
    old_center = np.asarray(ha.center, dtype=float)
    if np.abs(new_center - old_center).max() <= _EPS:
        return

    _syncing = True
    try:
        ha.max_height += float(new_center[2] - old_center[2])
        ha.center = tuple(float(v) for v in new_center)
        # Box height is unchanged, so the meshes stay valid; record that.
        volume[_GEOMETRY_KEY] = _geometry_key(settings)
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
