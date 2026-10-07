"""Holding areas as real scene objects ("Create in Scene").

The GPU overlay (`viewport_drawer`) only previews the leftover parked drones.
This module builds editable geometry instead, so a designer can see the
takeoff volumes next to the formations and keep them out of them. One set of
objects per holding area `n` (1-based, in list order):

  DSS Holding Area (collection)
  ├── DSS_HoldingArea_<n>_Volume     wire box: the area's region the clearance
  │                                  check uses (parked grid padded by half a
  │                                  grid step); grab it (G) to move the area
  ├── DSS_HoldingArea_<n>_Slots      one sphere of R_safe per launch slot of
  │                                  the drones whose home it is, parented to
  │                                  the box
  └── DSS_HoldingArea_<n>_Clearance  yellow wire box: the region grown by the
                                     safe distance to the show; formation
                                     points should stay outside it

Each area is declared in ENU, so its box sits at `blender_from_enu(center)`
and is turned by the heading offset. Moving a box writes the new center (and
shifts `max_height` by the same dz) back into that area's settings; rotation
and scale are locked because they have no meaning in the settings (size is
edited in the panel).

The object helpers here are shared with the waiting areas' objects.
"""

import bpy
import numpy as np

from .. import config
from ..core import holding_area, timeline_sampler

COLLECTION_NAME = "DSS Holding Area"
_PREFIX = "DSS_HoldingArea_"
# Before the holding area became a list, its objects had no number.
_LEGACY_NAMES = ("DSS_HoldingArea_Slots", "DSS_HoldingArea_Clearance", "DSS_HoldingArea_Volume")
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


def area_props(settings):
    """The holding areas' property groups, in list order. A scene that never
    added a second area (or a .blend saved before the list existed) keeps its
    one area in `settings.holding_area`."""
    return list(settings.holding_areas) or [settings.holding_area]


def areas_of(settings):
    return [
        holding_area.HoldingArea(
            tuple(float(v) for v in a.center),
            tuple(float(v) for v in a.size),
            float(a.max_height),
            float(a.grid_spacing_m),
            float(a.layer_spacing_m),
            bool(a.staggered_layers),
            float(a.show_clearance_m),
        )
        for a in area_props(settings)
    ]


def slot_counts(settings):
    """Drones per area, in list order (exported as `slot_count`)."""
    return holding_area.allocate_holding_counts(areas_of(settings), settings.fleet_size)


def layouts_for(settings):
    """Each area's resolved layout for its own drones."""
    return [holding_area.compute_holding_layout(n, *a.args) for a, n in zip(areas_of(settings), slot_counts(settings))]


def regions_for(settings):
    """One (lo, hi) ENU box per area."""
    return holding_area.holding_regions(areas_of(settings), slot_counts(settings))


def area_names(prefix: str, i: int):
    base = f"{prefix}{i + 1}_"
    return base + "Volume", base + "Slots", base + "Clearance"


def _geometry_key(area, count: int, safety_radius_m: float) -> str:
    """Everything an area's meshes depend on. The center's X / Y are left out
    on purpose: the meshes are built relative to it, so moving the box needs
    no rebuild."""
    return repr(
        (
            int(count),
            round(area.center[2], 6),
            tuple(round(v, 6) for v in area.size),
            round(area.max_height, 6),
            round(area.grid_spacing_m, 6),
            round(area.layer_spacing_m, 6),
            bool(area.staggered_layers),
            round(safety_radius_m, 6),
            round(area.show_clearance_m, 6),
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


def build_area_meshes(volume, slots, clearance, center, lo, hi, clearance_m, slot_positions, safety_radius) -> None:
    """An area's region box, its safe-distance box and one sphere per slot, in
    the box's local frame (ENU offsets from `center`; the box carries the
    heading)."""
    center = np.asarray(center, dtype=float)
    lo, hi = lo - center, hi - center
    _box_mesh(volume.data, lo, hi)
    if clearance_m > 0.0:
        # The true zone has rounded edges/corners; the box around it is the
        # simple, conservative picture of it.
        _box_mesh(clearance.data, lo - clearance_m, hi + clearance_m)
    else:
        clearance.data.clear_geometry()
        clearance.data.update()
    local = np.asarray(slot_positions, dtype=float).reshape(-1, 3) - center
    verts = (local[:, None, :] + _ICO_VERTS[None, :, :] * safety_radius).reshape(-1, 3)
    faces = (_ICO_FACES[None, :, :] + (np.arange(len(local)) * len(_ICO_VERTS))[:, None, None]).reshape(-1, 3)
    _set_mesh(slots.data, verts, faces)


def _ensure_collection(scene, name):
    coll = bpy.data.collections.get(name)
    if coll is None:
        coll = bpy.data.collections.new(name)
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


def ensure_area_objects(scene, collection_name, names, color):
    """(volume, slots, clearance) of one area: the box, and the slots and
    safe-distance box parented to it (clicks go to the box, which is what
    moves the area)."""
    coll = _ensure_collection(scene, collection_name)
    volume, slots, clearance = (_ensure_object(name, coll) for name in names)
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


def existing_indices(prefix):
    """Area indices that have objects named `<prefix><n>_...`."""
    found = set()
    for obj in bpy.data.objects:
        if obj.name.startswith(prefix):
            head = obj.name[len(prefix):].split("_", 1)[0]
            if head.isdigit():
                found.add(int(head) - 1)
    return found


def remove_named(names) -> None:
    for name in names:
        obj = bpy.data.objects.get(name)
        if obj is not None:
            mesh = obj.data
            bpy.data.objects.remove(obj, do_unlink=True)
            if mesh is not None and mesh.users == 0:
                bpy.data.meshes.remove(mesh)


def remove_area_objects(prefix, collection_name) -> None:
    for i in sorted(existing_indices(prefix)):
        remove_named(area_names(prefix, i)[::-1])
    coll = bpy.data.collections.get(collection_name)
    if coll is not None and len(coll.all_objects) == 0:
        bpy.data.collections.remove(coll)


def place_box(volume, center, heading_deg) -> None:
    """Put an area's box at its ENU center, turned by the heading."""
    target = timeline_sampler.blender_from_enu(tuple(center), heading_deg)[0]
    if np.abs(np.asarray(volume.location) - target).max() > _EPS:
        volume.location = target
    rot_z = np.radians(heading_deg)
    if abs(volume.rotation_euler.z - rot_z) > 1e-6 or volume.rotation_euler.x or volume.rotation_euler.y:
        volume.rotation_euler = (0.0, 0.0, rot_z)
    if tuple(volume.scale) != (1.0, 1.0, 1.0):
        volume.scale = (1.0, 1.0, 1.0)


def remove_objects() -> None:
    remove_named(_LEGACY_NAMES)
    remove_area_objects(_PREFIX, COLLECTION_NAME)


def sync_objects(scene) -> None:
    """Rebuild / re-place every area's objects from the settings (settings win)."""
    global _syncing
    settings = getattr(scene, "drone_show_settings", None)
    if settings is None or _syncing:
        return
    _syncing = True
    try:
        if not settings.show_holding_area_object:
            remove_objects()
            return
        remove_named(_LEGACY_NAMES)
        props, areas, counts = area_props(settings), areas_of(settings), slot_counts(settings)
        for i in existing_indices(_PREFIX) - set(range(len(areas))):
            remove_named(area_names(_PREFIX, i)[::-1])
        for i, (prop, area, count) in enumerate(zip(props, areas, counts)):
            volume, slots, clearance = ensure_area_objects(
                scene, COLLECTION_NAME, area_names(_PREFIX, i), config.COLOR_HOLDING_RGBA
            )
            key = _geometry_key(prop, count, settings.safety_radius_m)
            if volume.get(_GEOMETRY_KEY) != key:
                lo, hi = holding_area.holding_region_bounds(count, *area.args)
                build_area_meshes(
                    volume, slots, clearance, area.center, lo, hi, area.show_clearance_m,
                    holding_area.compute_holding_positions(count, *area.args), settings.safety_radius_m,
                )
                volume[_GEOMETRY_KEY] = key
            place_box(volume, area.center, settings.heading_offset_deg)
    finally:
        _syncing = False


def on_settings_changed(self, context):
    """`update=` callback for every property the holding-area objects use."""
    if context is not None and context.scene is not None:
        sync_objects(context.scene)


def _pull_from_objects(scene) -> None:
    """A box was moved in the viewport: write its position back to its area's
    settings (objects win). Moving on Z carries `max_height` along. A deleted
    box turns the toggle off and clears the rest, instead of recreating it."""
    global _syncing
    settings = getattr(scene, "drone_show_settings", None)
    if settings is None or not settings.show_holding_area_object or _syncing:
        return
    for i, ha in enumerate(area_props(settings)):
        volume = bpy.data.objects.get(area_names(_PREFIX, i)[0])
        if volume is None or volume.name not in scene.objects:
            _syncing = True
            try:
                settings.show_holding_area_object = False
                remove_objects()
            finally:
                _syncing = False
            return
        new_center = timeline_sampler.enu_transform(tuple(volume.location), settings.heading_offset_deg)[0]
        old_center = np.asarray(ha.center, dtype=float)
        if np.abs(new_center - old_center).max() <= _EPS:
            continue
        _syncing = True
        try:
            ha.max_height += float(new_center[2] - old_center[2])
            ha.center = tuple(float(v) for v in new_center)
            # Box height is unchanged, so the meshes stay valid; record that.
            volume[_GEOMETRY_KEY] = _geometry_key(ha, slot_counts(settings)[i], settings.safety_radius_m)
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
