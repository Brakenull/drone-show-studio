"""Scene property group + N-Panel UI."""

import bpy

from .. import config
from ..core import ground as ground_core
from ..core import holding_area as holding_area_core
from ..core import show_legs
from ..core import waiting_area as waiting_area_core
from ..core.kinematic_validator import STATUS_ERROR, STATUS_OK, STATUS_WARNING
from . import ground_scene, holding_area_scene, viewport_drawer, waiting_area_scene


def _on_layout_changed(self, context):
    """`update=` callback for everything the scene helpers are built from:
    the holding-area objects and the ground grid (whose extent covers the
    holding area)."""
    holding_area_scene.on_settings_changed(self, context)
    ground_scene.on_settings_changed(self, context)
    waiting_area_scene.on_settings_changed(self, context)


def _on_waiting_changed(self, context):
    """`update=` callback for the waiting areas' settings."""
    waiting_area_scene.on_settings_changed(self, context)

# Cache of the last kinematic pre-validation pass, so the
# panel can render it on every redraw without re-sampling every keyframe each
# time (that's the same expensive work Export does). Refreshed by
# DSS_OT_CheckKinematics, DSS_OT_ExportIntermediate and DSS_OT_AutoFixTimeline
# in __init__.py; `None` means "never checked since the target object/scene
# was last touched".
_kinematic_cache = {"transitions": None}


def set_kinematic_cache(transitions) -> None:
    _kinematic_cache["transitions"] = transitions


def get_kinematic_cache():
    return _kinematic_cache["transitions"]


def has_kinematic_error() -> bool:
    transitions = _kinematic_cache["transitions"]
    return bool(transitions) and any(t.status == STATUS_ERROR for t in transitions)


# Each keyframe's formation points (ENU, parked drones excluded) from the last
# full sampling pass, for the holding-area clearance check. Results are recomputed from it whenever the holding-area settings
# or the safe distance change, without re-sampling.
_formation_cache = {"formations": None, "key": None, "results": None}


def set_formation_cache(formations) -> None:
    _formation_cache.update(formations=formations, key=None, results=None)


def get_clearance_results(settings):
    """Clearance results for the cached formations, or None if never sampled."""
    formations = _formation_cache["formations"]
    if formations is None:
        return None
    ha = settings.holding_area
    args = holding_area_scene.layout_args(settings)
    key = (args, ha.show_clearance_m)
    if _formation_cache["key"] != key:
        lo, hi = holding_area_core.holding_region_bounds(*args)
        _formation_cache["results"] = holding_area_core.check_show_clearance(
            formations, lo, hi, ha.show_clearance_m
        )
        _formation_cache["key"] = key
    return _formation_cache["results"]


def ground_warnings(settings) -> list:
    """Warning lines for anything below the ground:
    parked drones in the holding area (always known) and formation points
    (known after a sampling pass). Any line locks Export."""
    ground_z = settings.ground_z_m
    messages = []
    # The bottom layer of parked drones always sits at the holding area's center Z.
    depth = ground_core.lowest_point_below([(0.0, 0.0, settings.holding_area.center[2])], ground_z)
    if depth > 0.0:
        messages.append(f"Holding area: the bottom layer is {depth:.2f} m below the ground")
    for r in get_ground_results(settings) or []:
        if r.is_warning:
            messages.append(f"'{r.shape_name}': {r.below} point(s) below the ground (lowest {r.depth_m:.2f} m below)")
    return messages


def get_ground_results(settings):
    """Per-keyframe ground results for the cached formations, or None."""
    formations = _formation_cache["formations"]
    if formations is None:
        return None
    return ground_core.check_formations_ground(formations, settings.ground_z_m)


def min_layer_gap(settings) -> float:
    """Smallest allowed holding-area layer gap: a pad's
    vertical path to its hover point (Stage 2's landing approach height)
    keeps Stage 2's planning distance below the slot above, and stacked slots
    stay a grid step apart."""
    planning = settings.min_distance_m * config.STAGE2_PLANNING_DISTANCE_FACTOR
    hover_clear = holding_area_core.min_layer_spacing(config.STAGE2_HOVER_HEIGHT_M, planning)
    return max(hover_clear, settings.holding_area.grid_spacing_m)


def layer_gap_message(settings):
    """Caution (locks Export) when the fleet stacks layers closer than
    `min_layer_gap`, else None. A one-layer holding area has no gap to check."""
    if holding_area_scene.layout_for(settings).layers < 2:
        return None
    gap, minimum = settings.holding_area.layer_spacing_m, min_layer_gap(settings)
    if gap >= minimum - 1e-9:
        return None
    return f"Layer gap {gap:g} m is below {minimum:.2f} m: pads' hover points reach the layer above"


def waiting_messages(settings) -> list:
    """Caution lines for the waiting areas; any line locks
    Export. Formations too close are known after a sampling pass; the
    holding-area gap, overlaps and height always."""
    if not len(settings.waiting_areas):
        return []
    areas = waiting_area_scene.areas_of(settings)
    counts = waiting_area_scene.slot_counts(settings)
    lo, hi = holding_area_core.holding_region_bounds(*holding_area_scene.layout_args(settings))
    formations = _formation_cache["formations"] or []
    check = waiting_area_core.check_waiting_areas(areas, counts, formations, lo, hi, settings.ground_z_m)
    return check.messages(settings.holding_area.show_clearance_m)


def waiting_detour_notes(settings) -> list:
    """Notes (they do not lock Export) for short keyframes whose spare drones
    would fly farther to every waiting area and back than to the holding area
    and back. Known after a sampling pass."""
    formations = _formation_cache["formations"]
    if not len(settings.waiting_areas) or not formations:
        return []
    lo, hi = holding_area_core.holding_region_bounds(*holding_area_scene.layout_args(settings))
    detours = waiting_area_core.check_detours(
        waiting_area_scene.areas_of(settings), waiting_area_scene.slot_counts(settings), formations,
        settings.fleet_size, lo, hi,
    )
    return [
        f"'{d.shape_name}': its {d.spare} spare drones fly ~{d.waiting_m:.0f} m to the nearest waiting area "
        f"and back, ~{d.home_m:.0f} m via the holding area: move an area closer"
        for d in detours
    ]


def clearance_cautions(settings) -> list:
    """Formations too close to the holding area (they lock Export)."""
    return [r for r in get_clearance_results(settings) or [] if r.is_caution]


def clearance_message(result, clearance_m: float) -> str:
    if result.inside:
        return (
            f"'{result.shape_name}': {result.inside} point(s) inside the holding area"
            + (f", {result.too_close} more within {clearance_m:g} m" if result.too_close else "")
        )
    return (
        f"'{result.shape_name}': {result.too_close} point(s) within {clearance_m:g} m "
        f"of the holding area (closest {result.closest_m:.2f} m)"
    )


def configured_v_max(settings) -> float:
    if settings.kinematic_constraints.enabled:
        return settings.kinematic_constraints.v_max_mps
    return config.DEFAULT_KINEMATIC_CONSTRAINTS["v_max_mps"]


# First / last keyframe positions (full fleet, parked drones included) and the
# keyframe span from the last full sampling pass, for the takeoff / return
# estimates. Like the clearance results, the estimates are
# recomputed from these when the holding area or v_max changes.
_leg_cache = {"first": None, "last": None, "show_span": 0.0, "key": None, "estimates": None}


def set_leg_cache(first_positions, last_positions, show_span_sec: float) -> None:
    _leg_cache.update(
        first=first_positions, last=last_positions, show_span=show_span_sec, key=None, estimates=None
    )


def get_leg_estimates(settings):
    """(takeoff, return) `LegEstimate`s, or None if never sampled or the fleet
    size changed since."""
    first, last = _leg_cache["first"], _leg_cache["last"]
    if first is None or len(first) != settings.fleet_size:
        return None
    args = holding_area_scene.layout_args(settings)
    v_max = configured_v_max(settings)
    key = (args, v_max)
    if _leg_cache["key"] != key:
        slots = holding_area_core.compute_holding_positions(*args)
        waves = show_legs.launch_wave_span(*args, wave_delay_s=config.STAGE2_STAGGER_WAVE_DELAY_S)
        _leg_cache["estimates"] = (
            show_legs.estimate_leg(slots, first, v_max, wave_span_sec=waves),
            show_legs.estimate_leg(last, slots, v_max),
        )
        _leg_cache["key"] = key
    return _leg_cache["estimates"]


def _leg_targets(settings):
    legs = settings.legs
    return (
        ("Takeoff", legs.takeoff_mode, legs.takeoff_duration_sec),
        ("Return", legs.return_mode, legs.return_duration_sec),
    )


def short_leg_messages(settings) -> list:
    """Warnings for leg targets below their estimated minimum."""
    estimates = get_leg_estimates(settings)
    if estimates is None:
        return []
    messages = []
    for (label, mode, seconds), est in zip(_leg_targets(settings), estimates):
        if mode == config.LEG_MODE_TARGET and seconds < est.min_duration_sec:
            messages.append(
                f"{label} target {seconds:g} s is below the estimated minimum "
                f"{est.min_duration_sec:.1f} s; Stage 2 will stretch it"
            )
    return messages


_STATUS_ICON = {
    STATUS_OK: "CHECKMARK",
    STATUS_WARNING: "ERROR",
    STATUS_ERROR: "CANCEL",
}


class DSS_PG_WaitingArea(bpy.types.PropertyGroup):
    """One waiting area: one flat layer of slots at center Z
    where spare drones wait, LEDs off, instead of flying home mid-show."""

    name: bpy.props.StringProperty(name="Name", default="Waiting Area")
    center: bpy.props.FloatVectorProperty(
        name="Center", size=3, default=config.DEFAULT_WAITING_AREA["center"], subtype="XYZ",
        description="ENU center of the area; its slots are at this height", update=_on_waiting_changed,
    )
    size: bpy.props.FloatVectorProperty(
        name="Size (W, L)", size=2, default=config.DEFAULT_WAITING_AREA["size"], min=0.0,
        description="Footprint; the last area widens along X if the spare drones don't fit",
        update=_on_waiting_changed,
    )
    grid_spacing_m: bpy.props.FloatProperty(
        name="Slot Spacing", default=config.DEFAULT_WAITING_AREA["grid_spacing_m"],
        min=config.MIN_GRID_SPACING_M, unit="LENGTH", update=_on_waiting_changed,
    )
    show_clearance_m: bpy.props.FloatProperty(
        name="Safe Distance to Show", default=config.DEFAULT_WAITING_AREA["show_clearance_m"], min=0.0,
        unit="LENGTH", description="Minimum distance from any formation point to this area",
        update=_on_waiting_changed,
    )


class DSS_UL_WaitingAreas(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        row = layout.row(align=True)
        row.label(text=f"{index + 1}.", icon="EMPTY_AXIS")
        row.prop(item, "name", text="", emboss=False)
        c = item.center
        row.label(text=f"({c[0]:.0f}, {c[1]:.0f}, {c[2]:.0f})")


class DSS_OT_WaitingAreaAdd(bpy.types.Operator):
    """Add a waiting area (spare drones wait there in the air instead of going home)"""

    bl_idname = "dss.waiting_area_add"
    bl_label = "Add Waiting Area"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        settings = context.scene.drone_show_settings
        area = settings.waiting_areas.add()
        area.name = f"Waiting Area {len(settings.waiting_areas)}"
        settings.waiting_area_index = len(settings.waiting_areas) - 1
        waiting_area_scene.sync_objects(context.scene)
        return {"FINISHED"}


class DSS_OT_WaitingAreaRemove(bpy.types.Operator):
    """Remove the selected waiting area"""

    bl_idname = "dss.waiting_area_remove"
    bl_label = "Remove Waiting Area"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return len(context.scene.drone_show_settings.waiting_areas) > 0

    def execute(self, context):
        settings = context.scene.drone_show_settings
        settings.waiting_areas.remove(settings.waiting_area_index)
        settings.waiting_area_index = max(0, min(settings.waiting_area_index, len(settings.waiting_areas) - 1))
        waiting_area_scene.remove_objects()  # names follow the list order: rebuild them all
        waiting_area_scene.sync_objects(context.scene)
        return {"FINISHED"}


class DSS_PG_HoldingArea(bpy.types.PropertyGroup):
    center: bpy.props.FloatVectorProperty(
        name="Center", size=3, default=config.DEFAULT_HOLDING_AREA["center"], subtype="XYZ",
        update=_on_layout_changed,
    )
    size: bpy.props.FloatVectorProperty(
        name="Size (W, L)", size=2, default=config.DEFAULT_HOLDING_AREA["size"],
        update=_on_layout_changed,
    )
    max_height: bpy.props.FloatProperty(
        name="Max Height (Z hold max)",
        default=config.DEFAULT_HOLDING_AREA["max_height"],
        min=0.1,
        unit="LENGTH",
        update=_on_layout_changed,
    )
    grid_spacing_m: bpy.props.FloatProperty(
        name="Launch Grid Spacing (d_launch)",
        description=(
            "Holding-area grid pitch. Deliberately larger than the in-flight "
            "min_distance_m: rest-to-rest launch points can't bend to dodge a "
            "neighbor, so this needs its own margin"
        ),
        default=config.DEFAULT_GRID_SPACING_M,
        min=config.MIN_GRID_SPACING_M,
        unit="LENGTH",
        update=_on_layout_changed,
    )
    layer_spacing_m: bpy.props.FloatProperty(
        name="Layer Gap",
        description=(
            "Height between stacked layers of parked drones. At least the hover height "
            "(2 m) plus Stage 2's planning distance, so a pad's vertical path stays clear "
            "of the layer above; more gap means less downwash on the drones below"
        ),
        default=config.DEFAULT_LAYER_SPACING_M,
        min=config.MIN_GRID_SPACING_M,
        unit="LENGTH",
        update=_on_layout_changed,
    )
    staggered_layers: bpy.props.BoolProperty(
        name="Shift Alternate Layers",
        description=(
            "Shift every other layer half a slot in X and Y (one column and one row fewer), "
            "so no parked drone sits straight above another"
        ),
        default=config.DEFAULT_STAGGERED_LAYERS,
        update=_on_layout_changed,
    )
    show_clearance_m: bpy.props.FloatProperty(
        name="Safe Distance to Show",
        description=(
            "Minimum distance from any formation point to the holding area (the parked grid, "
            "padded by half a grid step). Closer points raise a caution; 0 flags only points "
            "inside the holding area"
        ),
        default=config.DEFAULT_SHOW_CLEARANCE_M,
        min=0.0,
        unit="LENGTH",
        update=_on_layout_changed,
    )


def _leg_mode_items(self, context):
    return [
        (config.LEG_MODE_AUTO, "Auto", "Fly this leg as fast as Stage 2 can do it safely"),
        (config.LEG_MODE_TARGET, "Target", "Fly this leg in a set time (Stage 2 stretches it if that's too short)"),
    ]


class DSS_PG_ShowLegs(bpy.types.PropertyGroup):
    """Takeoff (holding area -> first keyframe) and return (last keyframe ->
    holding area) legs."""

    takeoff_mode: bpy.props.EnumProperty(name="Takeoff", items=_leg_mode_items)
    takeoff_duration_sec: bpy.props.FloatProperty(
        name="Takeoff Time (s)",
        description="Target time for the whole fleet to fly from the holding area to the first formation",
        default=config.DEFAULT_LEG_DURATION_SEC, min=0.1, soft_max=600.0,
    )
    return_mode: bpy.props.EnumProperty(name="Return", items=_leg_mode_items)
    return_duration_sec: bpy.props.FloatProperty(
        name="Return Time (s)",
        description="Target time for the whole fleet to fly from the last formation back to the holding area",
        default=config.DEFAULT_LEG_DURATION_SEC, min=0.1, soft_max=600.0,
    )


class DSS_PG_KinematicConstraints(bpy.types.PropertyGroup):
    enabled: bpy.props.BoolProperty(name="Override Kinematic Constraints", default=False)
    v_max_mps: bpy.props.FloatProperty(
        name="v_max (m/s)", default=config.DEFAULT_KINEMATIC_CONSTRAINTS["v_max_mps"], min=0.01
    )
    a_max_mps2: bpy.props.FloatProperty(
        name="a_max (m/s^2)", default=config.DEFAULT_KINEMATIC_CONSTRAINTS["a_max_mps2"], min=0.01
    )
    j_max_mps3: bpy.props.FloatProperty(
        name="j_max (m/s^3)", default=config.DEFAULT_KINEMATIC_CONSTRAINTS["j_max_mps3"], min=0.01
    )


class DSS_PG_OriginGPS(bpy.types.PropertyGroup):
    latitude: bpy.props.FloatProperty(name="Latitude", default=10.762622, min=-90.0, max=90.0)
    longitude: bpy.props.FloatProperty(name="Longitude", default=106.660172, min=-180.0, max=180.0)
    altitude_amsl: bpy.props.FloatProperty(name="Altitude AMSL (m)", default=15.0)


def _sampling_mode_items(self, context):
    return [
        (config.SAMPLING_MODE_KEYFRAME_ONLY, "Keyframe Only", "Sample only at animator-authored keyframes"),
        (config.SAMPLING_MODE_DENSE_SAMPLED, "Dense Sampled", "Sample densely at a fixed FPS"),
    ]


def _sample_method_items(self, context):
    return [
        ("SURFACE", "Surface", "Poisson-disk sample the mesh surface"),
        ("VOLUME", "Volume", "Jittered-grid volumetric sample of the mesh interior"),
    ]


class DSS_PG_ProjectSettings(bpy.types.PropertyGroup):
    target_object: bpy.props.PointerProperty(
        name="Target Object", type=bpy.types.Object,
        description="Mesh object to sample into the drone point cloud",
    )
    sample_method: bpy.props.EnumProperty(
        name="Sample Method", items=_sample_method_items,
    )
    fleet_size: bpy.props.IntProperty(
        name="Fleet Size", default=config.DEFAULT_FLEET_SIZE, min=1,
        update=_on_layout_changed,
    )
    sampling_mode: bpy.props.EnumProperty(
        name="Sampling Mode", items=_sampling_mode_items,
    )
    dense_fps: bpy.props.IntProperty(
        name="Dense FPS", default=config.DEFAULT_DENSE_FPS, min=1,
    )
    safety_radius_m: bpy.props.FloatProperty(
        name="Safety Radius R_safe (m)", default=config.SAFETY_RADIUS_M, min=0.01, unit="LENGTH",
        update=_on_layout_changed,
    )
    min_distance_m: bpy.props.FloatProperty(
        name="Min Distance d_min (m)", default=config.MIN_DISTANCE_M, min=0.02, unit="LENGTH",
    )
    heading_offset_deg: bpy.props.FloatProperty(
        name="Heading Offset (deg, True North)",
        default=config.DEFAULT_HEADING_OFFSET_DEG, min=0.0, max=359.999,
        update=_on_layout_changed,
    )
    def _on_toggle_overlay(self, context):
        if self.show_viewport_overlay:
            viewport_drawer.enable()
        else:
            viewport_drawer.disable()
        for area in context.screen.areas:
            if area.type == "VIEW_3D":
                area.tag_redraw()

    show_viewport_overlay: bpy.props.BoolProperty(
        name="Show Safety Overlay", default=False, update=_on_toggle_overlay,
    )
    live_update_on_frame_change: bpy.props.BoolProperty(
        name="Live Update on Frame Change",
        description="Re-sample the target object and refresh the overlay whenever the current frame changes (playback or scrubbing)",
        default=False,
    )
    show_holding_area_preview: bpy.props.BoolProperty(
        name="Show Holding Area",
        description="Also preview the leftover drones parked in the Holding Area (blue spheres) alongside the sampled shape",
        default=True,
    )
    ground_z_m: bpy.props.FloatProperty(
        name="Ground Level (Z)",
        description=(
            "Height of the ground in ENU (also Blender Z; the heading only rotates about Z). "
            "Formation points and parked drones below it raise a warning"
        ),
        default=config.DEFAULT_GROUND_Z_M,
        unit="LENGTH",
        update=_on_layout_changed,
    )
    show_ground_object: bpy.props.BoolProperty(
        name="Show Ground in Scene",
        description="Draw the ground level as a wire grid (10 m cells) under the holding area and the show",
        default=False,
        update=ground_scene.on_settings_changed,
    )
    show_holding_area_object: bpy.props.BoolProperty(
        name="Create in Scene",
        description=(
            "Create the holding area as scene objects: a wire box for the volume and a sphere "
            "(R_safe) per launch slot for the whole fleet. Grab the box (G) to move the holding "
            "area; size and height are edited here"
        ),
        default=False,
        update=_on_layout_changed,
    )
    holding_area_object: bpy.props.PointerProperty(
        name="Holding Area Object", type=bpy.types.Object,
        description="The wire box created by Create in Scene (managed by the add-on)",
    )
    show_compass_gizmo: bpy.props.BoolProperty(
        name="Show Compass Gizmo", default=True,
    )
    export_path: bpy.props.StringProperty(
        name="Export Path", subtype="FILE_PATH", default="//intermediate_export.json",
    )
    export_format: bpy.props.EnumProperty(
        name="Format",
        items=[("JSON", "JSON", "Human-readable JSON"), ("MSGPACK", "MessagePack", "Compact binary")],
    )
    origin_gps: bpy.props.PointerProperty(type=DSS_PG_OriginGPS)
    holding_area: bpy.props.PointerProperty(type=DSS_PG_HoldingArea)
    waiting_areas: bpy.props.CollectionProperty(type=DSS_PG_WaitingArea)
    waiting_area_index: bpy.props.IntProperty(name="Active Waiting Area", default=0, min=0)
    show_waiting_area_objects: bpy.props.BoolProperty(
        name="Create in Scene",
        description=(
            "Build each waiting area as scene objects: a wire box (grab it to move the area), "
            "one sphere per slot and the safe-distance box"
        ),
        default=False,
        update=_on_waiting_changed,
    )
    legs: bpy.props.PointerProperty(type=DSS_PG_ShowLegs)
    kinematic_constraints: bpy.props.PointerProperty(type=DSS_PG_KinematicConstraints)


class DSS_PT_MainPanel(bpy.types.Panel):
    bl_label = "Drone Show Studio"
    bl_idname = "DSS_PT_main_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Drone Show"

    def draw(self, context):
        layout = self.layout
        settings = context.scene.drone_show_settings

        box = layout.box()
        box.label(text="Source & Fleet")
        box.prop(settings, "target_object")
        box.prop(settings, "sample_method")
        box.prop(settings, "fleet_size")

        box = layout.box()
        box.label(text="Safety Distances")
        box.prop(settings, "safety_radius_m")
        box.prop(settings, "min_distance_m")

        box = layout.box()
        box.label(text="Timeline Sampling")
        box.prop(settings, "sampling_mode")
        if settings.sampling_mode == config.SAMPLING_MODE_DENSE_SAMPLED:
            box.prop(settings, "dense_fps")

        box = layout.box()
        box.label(text="ENU / True North")
        box.prop(settings, "heading_offset_deg")
        box.prop(settings, "show_compass_gizmo")
        col = box.column(align=True)
        col.prop(settings.origin_gps, "latitude")
        col.prop(settings.origin_gps, "longitude")
        col.prop(settings.origin_gps, "altitude_amsl")

        box = layout.box()
        box.label(text="Ground")
        box.prop(settings, "ground_z_m")
        box.prop(settings, "show_ground_object", toggle=True, icon="MESH_GRID")
        warnings = ground_warnings(settings)
        if warnings:
            col = box.column(align=True)
            col.alert = True
            col.label(text="WARNING: drones below the ground", icon="ERROR")
            for msg in warnings:
                col.label(text=msg)
            col.label(text="Export is locked")
        elif get_ground_results(settings) is None:
            box.label(text="Formations: run Check Kinematics to check them")
        else:
            lowest = min((r.lowest_z for r in get_ground_results(settings)), default=None)
            box.label(
                text="All drones above the ground"
                + (f" (lowest {lowest - settings.ground_z_m:.1f} m up)" if lowest is not None else ""),
                icon="CHECKMARK",
            )

        box = layout.box()
        box.label(text="Holding Area")
        box.prop(settings, "show_holding_area_object", toggle=True, icon="MESH_CUBE")
        layout_info = holding_area_scene.layout_for(settings)
        per_layer = f"{layout_info.cols} x {layout_info.rows} grid"
        if layout_info.staggered and layout_info.layers > 1:
            per_layer = f"{layout_info.layer(0).capacity} / {layout_info.layer(1).capacity} per layer (shifted)"
        box.label(
            text=f"{settings.fleet_size} slots: {per_layer}, {layout_info.layers} layer(s) "
            f"{settings.holding_area.layer_spacing_m:g} m apart",
        )
        if layout_info.widened:
            box.label(text=f"Widened to {layout_info.width:.1f} m to fit the fleet", icon="ERROR")
        box.prop(settings.holding_area, "center")
        box.prop(settings.holding_area, "size")
        box.prop(settings.holding_area, "max_height")
        box.prop(settings.holding_area, "grid_spacing_m")
        box.prop(settings.holding_area, "layer_spacing_m")
        box.prop(settings.holding_area, "staggered_layers")
        gap_message = layer_gap_message(settings)
        if gap_message:
            col = box.column(align=True)
            col.alert = True
            col.label(text=f"CAUTION: {gap_message}", icon="ERROR")
            col.label(text="Export is locked")
        box.prop(settings.holding_area, "show_clearance_m")
        results = get_clearance_results(settings)
        if results is None:
            box.label(text="Clearance: run Check Kinematics to check the show")
        else:
            cautions = [r for r in results if r.is_caution]
            if not cautions:
                closest = min((r.closest_m for r in results), default=float("inf"))
                box.label(
                    text=f"Show clears the holding area (closest {closest:.1f} m)"
                    if results else "Show clears the holding area",
                    icon="CHECKMARK",
                )
            else:
                col = box.column(align=True)
                col.alert = True
                col.label(text=f"CAUTION: {len(cautions)} formation(s) too close to the holding area", icon="ERROR")
                for r in cautions:
                    col.label(text=clearance_message(r, settings.holding_area.show_clearance_m))
                col.label(text="Export is locked")

        _draw_waiting_areas(layout, settings)

        _draw_legs(layout, settings)

        box = layout.box()
        box.label(text="Kinematic Constraints")
        box.prop(settings.kinematic_constraints, "enabled")
        if settings.kinematic_constraints.enabled:
            col = box.column(align=True)
            col.prop(settings.kinematic_constraints, "v_max_mps")
            col.prop(settings.kinematic_constraints, "a_max_mps2")
            col.prop(settings.kinematic_constraints, "j_max_mps3")

        box = layout.box()
        box.label(text="Kinematic Pre-Validation")
        box.operator("dss.check_kinematics", icon="FILE_REFRESH")
        transitions = get_kinematic_cache()
        if transitions is None:
            box.label(text="Not checked yet")
        elif not transitions:
            box.label(text="Nothing to check (need >= 2 keyframes)", icon="CHECKMARK")
        else:
            col = box.column(align=True)
            for t in transitions:
                col.label(
                    text=f"[{t.from_index}->{t.to_index}] {t.v_req:.1f} m/s over {t.delta_t:.2f}s (D={t.d_max:.1f}m)",
                    icon=_STATUS_ICON[t.status],
                )
            if has_kinematic_error():
                box.label(text="ERROR: transition(s) exceed v_max — Export is locked", icon="ERROR")
            box.operator("dss.auto_fix_timeline", icon="MOD_TIME")

        box = layout.box()
        box.label(text="Viewport")
        box.operator("dss.preview_sample", icon="FILE_REFRESH")
        box.prop(settings, "show_viewport_overlay", toggle=True)
        row = box.row()
        row.enabled = settings.show_viewport_overlay
        row.prop(settings, "live_update_on_frame_change")
        row = box.row()
        row.enabled = settings.show_viewport_overlay
        row.prop(settings, "show_holding_area_preview")

        box = layout.box()
        box.label(text="Export")
        box.prop(settings, "export_path")
        box.prop(settings, "export_format")
        clearance_locked = bool(clearance_cautions(settings))
        ground_locked = bool(ground_warnings(settings))
        gap_locked = layer_gap_message(settings) is not None
        waiting_locked = bool(waiting_messages(settings))
        row = box.row()
        row.enabled = (
            not has_kinematic_error() and not clearance_locked and not ground_locked and not gap_locked
            and not waiting_locked
        )
        row.operator("dss.export_intermediate", icon="EXPORT")
        if has_kinematic_error():
            box.label(text="Fix the kinematic error(s) above (or Auto-Fix) to unlock Export", icon="ERROR")
        if clearance_locked:
            box.label(
                text="Move the formations or the holding area apart (or lower Safe Distance) to unlock Export",
                icon="ERROR",
            )
        if ground_locked:
            box.label(text="Raise the drones above the ground (or lower Ground Level) to unlock Export", icon="ERROR")
        if gap_locked:
            box.label(
                text=f"Raise the holding area's Layer Gap to {min_layer_gap(settings):.2f} m to unlock Export",
                icon="ERROR",
            )
        if waiting_locked:
            box.label(text="Fix the waiting area caution(s) above to unlock Export", icon="ERROR")


def _draw_waiting_areas(layout, settings):
    box = layout.box()
    box.label(text="Waiting Areas")
    row = box.row()
    row.template_list("DSS_UL_WaitingAreas", "", settings, "waiting_areas", settings, "waiting_area_index", rows=2)
    col = row.column(align=True)
    col.operator("dss.waiting_area_add", icon="ADD", text="")
    col.operator("dss.waiting_area_remove", icon="REMOVE", text="")
    if not len(settings.waiting_areas):
        box.label(text="None: spare drones fly home to the holding area mid-show")
        return
    box.prop(settings, "show_waiting_area_objects", toggle=True, icon="MESH_GRID")
    index = min(settings.waiting_area_index, len(settings.waiting_areas) - 1)
    area = settings.waiting_areas[index]
    col = box.column(align=True)
    col.prop(area, "center")
    col.prop(area, "size")
    col.prop(area, "grid_spacing_m")
    col.prop(area, "show_clearance_m")
    counts = waiting_area_scene.slot_counts(settings)
    spare = waiting_area_scene.spare_needed()
    box.label(
        text=f"{sum(counts)} slots ({' + '.join(str(c) for c in counts)}); "
        + (f"{spare} spare drones at most" if _formation_cache["formations"] is not None
           else "run Check Kinematics for the spare count"),
    )
    areas = waiting_area_scene.areas_of(settings)
    for i, (a, n) in enumerate(zip(areas, counts)):
        if n > a.capacity:
            w, length = waiting_area_core.size_needed(n, a)
            box.label(
                text=f"Area {i + 1} needs {n} places ({a.capacity} declared): grows to {w:g} x {length:g} m",
                icon="ERROR",
            )
    for note in waiting_detour_notes(settings):
        box.label(text=note, icon="INFO")
    messages = waiting_messages(settings)
    if messages:
        col = box.column(align=True)
        col.alert = True
        col.label(text=f"CAUTION: {len(messages)} waiting area problem(s)", icon="ERROR")
        for m in messages:
            col.label(text=m)
        col.label(text="Export is locked")


def _draw_legs(layout, settings):
    box = layout.box()
    box.label(text="Takeoff & Return")
    legs = settings.legs
    estimates = get_leg_estimates(settings)

    planned = []
    for (label, mode, seconds), prop, est_index in (
        (_leg_targets(settings)[0], "takeoff", 0),
        (_leg_targets(settings)[1], "return", 1),
    ):
        row = box.row(align=True)
        row.prop(legs, f"{prop}_mode", expand=True)
        if mode == config.LEG_MODE_TARGET:
            box.prop(legs, f"{prop}_duration_sec")
        if estimates is None:
            continue
        est = estimates[est_index]
        col = box.column(align=True)
        col.label(text=f"Estimated minimum ~{est.min_duration_sec:.1f} s (longest flight {est.d_max_m:.1f} m)")
        if est.wave_span_sec > 0.0:
            col.label(text=f"+ up to {est.wave_span_sec:.1f} s if Stage 2 launches row by row")
        if mode == config.LEG_MODE_TARGET and seconds < est.min_duration_sec:
            warn = box.column(align=True)
            warn.alert = True
            warn.label(text=f"{label} target is below the estimated minimum:", icon="ERROR")
            warn.label(text=f"Stage 2 will stretch it to ~{est.min_duration_sec:.1f} s")
        planned.append(max(seconds, est.min_duration_sec) if mode == config.LEG_MODE_TARGET else est.min_duration_sec)

    if estimates is None:
        box.label(text="Estimates: run Check Kinematics")
    else:
        show_span = _leg_cache["show_span"]
        box.label(
            text=f"Takeoff ~{planned[0]:.0f} s + show {show_span:.0f} s + return ~{planned[1]:.0f} s "
            f"= ~{sum(planned) + show_span:.0f} s",
            icon="TIME",
        )



CLASSES = (
    DSS_PG_WaitingArea,
    DSS_UL_WaitingAreas,
    DSS_OT_WaitingAreaAdd,
    DSS_OT_WaitingAreaRemove,
    DSS_PG_HoldingArea,
    DSS_PG_ShowLegs,
    DSS_PG_KinematicConstraints,
    DSS_PG_OriginGPS,
    DSS_PG_ProjectSettings,
    DSS_PT_MainPanel,
)


def register():
    for cls in CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Scene.drone_show_settings = bpy.props.PointerProperty(type=DSS_PG_ProjectSettings)


def unregister():
    del bpy.types.Scene.drone_show_settings
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
