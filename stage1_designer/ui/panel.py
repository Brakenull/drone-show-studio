"""Scene property group + N-Panel UI (spec section 2, `ui/panel.py`)."""

import bpy

from .. import config
from ..core.kinematic_validator import STATUS_ERROR, STATUS_OK, STATUS_WARNING
from . import viewport_drawer

# Cache of the last kinematic pre-validation pass (spec section 3.5), so the
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


_STATUS_ICON = {
    STATUS_OK: "CHECKMARK",
    STATUS_WARNING: "ERROR",
    STATUS_ERROR: "CANCEL",
}


class DSS_PG_HoldingArea(bpy.types.PropertyGroup):
    center: bpy.props.FloatVectorProperty(
        name="Center", size=3, default=config.DEFAULT_HOLDING_AREA["center"], subtype="XYZ"
    )
    size: bpy.props.FloatVectorProperty(
        name="Size (W, L)", size=2, default=config.DEFAULT_HOLDING_AREA["size"]
    )
    max_height: bpy.props.FloatProperty(
        name="Max Height (Z hold max)",
        default=config.DEFAULT_HOLDING_AREA["max_height"],
        min=0.1,
        unit="LENGTH",
    )
    grid_spacing_m: bpy.props.FloatProperty(
        name="Launch Grid Spacing (d_launch)",
        description=(
            "Holding-area grid pitch. Deliberately larger than the in-flight "
            "min_distance_m: rest-to-rest launch points can't bend to dodge a "
            "neighbor, so this needs its own margin (spec section 3.2)"
        ),
        default=config.DEFAULT_GRID_SPACING_M,
        min=config.MIN_GRID_SPACING_M,
        unit="LENGTH",
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
    )
    sampling_mode: bpy.props.EnumProperty(
        name="Sampling Mode", items=_sampling_mode_items,
    )
    dense_fps: bpy.props.IntProperty(
        name="Dense FPS", default=config.DEFAULT_DENSE_FPS, min=1,
    )
    safety_radius_m: bpy.props.FloatProperty(
        name="Safety Radius R_safe (m)", default=config.SAFETY_RADIUS_M, min=0.01, unit="LENGTH",
    )
    min_distance_m: bpy.props.FloatProperty(
        name="Min Distance d_min (m)", default=config.MIN_DISTANCE_M, min=0.02, unit="LENGTH",
    )
    heading_offset_deg: bpy.props.FloatProperty(
        name="Heading Offset (deg, True North)",
        default=config.DEFAULT_HEADING_OFFSET_DEG, min=0.0, max=359.999,
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
        box.label(text="Holding Area")
        box.prop(settings.holding_area, "center")
        box.prop(settings.holding_area, "size")
        box.prop(settings.holding_area, "max_height")
        box.prop(settings.holding_area, "grid_spacing_m")

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
        row = box.row()
        row.enabled = not has_kinematic_error()
        row.operator("dss.export_intermediate", icon="EXPORT")
        if has_kinematic_error():
            box.label(text="Fix the kinematic error(s) above (or Auto-Fix) to unlock Export", icon="ERROR")


CLASSES = (
    DSS_PG_HoldingArea,
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
