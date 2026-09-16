"""Drone Show Studio - Phase 1: Blender Add-on (Design & Voxelization).

See `.claude/docs/1-phase_1.md` for the full technical specification.

This top-level package guards its `bpy` import so the pure algorithm
submodules (`core.holding_area`, `core.sampler`'s geometry functions,
`core.timeline_sampler.enu_transform`, `exporters.intermediate_exporter`)
remain importable - and unit-testable with pytest - outside Blender.
"""

bl_info = {
    "name": "Drone Show Studio - Designer",
    "author": "Drone Show Studio",
    "version": (1, 4, 0),
    "blender": (4, 0, 0),
    "location": "View3D > Sidebar > Drone Show",
    "description": (
        "Convert 3D models/animation into drone point clouds (Phase 1: "
        "Design & Voxelization) for the Drone Show Studio pipeline."
    ),
    "category": "Object",
}

try:
    import bpy

    _HAS_BPY = True
except ImportError:  # pragma: no cover - outside Blender
    bpy = None
    _HAS_BPY = False


if _HAS_BPY:
    import numpy as np

    from . import config
    from .core import color_extractor, holding_area, sampler, timeline_sampler
    from .exporters import intermediate_exporter
    from .ui import compass_gizmo, panel, viewport_drawer

    _redraw_timer_running = False

    def _redraw_all_view3d():
        for window in bpy.context.window_manager.windows:
            for area in window.screen.areas:
                if area.type == "VIEW_3D":
                    area.tag_redraw()

    def _blink_timer():
        any_overlay_active = any(
            getattr(scene, "drone_show_settings", None) is not None
            and scene.drone_show_settings.show_viewport_overlay
            for scene in bpy.data.scenes
        )
        if any_overlay_active:
            _redraw_all_view3d()
        return 0.1  # reschedule

    def _sample_shape_points(settings, obj):
        """Sample `obj` with the configured method, capped at fleet_size, and
        return (positions_world, colors_rgb8) with matching length."""
        if settings.sample_method == "SURFACE":
            points = sampler.sample_object_surface(
                obj, settings.min_distance_m, max_points=settings.fleet_size
            )
        else:
            points = sampler.sample_object_volume(obj, settings.min_distance_m)
            if len(points) > settings.fleet_size:
                points = points[: settings.fleet_size]

        shape_color = color_extractor.get_material_base_color(obj)
        colors = [shape_color] * len(points)
        return points, colors

    def _build_keyframe_for_time(settings, obj, scene, time_sec, shape_name):
        timeline_sampler.evaluate_object_at_time(obj, scene, time_sec)
        raw_points, colors = _sample_shape_points(settings, obj)

        n_sampled = len(raw_points)
        fleet_size = settings.fleet_size

        if n_sampled > fleet_size:
            raw_points = raw_points[:fleet_size]
            colors = colors[:fleet_size]
            n_sampled = fleet_size

        enu_points = (
            timeline_sampler.enu_transform(raw_points, settings.heading_offset_deg)
            if n_sampled > 0
            else np.zeros((0, 3))
        )

        n_park = fleet_size - n_sampled
        if n_park > 0:
            holding_positions = holding_area.compute_holding_positions(
                n_park,
                tuple(settings.holding_area.center),
                tuple(settings.holding_area.size),
                max_height=settings.holding_area.max_height,
                min_dist=settings.min_distance_m,
            )
            all_positions = np.vstack([enu_points, holding_positions]) if n_sampled else holding_positions
            all_colors = list(colors) + [color_extractor.BLACK_RGB8] * n_park
        else:
            all_positions = enu_points
            all_colors = list(colors)

        return intermediate_exporter.build_keyframe_entry(
            time_sec, shape_name, all_positions, all_colors
        )

    def _shape_name_for_frame(scene, frame: int) -> str:
        for marker in scene.timeline_markers:
            if marker.frame == frame:
                return marker.name
        return f"Shape_{frame}"

    def _refresh_overlay_for_scene(scene) -> None:
        """Re-sample the target object at the scene's current frame and push
        the result into the viewport overlay cache. Used by both the manual
        Preview Sample operator and the automatic frame-change handler."""
        settings = getattr(scene, "drone_show_settings", None)
        if settings is None or settings.target_object is None:
            return
        points, _colors = _sample_shape_points(settings, settings.target_object)
        viewport_drawer.set_preview_points(
            points, settings.min_distance_m, settings.safety_radius_m
        )

        n_park = settings.fleet_size - len(points)
        if settings.show_holding_area_preview and n_park > 0:
            holding_points = holding_area.compute_holding_positions(
                n_park,
                tuple(settings.holding_area.center),
                tuple(settings.holding_area.size),
                max_height=settings.holding_area.max_height,
                min_dist=settings.min_distance_m,
            )
            viewport_drawer.set_holding_preview_points(holding_points)
        else:
            viewport_drawer.set_holding_preview_points(np.zeros((0, 3)))

        return points

    @bpy.app.handlers.persistent
    def _on_frame_change_post(scene, _depsgraph):
        settings = getattr(scene, "drone_show_settings", None)
        if settings is None:
            return
        if not (settings.show_viewport_overlay and settings.live_update_on_frame_change):
            return
        try:
            _refresh_overlay_for_scene(scene)
        except ImportError:
            pass  # scipy missing - Preview Sample already surfaces this error explicitly
        _redraw_all_view3d()

    class DSS_OT_PreviewSample(bpy.types.Operator):
        bl_idname = "dss.preview_sample"
        bl_label = "Preview Sample"
        bl_description = "Sample the target object at the current frame and update the safety overlay"

        def execute(self, context):
            settings = context.scene.drone_show_settings
            if settings.target_object is None:
                self.report({"ERROR"}, "No target object set")
                return {"CANCELLED"}

            try:
                points = _refresh_overlay_for_scene(context.scene)
            except ImportError as exc:
                self.report({"ERROR"}, str(exc))
                return {"CANCELLED"}

            _redraw_all_view3d()
            self.report({"INFO"}, f"Sampled {len(points)} points")
            return {"FINISHED"}

    class DSS_OT_ExportIntermediate(bpy.types.Operator):
        bl_idname = "dss.export_intermediate"
        bl_label = "Export Intermediate Data"
        bl_description = "Sample all keyframes and export the Phase 1 -> Phase 2 interchange file"

        def execute(self, context):
            scene = context.scene
            settings = scene.drone_show_settings
            obj = settings.target_object
            if obj is None:
                self.report({"ERROR"}, "No target object set")
                return {"CANCELLED"}

            original_frame = scene.frame_current
            try:
                times = timeline_sampler.get_sample_times_seconds(
                    obj,
                    scene,
                    settings.sampling_mode,
                    fps=settings.dense_fps if settings.sampling_mode == config.SAMPLING_MODE_DENSE_SAMPLED else None,
                )
            except ValueError as exc:
                self.report({"ERROR"}, str(exc))
                return {"CANCELLED"}

            if not times:
                self.report({"ERROR"}, "No keyframes found on target object")
                return {"CANCELLED"}

            try:
                keyframe_entries = []
                for time_sec in times:
                    frame = int(round(time_sec * (scene.render.fps / scene.render.fps_base)))
                    shape_name = _shape_name_for_frame(scene, frame)
                    keyframe_entries.append(
                        _build_keyframe_for_time(settings, obj, scene, time_sec, shape_name)
                    )
            except ImportError as exc:
                self.report({"ERROR"}, str(exc))
                return {"CANCELLED"}
            finally:
                scene.frame_set(original_frame)

            kinematic_constraints = None
            if settings.kinematic_constraints.enabled:
                kinematic_constraints = {
                    "v_max_mps": settings.kinematic_constraints.v_max_mps,
                    "a_max_mps2": settings.kinematic_constraints.a_max_mps2,
                    "j_max_mps3": settings.kinematic_constraints.j_max_mps3,
                }

            metadata = intermediate_exporter.build_project_metadata(
                fleet_size=settings.fleet_size,
                sampling_mode=settings.sampling_mode,
                total_duration_sec=times[-1] - times[0] if len(times) > 1 else 0.0,
                heading_offset_deg=settings.heading_offset_deg,
                origin_gps=(
                    settings.origin_gps.latitude,
                    settings.origin_gps.longitude,
                    settings.origin_gps.altitude_amsl,
                ),
                holding_area={
                    "center": tuple(settings.holding_area.center),
                    "size": tuple(settings.holding_area.size),
                    "max_height": settings.holding_area.max_height,
                    "layer_spacing_m": settings.min_distance_m,
                },
                fps=settings.dense_fps if settings.sampling_mode == config.SAMPLING_MODE_DENSE_SAMPLED else None,
                safety_radius_m=settings.safety_radius_m,
                min_distance_m=settings.min_distance_m,
                kinematic_constraints=kinematic_constraints,
            )

            data = intermediate_exporter.build_intermediate_data(metadata, keyframe_entries)
            errors = intermediate_exporter.validate_intermediate_data(data)
            if errors:
                for err in errors[:10]:
                    self.report({"ERROR"}, err)
                return {"CANCELLED"}

            filepath = bpy.path.abspath(settings.export_path)
            if settings.export_format == "JSON":
                intermediate_exporter.export_json(data, filepath)
            else:
                intermediate_exporter.export_msgpack(data, filepath)

            self.report({"INFO"}, f"Exported {len(keyframe_entries)} keyframes to {filepath}")
            return {"FINISHED"}

    _CLASSES = (DSS_OT_PreviewSample, DSS_OT_ExportIntermediate)

    def register():
        global _redraw_timer_running
        panel.register()
        for cls in _CLASSES:
            bpy.utils.register_class(cls)
        compass_gizmo.register()
        if not bpy.app.timers.is_registered(_blink_timer):
            bpy.app.timers.register(_blink_timer, first_interval=0.1)
            _redraw_timer_running = True
        if _on_frame_change_post not in bpy.app.handlers.frame_change_post:
            bpy.app.handlers.frame_change_post.append(_on_frame_change_post)

    def unregister():
        global _redraw_timer_running
        if _on_frame_change_post in bpy.app.handlers.frame_change_post:
            bpy.app.handlers.frame_change_post.remove(_on_frame_change_post)
        if _redraw_timer_running and bpy.app.timers.is_registered(_blink_timer):
            bpy.app.timers.unregister(_blink_timer)
            _redraw_timer_running = False
        compass_gizmo.unregister()
        viewport_drawer.unregister()
        for cls in reversed(_CLASSES):
            bpy.utils.unregister_class(cls)
        panel.unregister()

else:  # pragma: no cover - outside Blender

    def register():
        raise RuntimeError("stage1_designer.register() requires running inside Blender (bpy not found)")

    def unregister():
        pass
