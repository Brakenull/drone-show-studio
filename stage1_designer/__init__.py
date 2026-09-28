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
    "version": (1, 6, 0),
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
    from .core import color_extractor, holding_area, kinematic_validator, sampler, show_legs, timeline_sampler
    from .exporters import intermediate_exporter
    from .ui import compass_gizmo, ground_scene, holding_area_scene, panel, viewport_drawer

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
                grid_spacing_m=settings.holding_area.grid_spacing_m,
            )
            all_positions = np.vstack([enu_points, holding_positions]) if n_sampled else holding_positions
            all_colors = list(colors) + [color_extractor.BLACK_RGB8] * n_park
        else:
            all_positions = enu_points
            all_colors = list(colors)

        entry = intermediate_exporter.build_keyframe_entry(
            time_sec, shape_name, all_positions, all_colors
        )
        return entry, enu_points

    def _shape_name_for_frame(scene, frame: int) -> str:
        for marker in scene.timeline_markers:
            if marker.frame == frame:
                return marker.name
        return f"Shape_{frame}"

    def _configured_v_max(settings) -> float:
        return panel.configured_v_max(settings)

    def _sample_all_keyframes_for_validation(settings, obj, scene, times, shape_name_fn):
        """Sample every keyframe exactly as Export would (same function, same
        holding-area padding) and return (keyframe_entries, transitions) so
        Export, Check Kinematics and Auto-Fix all validate against identical
        data — there's only one code path that decides what v_req actually is.

        Also refreshes the panel's formation cache (each keyframe's sampled
        ENU points, parked drones excluded) for the holding-area clearance
        check (spec section 3.2.2), and the first / last keyframe's full
        positions for the takeoff / return estimates (spec section 3.8)."""
        original_frame = scene.frame_current
        try:
            entries = []
            formations = []
            for time_sec in times:
                frame = int(round(time_sec * (scene.render.fps / scene.render.fps_base)))
                shape_name = shape_name_fn(scene, frame)
                entry, formation_points = _build_keyframe_for_time(settings, obj, scene, time_sec, shape_name)
                entries.append(entry)
                formations.append((shape_name, formation_points))
        finally:
            scene.frame_set(original_frame)

        panel.set_formation_cache(formations)
        positions = [np.array([p["pos"] for p in kf["points"]]) for kf in entries]
        panel.set_leg_cache(positions[0], positions[-1], float(times[-1] - times[0]))
        ground_scene.set_show_extent([pts for _name, pts in formations])
        ground_scene.sync(scene)
        transitions = kinematic_validator.evaluate_transitions(list(times), positions, _configured_v_max(settings))
        return entries, transitions

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
                grid_spacing_m=settings.holding_area.grid_spacing_m,
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

    def _get_target_and_times(self, context):
        """Shared setup for the kinematic-check / export / auto-fix operators:
        validates a target object is set and returns (scene, settings, obj,
        times), or reports an error and returns None."""
        scene = context.scene
        settings = scene.drone_show_settings
        obj = settings.target_object
        if obj is None:
            self.report({"ERROR"}, "No target object set")
            return None

        try:
            times = timeline_sampler.get_sample_times_seconds(
                obj,
                scene,
                settings.sampling_mode,
                fps=settings.dense_fps if settings.sampling_mode == config.SAMPLING_MODE_DENSE_SAMPLED else None,
            )
        except ValueError as exc:
            self.report({"ERROR"}, str(exc))
            return None

        if not times:
            self.report({"ERROR"}, "No keyframes found on target object")
            return None

        return scene, settings, obj, times

    class DSS_OT_CheckKinematics(bpy.types.Operator):
        bl_idname = "dss.check_kinematics"
        bl_label = "Check Kinematics"
        bl_description = "Sample all keyframes and re-check required transition speeds against v_max"

        def execute(self, context):
            setup = _get_target_and_times(self, context)
            if setup is None:
                return {"CANCELLED"}
            scene, settings, obj, times = setup

            try:
                _entries, transitions = _sample_all_keyframes_for_validation(
                    settings, obj, scene, times, _shape_name_for_frame
                )
            except ImportError as exc:
                self.report({"ERROR"}, str(exc))
                return {"CANCELLED"}

            panel.set_kinematic_cache(transitions)
            _redraw_all_view3d()
            if kinematic_validator.has_error(transitions):
                self.report({"ERROR"}, "One or more transitions exceed v_max — see the panel for details")
            else:
                self.report({"INFO"}, f"Checked {len(transitions)} transition(s), all within budget")
            return {"FINISHED"}

    class DSS_OT_AutoFixTimeline(bpy.types.Operator):
        bl_idname = "dss.auto_fix_timeline"
        bl_label = "Auto-Fix Timeline Timing"
        bl_description = (
            "Stretch unsafe keyframe transitions to the minimum duration v_max allows "
            "(plus collision-avoidance slack), without changing the 3D shapes"
        )

        def execute(self, context):
            setup = _get_target_and_times(self, context)
            if setup is None:
                return {"CANCELLED"}
            scene, settings, obj, times = setup

            try:
                entries, transitions = _sample_all_keyframes_for_validation(
                    settings, obj, scene, times, _shape_name_for_frame
                )
            except ImportError as exc:
                self.report({"ERROR"}, str(exc))
                return {"CANCELLED"}

            if not kinematic_validator.has_error(transitions):
                panel.set_kinematic_cache(transitions)
                self.report({"INFO"}, "Nothing to fix — every transition is already within v_max")
                return {"FINISHED"}

            positions = [np.array([p["pos"] for p in kf["points"]]) for kf in entries]
            v_max = _configured_v_max(settings)
            fixed_times = kinematic_validator.auto_fix_times(times, positions, v_max)

            scene_fps = scene.render.fps / scene.render.fps_base
            frame_mapping = {
                int(round(old_t * scene_fps)): int(round(new_t * scene_fps))
                for old_t, new_t in zip(times, fixed_times)
            }
            moved_keyframes = timeline_sampler.shift_keyframes_to_frames(obj, frame_mapping)
            timeline_sampler.shift_timeline_markers_to_frames(scene, frame_mapping)

            try:
                _entries2, transitions2 = _sample_all_keyframes_for_validation(
                    settings, obj, scene, fixed_times, _shape_name_for_frame
                )
            except ImportError as exc:
                self.report({"ERROR"}, str(exc))
                return {"CANCELLED"}
            panel.set_kinematic_cache(transitions2)
            _redraw_all_view3d()

            self.report(
                {"INFO"},
                f"Stretched timeline: {times[0]:.2f}-{times[-1]:.2f}s -> "
                f"{fixed_times[0]:.2f}-{fixed_times[-1]:.2f}s ({moved_keyframes} keyframe points moved)",
            )
            return {"FINISHED"}

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
            setup = _get_target_and_times(self, context)
            if setup is None:
                return {"CANCELLED"}
            scene, settings, obj, times = setup

            try:
                keyframe_entries, transitions = _sample_all_keyframes_for_validation(
                    settings, obj, scene, times, _shape_name_for_frame
                )
            except ImportError as exc:
                self.report({"ERROR"}, str(exc))
                return {"CANCELLED"}

            panel.set_kinematic_cache(transitions)
            _redraw_all_view3d()

            # Kinematic Gatekeeping (spec section 3.5): re-checked here
            # regardless of whether the user ran Check Kinematics first —
            # the panel's Export button being enabled is only a UI hint, this
            # is the actual, unconditional gate.
            if kinematic_validator.has_error(transitions):
                for t in transitions:
                    if t.status == kinematic_validator.STATUS_ERROR:
                        self.report(
                            {"ERROR"},
                            f"Keyframe transit [{t.from_index} -> {t.to_index}] requires "
                            f"~{t.v_req:.1f} m/s (Limit: {_configured_v_max(settings):.1f} m/s)",
                        )
                return {"CANCELLED"}

            # Holding-area clearance gate (spec section 3.2.2): same rule as
            # above - _sample_all_keyframes_for_validation just refreshed the
            # formation cache, so this checks the show being exported now.
            cautions = panel.clearance_cautions(settings)
            if cautions:
                for r in cautions:
                    self.report({"ERROR"}, panel.clearance_message(r, settings.holding_area.show_clearance_m))
                self.report(
                    {"ERROR"},
                    f"Export blocked: {len(cautions)} formation(s) closer than "
                    f"{settings.holding_area.show_clearance_m:g} m to the holding area",
                )
                return {"CANCELLED"}

            # Ground gate (spec section 3.9): formation points (freshly
            # sampled above) and parked drones below the ground.
            below_ground = panel.ground_warnings(settings)
            if below_ground:
                for msg in below_ground:
                    self.report({"ERROR"}, msg)
                self.report(
                    {"ERROR"},
                    f"Export blocked: drones below the ground (z = {settings.ground_z_m:g} m)",
                )
                return {"CANCELLED"}

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
                    "grid_spacing_m": settings.holding_area.grid_spacing_m,
                    "layer_spacing_m": settings.holding_area.grid_spacing_m,
                },
                fps=settings.dense_fps if settings.sampling_mode == config.SAMPLING_MODE_DENSE_SAMPLED else None,
                safety_radius_m=settings.safety_radius_m,
                min_distance_m=settings.min_distance_m,
                kinematic_constraints=kinematic_constraints,
                takeoff_duration_sec=show_legs.target_duration(
                    settings.legs.takeoff_mode, settings.legs.takeoff_duration_sec
                ),
                return_duration_sec=show_legs.target_duration(
                    settings.legs.return_mode, settings.legs.return_duration_sec
                ),
                ground_z_m=settings.ground_z_m,
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

            # Leg targets below the estimated minimum (spec section 3.8): a
            # warning only - the estimate is rough and Stage 2 stretches the
            # leg to its real minimum anyway.
            short_legs = panel.short_leg_messages(settings)
            for msg in short_legs:
                self.report({"WARNING"}, msg)
            self.report(
                {"WARNING"} if short_legs else {"INFO"},
                f"Exported {len(keyframe_entries)} keyframes to {filepath}",
            )
            return {"FINISHED"}

    _CLASSES = (
        DSS_OT_PreviewSample,
        DSS_OT_CheckKinematics,
        DSS_OT_AutoFixTimeline,
        DSS_OT_ExportIntermediate,
    )

    def register():
        global _redraw_timer_running
        panel.register()
        for cls in _CLASSES:
            bpy.utils.register_class(cls)
        compass_gizmo.register()
        holding_area_scene.register()
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
        holding_area_scene.unregister()
        ground_scene.remove()
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
