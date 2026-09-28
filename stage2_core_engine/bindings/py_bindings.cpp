#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cmath>
#include <exception>
#include <filesystem>
#include <optional>

#include "config.hpp"
#include "io/pipeline.hpp"
#include "io/project_loader.hpp"
#include "progress.hpp"
#include "py_json_convert.hpp"

#ifndef DRONE_CORE_CONFIG_DIR
#define DRONE_CORE_CONFIG_DIR "."
#endif

namespace py = pybind11;

namespace {

py::list control_points_to_py(const Eigen::MatrixXd& control_points) {
    py::list out;
    for (int i = 0; i < control_points.rows(); ++i) {
        out.append(py::make_tuple(control_points(i, 0), control_points(i, 1), control_points(i, 2)));
    }
    return out;
}

py::list knot_vector_to_py(const Eigen::VectorXd& knots) {
    py::list out;
    for (int i = 0; i < knots.size(); ++i) out.append(knots(i));
    return out;
}

py::list color_keyframes_to_py(const std::vector<drone_core::ColorKeyframe>& color_keyframes) {
    py::list out;
    for (const auto& ck : color_keyframes) {
        py::dict d;
        d["time_sec"] = ck.time_sec;
        d["color_rgb"] = py::make_tuple(ck.color_rgb.x(), ck.color_rgb.y(), ck.color_rgb.z());
        out.append(d);
    }
    return out;
}

py::tuple vec3_to_py(const Eigen::Vector3d& v) { return py::make_tuple(v.x(), v.y(), v.z()); }

py::dict metadata_to_py(const drone_core::io::ShowMetadata& meta) {
    py::dict metadata;
    metadata["version"] = meta.version;
    metadata["fleet_size"] = meta.fleet_size;
    metadata["spline_degree"] = meta.spline_degree;
    metadata["continuity"] = meta.continuity;
    metadata["total_duration_sec"] = meta.total_duration_sec;
    metadata["coordinate_system"] = meta.coordinate_system;
    metadata["min_distance_enforced_m"] = meta.min_distance_enforced_m;
    // Planned vs flown time per transition (bug-report P2-03).
    py::list transitions;
    for (const auto& t : meta.transitions) {
        py::dict d;
        d["index"] = t.index;
        d["from_keyframe"] = t.from_keyframe;
        d["to_keyframe"] = t.to_keyframe;
        d["start_time_sec"] = t.start_time_sec;
        d["end_time_sec"] = t.end_time_sec;
        d["planned_duration_sec"] = t.planned_duration_sec;
        d["flown_duration_sec"] = t.flown_duration_sec;
        d["attempts"] = t.attempts;
        transitions.append(d);
    }
    metadata["transitions"] = transitions;
    // The design's ground as enforced (docs/2-phase_2.md section 1.13); None = no floor.
    metadata["altitude_floor_m"] =
        meta.altitude_floor_m ? py::object(py::float_(*meta.altitude_floor_m)) : py::object(py::none());
    // The holding-area safe distance as enforced (section 1.14); None = no zone.
    metadata["holding_clearance_m"] =
        meta.holding_clearance_m ? py::object(py::float_(*meta.holding_clearance_m)) : py::object(py::none());
    // Only for Phase 1 files with `legs` (schema 1.6.0, 1-phase_1.md section 3.8).
    if (meta.takeoff_leg || meta.return_leg) {
        auto leg_to_py = [](const std::optional<drone_core::io::LegTiming>& leg) -> py::object {
            if (!leg) return py::none();  // e.g. a rejection before the return leg was flown
            py::dict d;
            d["start_time_sec"] = leg->start_time_sec;
            d["end_time_sec"] = leg->end_time_sec;
            d["duration_sec"] = leg->end_time_sec - leg->start_time_sec;
            d["target_duration_sec"] =
                leg->target_duration_sec ? py::object(py::float_(*leg->target_duration_sec)) : py::object(py::none());
            return d;
        };
        py::dict legs;
        legs["takeoff"] = leg_to_py(meta.takeoff_leg);
        legs["return"] = leg_to_py(meta.return_leg);
        metadata["legs"] = legs;
    }
    return metadata;
}

py::list trajectories_to_py(const std::vector<drone_core::DroneTrajectory>& trajectories) {
    py::list out;
    for (const auto& traj : trajectories) {
        py::dict traj_dict;
        traj_dict["drone_id"] = traj.drone_id;

        py::list segments;
        for (const auto& seg : traj.segments) {
            py::dict seg_dict;
            seg_dict["segment_index"] = seg.segment_index;
            seg_dict["start_time_sec"] = seg.start_time_sec;
            seg_dict["end_time_sec"] = seg.end_time_sec;
            seg_dict["knot_vector"] = knot_vector_to_py(seg.knot_vector);
            seg_dict["control_points"] = control_points_to_py(seg.control_points);
            seg_dict["color_keyframes"] = color_keyframes_to_py(seg.color_keyframes);
            segments.append(seg_dict);
        }
        traj_dict["segments"] = segments;
        out.append(traj_dict);
    }
    return out;
}

// SafetyViolationError.report (docs/5-studio_gui.md section 5.1, B1). All
// times are show time. "rejected" and "completed" are each a full section 5
// output dict (metadata + trajectories) so any contract consumer
// (arrow_loader.py, the Studio replay) can load them unchanged.
py::dict safety_failure_to_py(const drone_core::io::TransitionSafetyFailure& f) {
    const double t0 = f.transition_start_time_sec;

    py::dict transition;
    transition["index"] = f.transition_index;
    transition["from_keyframe"] = f.from_keyframe;
    transition["to_keyframe"] = f.to_keyframe;
    transition["start_time_sec"] = t0;
    transition["duration_sec"] = f.transition_duration_sec;

    py::list attempts;
    for (size_t i = 0; i < f.solver.attempts.size(); ++i) {
        py::dict a;
        a["attempt"] = static_cast<int>(i + 1);
        a["duration_sec"] = f.solver.attempts[i].duration_sec;
        a["worst_separation_m"] = f.solver.attempts[i].worst_separation_m;
        attempts.append(a);
    }

    py::list violations;
    for (const auto& v : f.solver.violations) {
        py::dict d;
        d["drone_a"] = v.drone_a;
        d["drone_b"] = v.drone_b;
        d["time_sec"] = t0 + v.time_sec;
        d["distance_m"] = v.distance_m;
        d["position_a"] = vec3_to_py(v.position_a);
        d["position_b"] = vec3_to_py(v.position_b);
        violations.append(d);
    }

    drone_core::io::ShowMetadata completed_meta = f.metadata;
    completed_meta.total_duration_sec = t0;

    py::dict rejected;
    rejected["metadata"] = metadata_to_py(f.metadata);
    rejected["trajectories"] = trajectories_to_py(f.rejected_trajectories);
    py::dict completed;
    completed["metadata"] = metadata_to_py(completed_meta);
    completed["trajectories"] = trajectories_to_py(f.completed_trajectories);

    py::dict report;
    report["transition"] = transition;
    report["worst_separation_m"] = f.solver.worst_separation_m;
    report["required_separation_m"] = f.solver.required_separation_m;
    report["enforced_min_distance_m"] = f.solver.enforced_min_distance_m;
    report["verification_frequency_hz"] = f.solver.verification_frequency_hz;
    report["attempts"] = attempts;
    report["violating_pair_count"] = f.solver.violating_pair_count;
    report["violations"] = violations;
    report["violations_truncated"] = f.solver.violating_pair_count > static_cast<int>(f.solver.violations.size());
    report["rejected"] = rejected;
    report["completed"] = completed;
    return report;
}

// progress_callback's argument (docs/5-studio_gui.md B2; the key list per
// "event" is in docs/2-phase_2.md section 5, "Progress events"). Only the keys
// that mean something for the event's kind are set; +infinity becomes None.
py::dict progress_event_to_py(const drone_core::ProgressEvent& e) {
    using Kind = drone_core::ProgressEvent::Kind;
    const auto finite_or_none = [](double v) -> py::object {
        return std::isfinite(v) ? py::object(py::float_(v)) : py::object(py::none());
    };

    py::dict d;
    switch (e.kind) {
        case Kind::TransitionStart: d["event"] = "transition_start"; break;
        case Kind::AttemptStart: d["event"] = "attempt_start"; break;
        case Kind::ScpIteration: d["event"] = "scp_iteration"; break;
        case Kind::AttemptEnd: d["event"] = "attempt_end"; break;
        case Kind::TransitionEnd: d["event"] = "transition_end"; break;
    }
    d["transition"] = e.transition_index;
    d["transition_count"] = e.transition_count;
    d["from_keyframe"] = e.from_keyframe;
    d["to_keyframe"] = e.to_keyframe;

    if (e.kind == Kind::TransitionStart || e.kind == Kind::TransitionEnd) {
        d["show_time_sec"] = e.show_time_sec;
        return d;
    }
    d["attempt"] = e.attempt;
    d["max_attempts"] = e.max_attempts;
    d["duration_sec"] = e.duration_sec;
    if (e.kind == Kind::ScpIteration) {
        d["substage"] = e.substage;
        d["substage_count"] = e.substage_count;
        d["iteration"] = e.iteration;
        d["max_iterations"] = e.max_iterations;
        d["conflict_pairs"] = e.conflict_pairs;
        d["max_delta_m"] = e.max_delta_m;
        d["min_separation_m"] = finite_or_none(e.min_separation_m);
        d["converged"] = e.converged;
    } else if (e.kind == Kind::AttemptEnd) {
        d["worst_separation_m"] = finite_or_none(e.worst_separation_m);
        d["required_separation_m"] = e.required_separation_m;
        d["passed"] = e.passed;
    }
    return d;
}

// Output schema (docs/2-phase_2.md Rev 2.3 section 5): matches Phase 3's
// arrow_loader.py / spline_evaluator.py contract.
//
// {
//   "metadata": {"version", "fleet_size", "spline_degree", "continuity",
//                "total_duration_sec", "coordinate_system",
//                "min_distance_enforced_m",
//                "transitions": [{"index", "from_keyframe", "to_keyframe", "start_time_sec",
//                                 "end_time_sec", "planned_duration_sec", "flown_duration_sec",
//                                 "attempts"}, ...],
//                "altitude_floor_m" (the file's ground_z_m, or None: no floor),
//                "holding_clearance_m" (the file's holding_area.show_clearance_m, or None: no zone),
//                "legs" (only for Phase 1 files with legs): {"takeoff", "return"}:
//                  {"start_time_sec", "end_time_sec", "duration_sec", "target_duration_sec"}},
//   "trajectories": [{"drone_id", "segments": [{"segment_index",
//                     "start_time_sec", "end_time_sec", "knot_vector",
//                     "control_points", "color_keyframes"}, ...]}, ...]
// }
py::dict optimize_trajectories(const py::dict& phase1_intermediate_json, const py::dict& optional_config_overrides,
                                const py::object& progress_callback) {
    const nlohmann::json phase1_json = drone_core::bindings::py_to_json(phase1_intermediate_json);
    const nlohmann::json overrides_json = drone_core::bindings::py_to_json(optional_config_overrides);

    const drone_core::io::ProjectData project = drone_core::io::parse_project(phase1_json);

    const std::filesystem::path default_config_path = std::filesystem::path(DRONE_CORE_CONFIG_DIR) / "core_config.json";
    const drone_core::CoreConfig config = drone_core::resolve_core_config(
        std::optional<nlohmann::json>(project.metadata.raw), overrides_json, default_config_path);

    // The solve runs without the GIL so other Python threads keep running
    // during a long show; the callback takes it back for each event. An
    // exception raised by the callback propagates out of run_pipeline() as
    // py::error_already_set (never from inside an OpenMP region; see
    // progress.hpp) and aborts the solve.
    drone_core::ProgressCallback progress;
    if (!progress_callback.is_none()) {
        progress = [&progress_callback](const drone_core::ProgressEvent& e) {
            py::gil_scoped_acquire gil;
            progress_callback(progress_event_to_py(e));
        };
    }
    drone_core::io::PipelineResult pipeline_result;
    {
        py::gil_scoped_release no_gil;
        pipeline_result = drone_core::io::run_pipeline(project, config, progress);
    }

    py::dict out;
    out["metadata"] = metadata_to_py(pipeline_result.metadata);
    out["trajectories"] = trajectories_to_py(pipeline_result.trajectories);
    return out;
}

}  // namespace

PYBIND11_MODULE(drone_core, m) {
    m.doc() = "Drone Show Studio Phase 2 Core Engine (assignment + quintic B-spline Gauss-Seidel SCP optimizer)";

    // RuntimeError subclass, so existing `except RuntimeError` callers keep
    // working; str(e) is the unchanged gatekeeper message and e.report holds
    // the structured diagnostics (see safety_failure_to_py above).
    py::object safety_error = py::reinterpret_steal<py::object>(PyErr_NewExceptionWithDoc(
        "drone_core.SafetyViolationError",
        "Raised when the continuous gatekeeper rejects a transition after its retry budget. "
        "`report` holds the structured diagnostics (docs/5-studio_gui.md B1).",
        PyExc_RuntimeError, nullptr));
    if (!safety_error) throw py::error_already_set();
    m.attr("SafetyViolationError") = safety_error;

    // Looked up through the module at translation time instead of caching a
    // py::object in static storage (which would outlive the interpreter).
    py::register_exception_translator([](std::exception_ptr p) {
        try {
            if (p) std::rethrow_exception(p);
        } catch (const drone_core::io::PipelineSafetyError& e) {
            py::object type = py::module_::import("drone_core").attr("SafetyViolationError");
            py::object instance = type(py::str(e.what()));
            instance.attr("report") = safety_failure_to_py(e.failure());
            PyErr_SetObject(type.ptr(), instance.ptr());
        }
    });

    m.def("optimize_trajectories", &optimize_trajectories, py::arg("phase1_intermediate_json"),
          py::arg("optional_config_overrides") = py::dict(), py::arg("progress_callback") = py::none(),
          "Assigns drones scene-to-scene and solves collision-free, kinematically-bounded quintic "
          "B-spline trajectories for the whole show. Raises drone_core.SafetyViolationError (a "
          "RuntimeError) with a `report` dict when a transition fails the continuous gatekeeper. "
          "`progress_callback`, if given, is called with one dict per progress event (transition "
          "start/end, gatekeeper attempt start/end, SCP iteration); an exception it raises aborts "
          "the solve and is re-raised.");
}
