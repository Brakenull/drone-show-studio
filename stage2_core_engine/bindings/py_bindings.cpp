#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <filesystem>
#include <optional>

#include "config.hpp"
#include "io/pipeline.hpp"
#include "io/project_loader.hpp"
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

// Output schema (docs/2-phase_2.md Rev 2.3 section 5): matches Phase 3's
// arrow_loader.py / spline_evaluator.py contract.
//
// {
//   "metadata": {"version", "fleet_size", "spline_degree", "continuity",
//                "total_duration_sec", "coordinate_system",
//                "min_distance_enforced_m"},
//   "trajectories": [{"drone_id", "segments": [{"segment_index",
//                     "start_time_sec", "end_time_sec", "knot_vector",
//                     "control_points", "color_keyframes"}, ...]}, ...]
// }
py::dict optimize_trajectories(const py::dict& phase1_intermediate_json,
                                const py::dict& optional_config_overrides) {
    const nlohmann::json phase1_json = drone_core::bindings::py_to_json(phase1_intermediate_json);
    const nlohmann::json overrides_json = drone_core::bindings::py_to_json(optional_config_overrides);

    const drone_core::io::ProjectData project = drone_core::io::parse_project(phase1_json);

    const std::filesystem::path default_config_path = std::filesystem::path(DRONE_CORE_CONFIG_DIR) / "core_config.json";
    const drone_core::CoreConfig config = drone_core::resolve_core_config(
        std::optional<nlohmann::json>(project.metadata.raw), overrides_json, default_config_path);

    const drone_core::io::PipelineResult pipeline_result = drone_core::io::run_pipeline(project, config);

    py::dict metadata;
    metadata["version"] = pipeline_result.metadata.version;
    metadata["fleet_size"] = pipeline_result.metadata.fleet_size;
    metadata["spline_degree"] = pipeline_result.metadata.spline_degree;
    metadata["continuity"] = pipeline_result.metadata.continuity;
    metadata["total_duration_sec"] = pipeline_result.metadata.total_duration_sec;
    metadata["coordinate_system"] = pipeline_result.metadata.coordinate_system;
    metadata["min_distance_enforced_m"] = pipeline_result.metadata.min_distance_enforced_m;

    py::list trajectories;
    for (const auto& traj : pipeline_result.trajectories) {
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
        trajectories.append(traj_dict);
    }

    py::dict out;
    out["metadata"] = metadata;
    out["trajectories"] = trajectories;
    return out;
}

}  // namespace

PYBIND11_MODULE(drone_core, m) {
    m.doc() = "Drone Show Studio Phase 2 Core Engine (assignment + quintic B-spline Gauss-Seidel SCP optimizer)";
    m.def("optimize_trajectories", &optimize_trajectories, py::arg("phase1_intermediate_json"),
          py::arg("optional_config_overrides") = py::dict(),
          "Assigns drones scene-to-scene and solves collision-free, kinematically-bounded quintic "
          "B-spline trajectories for the whole show.");
}
