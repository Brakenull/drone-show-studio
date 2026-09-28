#pragma once

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

// Glue between the Phase 1 intermediate JSON contract
// (schemas/project_intermediate.schema.json) and the Core Engine's
// algorithmic modules. Not one of the 4 modules enumerated in
// docs/2-phase_2.md section 5 — this is the integration layer the pybind11
// entry point (bindings/py_bindings.cpp) needs to turn that JSON into the
// per-transition assignment/trajectory problems those modules consume.

namespace drone_core::io {

struct Point {
    int index = 0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3i color = Eigen::Vector3i::Zero();
};

struct Keyframe {
    double time_sec = 0.0;
    std::string shape_name;
    std::vector<Point> points;
};

struct HoldingArea {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector2d size = Eigen::Vector2d::Zero();
    double max_height = 0.0;
    // Rev 1.5 (Phase 1) / Rev 2.6 (Phase 2): the launch-grid pitch d_launch,
    // deliberately larger than the in-flight min_distance_m — see Phase 1's
    // config.py DEFAULT_GRID_SPACING_M docstring. Falls back to
    // layer_spacing_m for exports from before Phase 1 Rev 1.5 added this
    // field (see parse_project()).
    double grid_spacing_m = 0.0;
    double layer_spacing_m = 0.0;
    // Phase 1 schema 1.6.0 (optional): the designer's "Safe Distance to Show"
    // (1-phase_1.md section 3.2.2). When set, show drones keep at least this
    // far from the holding region (docs/2-phase_2.md section 1.14).
    std::optional<double> show_clearance_m;
};

// The holding region (1-phase_1.md section 3.2, a port of Phase 1's
// holding_region_bounds()): the declared volume (footprint, widened if the
// fleet needed it, from center z up to max_height) together with the parked
// slot grid padded by half a grid step on every side. ENU, axis-aligned.
struct HoldingRegion {
    Eigen::Vector3d lo = Eigen::Vector3d::Zero();
    Eigen::Vector3d hi = Eigen::Vector3d::Zero();
};

// Phase 1 schema 1.6.0's optional project_metadata.legs (1-phase_1.md
// section 3.8): the takeoff (holding area -> keyframes[0]) and return (last
// keyframe -> holding area) legs. A duration of nullopt means Auto (fly the
// leg in its minimum time); a target is flown as max(target, T_min).
struct ShowLegs {
    bool present = false;  // false: pre-1.6.0 file, legacy timing and no return leg
    std::optional<double> takeoff_duration_sec;
    std::optional<double> return_duration_sec;
};

struct ProjectMetadata {
    int fleet_size = 0;
    std::string sampling_mode;
    double total_duration_sec = 0.0;
    double safety_radius_m = 0.0;
    double min_distance_m = 0.0;
    double heading_offset_deg = 0.0;
    HoldingArea holding_area;
    ShowLegs legs;
    // Phase 1 schema 1.6.0 (optional): ENU height of the ground (1-phase_1.md
    // section 3.9). nullopt for files that don't declare one.
    std::optional<double> ground_z_m;
    nlohmann::json raw;  // kept for CoreConfig Tier-1 override lookup
};

struct ProjectData {
    ProjectMetadata metadata;
    std::vector<Keyframe> keyframes;  // sorted by time_sec, size >= 1
};

// Parses and validates the required fields of the Phase 1 intermediate JSON
// (already schema-validated Python-side; this only extracts what Phase 2
// needs). Throws std::runtime_error on structurally missing required data.
ProjectData parse_project(const nlohmann::json& phase1_intermediate_json);

// Ports stage1_designer/core/holding_area.py's layout algorithm to C++ so
// the very first transition (holding area -> keyframes[0]) starts from the
// same physical launch-grid positions the Blender add-on lays out.
Eigen::MatrixXd compute_holding_positions(int fleet_size, const HoldingArea& holding_area, double grid_spacing_m);

// Rev 2.6 section 1.7's Staggered Wave Takeoff needs each parked drone's
// "Launch Row Index" (its row within its layer of the same grid
// compute_holding_positions() lays out) — returned in the same slot order
// as compute_holding_positions()'s rows.
std::vector<int> compute_holding_row_indices(int fleet_size, const HoldingArea& holding_area, double grid_spacing_m);

HoldingRegion compute_holding_region(int fleet_size, const HoldingArea& holding_area, double grid_spacing_m);

}  // namespace drone_core::io
