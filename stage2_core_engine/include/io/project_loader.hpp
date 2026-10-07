#pragma once

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

// Glue between the Phase 1 intermediate JSON contract
// (schemas/project_intermediate.schema.json) and the Core Engine's
// algorithmic modules. Not one of the 4 algorithm modules — this is the integration layer the pybind11
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
    // The launch-grid pitch d_launch,
    // deliberately larger than the in-flight min_distance_m — see Phase 1's
    // config.py DEFAULT_GRID_SPACING_M docstring. Falls back to
    // layer_spacing_m for older exports without this
    // field (see parse_project()).
    double grid_spacing_m = 0.0;
    // The vertical gap between stacked layers. Honoured since Phase 1 schema
    // 1.7.0; older files carry grid_spacing_m
    // here, which is the straight stacking they were laid out with.
    double layer_spacing_m = 0.0;
    // Phase 1 schema 1.7.0 (optional, false when absent): odd layers shifted
    // half a slot in x and y, one column and one row fewer.
    bool staggered_layers = false;
    // Phase 1 schema 1.6.0 (optional): the designer's "Safe Distance to Show".
    // When set, show drones keep at least this
    // far from the holding region.
    std::optional<double> show_clearance_m;
    // The drones whose home area this is (Phase 1 schema 1.8.0): they take
    // off from, land in, park on and return to this area only. A file with
    // the old single holding_area has one area holding the whole fleet.
    int slot_count = 0;
};

// The holding region (a port of Phase 1's
// holding_region_bounds()): the declared volume (footprint, widened if the
// fleet needed it, from center z up to max_height) together with the parked
// slot grid padded by half a grid step on every side. ENU, axis-aligned.
struct HoldingRegion {
    Eigen::Vector3d lo = Eigen::Vector3d::Zero();
    Eigen::Vector3d hi = Eigen::Vector3d::Zero();
};

// Phase 1 schema 1.7.0's optional project_metadata.waiting_areas:
// places in the air
// where spare drones wait instead of flying home. One flat layer of
// `slot_count` slots at center z (the declared grid, grown compactly on its
// shorter side when slot_count needs it); a port of
// stage1_designer/core/waiting_area.py.
struct WaitingArea {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector2d size = Eigen::Vector2d::Zero();
    double grid_spacing_m = 2.0;
    double show_clearance_m = 0.0;
    int slot_count = 0;
};

// Phase 1 schema 1.6.0's optional project_metadata.legs:
// the takeoff (holding area -> keyframes[0]) and return (last
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
    // Phase 1 schema 1.8.0's holding_areas, or the older single holding_area
    // as a one-item list. Their slot counts add up to fleet_size; drone i
    // takes off from slot i of every area's slots concatenated in list order.
    std::vector<HoldingArea> holding_areas;
    ShowLegs legs;
    // Phase 1 schema 1.6.0 (optional): ENU height of the ground.
    // nullopt for files that don't declare one.
    std::optional<double> ground_z_m;
    // Phase 1 schema 1.7.0 (optional): empty for files without waiting areas.
    std::vector<WaitingArea> waiting_areas;
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

// Staggered Wave Takeoff needs each parked drone's
// "Launch Row Index" (its row within its layer of the same grid
// compute_holding_positions() lays out) — returned in the same slot order
// as compute_holding_positions()'s rows.
std::vector<int> compute_holding_row_indices(int fleet_size, const HoldingArea& holding_area, double grid_spacing_m);

HoldingRegion compute_holding_region(int fleet_size, const HoldingArea& holding_area, double grid_spacing_m);

// Several holding areas, each laid out for its own slot_count drones with
// its own grid spacing: every area's slots (and their launch row indices)
// concatenated in list order, one region per area, and each drone's home
// area (the area of its takeoff slot).
Eigen::MatrixXd compute_all_holding_positions(const std::vector<HoldingArea>& areas);
std::vector<int> compute_all_holding_row_indices(const std::vector<HoldingArea>& areas);
std::vector<HoldingRegion> compute_holding_regions(const std::vector<HoldingArea>& areas);
std::vector<int> compute_home_areas(const std::vector<HoldingArea>& areas);

// One waiting area's slots (its slot_count of them, row by row) and region
// (the footprint, widened if needed, at center z, together with the slots
// padded by half a grid step): ports of Phase 1's compute_waiting_positions()
// and waiting_region_bounds().
Eigen::MatrixXd compute_waiting_positions(const WaitingArea& area);
HoldingRegion compute_waiting_region(const WaitingArea& area);
// Every area's slots, concatenated in area order.
Eigen::MatrixXd compute_all_waiting_slots(const std::vector<WaitingArea>& areas);

}  // namespace drone_core::io
