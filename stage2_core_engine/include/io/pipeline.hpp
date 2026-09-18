#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "config.hpp"
#include "io/project_loader.hpp"
#include "types.hpp"

// Orchestrates the full show: holding area -> keyframe[0] -> keyframe[1] ->
// ... , chaining the Point Assignment Module (section 3.1) and the SCP
// trajectory optimizer (section 3.3) transition-by-transition while tracking
// each physical drone's persistent identity, outgoing velocity, and current
// LED color across transitions. This glue is not one of the 4 algorithmic
// modules in section 5's file tree; it exists to give bindings/py_bindings.cpp
// something end-to-end to call, and to assemble the section 5 output schema.

namespace drone_core::io {

struct ShowMetadata {
    std::string version = "2.4.0";
    int fleet_size = 0;
    int spline_degree = 0;
    std::string continuity = "C4";
    double total_duration_sec = 0.0;
    std::string coordinate_system = "ENU";
    double min_distance_enforced_m = 0.0;
};

struct PipelineResult {
    ShowMetadata metadata;
    std::vector<DroneTrajectory> trajectories;  // one entry per drone_id, ascending
};

PipelineResult run_pipeline(const ProjectData& project, const CoreConfig& config);

}  // namespace drone_core::io
