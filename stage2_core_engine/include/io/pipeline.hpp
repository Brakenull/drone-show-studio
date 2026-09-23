#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "config.hpp"
#include "io/project_loader.hpp"
#include "optimizer/scp_solver.hpp"
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

// A gatekeeper rejection placed in show context (docs/5-studio_gui.md B1).
// `solver` keeps its transition-local times exactly as optimizer::solve()
// produced them; add transition_start_time_sec for show time. Both
// trajectory lists use show time and the section 5 output segment format.
struct TransitionSafetyFailure {
    optimizer::SafetyViolationReport solver;
    int transition_index = 0;          // 0 = holding area -> keyframes[0]
    std::string from_keyframe;         // "holding_area" for transition 0
    std::string to_keyframe;
    double transition_start_time_sec = 0.0;
    double transition_duration_sec = 0.0;  // rejected attempt's (possibly expanded) duration
    ShowMetadata metadata;             // total_duration_sec = end of the rejected attempt
    // The rejected attempt, one entry per drone_id, ascending. Synchronized
    // (unstaggered): exactly the configuration the gatekeeper rejected.
    std::vector<DroneTrajectory> rejected_trajectories;
    // Every transition that did pass, one entry per drone_id, ascending.
    std::vector<DroneTrajectory> completed_trajectories;
};

class PipelineSafetyError : public std::runtime_error {
public:
    PipelineSafetyError(const std::string& message, TransitionSafetyFailure failure)
        : std::runtime_error(message), failure_(std::make_shared<const TransitionSafetyFailure>(std::move(failure))) {}
    const TransitionSafetyFailure& failure() const noexcept { return *failure_; }

private:
    std::shared_ptr<const TransitionSafetyFailure> failure_;
};

// Throws PipelineSafetyError when a transition fails the continuous
// gatekeeper; other errors stay std::runtime_error.
PipelineResult run_pipeline(const ProjectData& project, const CoreConfig& config);

}  // namespace drone_core::io
