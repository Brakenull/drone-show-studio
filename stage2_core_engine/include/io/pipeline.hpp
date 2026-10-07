#pragma once

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "config.hpp"
#include "io/project_loader.hpp"
#include "optimizer/scp_solver.hpp"
#include "progress.hpp"
#include "types.hpp"

// Orchestrates the full show: holding area -> keyframe[0] -> keyframe[1] ->
// ... (-> holding area, when the Phase 1 file has a return leg), chaining the Point Assignment Module and the SCP
// trajectory optimizer transition-by-transition while tracking
// each physical drone's persistent identity, outgoing velocity, and current
// LED color across transitions. This glue is not one of the 4 algorithmic
// algorithm modules; it exists to give bindings/py_bindings.cpp
// something end-to-end to call, and to assemble the output schema.

namespace drone_core::io {

// Actual show-time span of a takeoff or return leg, reported only when the Phase 1 file has `legs`.
struct LegTiming {
    double start_time_sec = 0.0;
    double end_time_sec = 0.0;
    std::optional<double> target_duration_sec;  // the designer's target; nullopt = Auto
};

// Planned vs flown timing of one transition. A transition
// that passes only after a gatekeeper retry is flown longer than planned; the
// show timeline follows the flown duration.
// The vertical pad moves of one transition, per drone count.
struct PadMoves {
    int parked = 0;   // flown to the hover point above their pad
    int landed = 0;   // descended onto their pad (in the transition or right after the last one)
    int climbed = 0;  // climbed from their pad to its hover point
    int hovered = 0;  // stayed at their hover point (the stop too short to land, or a descent given up)
    int old_way = 0;  // reached or left a pad the old way (a vertical move couldn't be used)
};

struct TransitionTiming {
    int index = 0;
    std::string from_keyframe;
    std::string to_keyframe;
    double start_time_sec = 0.0;
    double end_time_sec = 0.0;          // start + flown + staggered-launch waves
    double planned_duration_sec = 0.0;  // max(nominal, T_min) before any retry
    double flown_duration_sec = 0.0;    // the passing attempt's duration
    int attempts = 1;
    PadMoves pad_moves;
};

struct ShowMetadata {
    std::string version = "2.4.0";
    int fleet_size = 0;
    int spline_degree = 0;
    std::string continuity = "C4";
    double total_duration_sec = 0.0;
    std::string coordinate_system = "ENU";
    double min_distance_enforced_m = 0.0;
    std::optional<LegTiming> takeoff_leg;  // holding area -> keyframes[0]
    std::optional<LegTiming> return_leg;   // last keyframe -> holding area
    std::vector<TransitionTiming> transitions;  // every transition that passed, in order
    std::optional<double> altitude_floor_m;     // the file's ground_z_m, when it declares one
    std::optional<double> holding_clearance_m;  // the largest show_clearance_m of the holding areas, when declared
    std::vector<HoldingArea> holding_areas;     // as read, with slot_count
    std::vector<int> takeoff_area;              // per drone_id: the holding area it takes off from
};

struct PipelineResult {
    ShowMetadata metadata;
    std::vector<DroneTrajectory> trajectories;  // one entry per drone_id, ascending
};

// A gatekeeper rejection placed in show context.
// `solver` keeps its transition-local times exactly as optimizer::solve()
// produced them; add transition_start_time_sec for show time. Both
// trajectory lists use show time and the output segment format.
struct TransitionSafetyFailure {
    optimizer::SafetyViolationReport solver;
    int transition_index = 0;          // 0 = holding area -> keyframes[0]
    std::string from_keyframe;         // "holding_area" for transition 0
    std::string to_keyframe;           // "holding_area" for the return leg
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
//
// `progress`, when set, receives every
// ProgressEvent kind with the transition fields filled in.
PipelineResult run_pipeline(const ProjectData& project, const CoreConfig& config,
                            const ProgressCallback& progress = {});

// Return path from one formation of an already planned show to the
// holding-area slots. Planned after the show passed, from its saved result.
struct ReturnPathResult {
    // One transition (formation -> holding_area), timed from 0;
    // return_leg holds its span and target.
    ShowMetadata metadata;
    std::vector<DroneTrajectory> trajectories;  // one entry per drone_id, ascending
    int keyframe_index = 0;
    std::string from_keyframe;
    double abort_time_sec = 0.0;      // show time the return starts from (the formation, or a point before it)
    double worst_separation_m = 0.0;  // the passing attempt's, as the gatekeeper measured it
    double min_duration_sec = 0.0;    // the Auto duration (T_min, plus the final descent), before any retry
    int farthest_drone_id = -1;       // the drone with the longest way to its slot, which sets T_min
    int farthest_area = -1;           // the holding area that slot is in
    double farthest_distance_m = 0.0;
};

// `show` is the show's own result (trajectories in show time); `transitions`
// its metadata.transitions. Every drone's state at the end of transition
// `keyframe_index` (the one into project.keyframes[keyframe_index]) is the
// return's start. With `abort_time_sec` (show time strictly inside that
// transition, an abort point) the start is every drone's state then instead: position,
// velocity and acceleration, the fleet still moving. `target_duration_sec`
// nullopt = Auto (T_min), otherwise max(target, T_min) like a leg. Throws
// PipelineSafetyError when the gatekeeper rejects the return,
// std::runtime_error on bad input. `estimate_only` stops before solving:
// the result has no trajectories, only min_duration_sec and the farthest
// drone (the estimate the rain return suggestions use).
ReturnPathResult plan_return_path(const ProjectData& project, const CoreConfig& config,
                                  const std::vector<DroneTrajectory>& show,
                                  const std::vector<TransitionTiming>& transitions, int keyframe_index,
                                  std::optional<double> target_duration_sec,
                                  const ProgressCallback& progress = {},
                                  std::optional<double> abort_time_sec = std::nullopt, bool estimate_only = false);

}  // namespace drone_core::io
