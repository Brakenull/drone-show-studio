#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

// Shared output types (docs/2-phase_2.md section 5): the final per-drone,
// per-segment trajectory representation returned by optimize_trajectories()
// to Phase 3 (arrow_loader.py / spline_evaluator.py).

namespace drone_core {

struct ColorKeyframe {
    double time_sec = 0.0;
    Eigen::Vector3i color_rgb = Eigen::Vector3i::Zero();
};

// One clamped quintic B-spline transition for a single drone, fully
// self-contained (its own knot vector) so a consumer can evaluate it without
// recomputing clamped_knot_vector().
struct TrajectorySegment {
    int segment_index = 0;
    double start_time_sec = 0.0;
    double end_time_sec = 0.0;
    Eigen::VectorXd knot_vector;
    Eigen::MatrixXd control_points;  // num_control_points x 3
    std::vector<ColorKeyframe> color_keyframes;
};

struct DroneTrajectory {
    int drone_id = 0;
    std::vector<TrajectorySegment> segments;
};

}  // namespace drone_core
