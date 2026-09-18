#pragma once

#include <vector>

#include <Eigen/Dense>

// Point Assignment Module (docs/2-phase_2.md section 3.1): matches N drones
// at scene P to N target points at scene Q, minimizing the augmented cost
// C_ij = w_d*||Pi-Qj|| + w_z*max(0, Qjz-Piz) + w_heading*delta_theta_ij,
// solved with the Auction Algorithm (Bertsekas).

namespace drone_core::assignment {

struct AssignmentInput {
    Eigen::MatrixXd P;        // Nx3 positions at the end of the previous scene
    Eigen::MatrixXd Q;        // Nx3 target positions in the next scene
    Eigen::MatrixXd v_in_xy;  // Nx2 incoming XY velocity per drone in P
    double w_distance = 1.0;
    double w_vertical_climb = 2.5;
    double w_heading_change = 0.5;
};

struct AssignmentResult {
    std::vector<int> assignment;  // assignment[i] = matched column index in Q
    double total_cost = 0.0;
};

// v_in = [sin(heading_offset_rad), cos(heading_offset_rad)], used for every
// drone at the very first transition (holding area -> first keyframe), per
// docs/2-phase_2.md section 3.1.
Eigen::Vector2d initial_heading_velocity(double heading_offset_rad);

// Builds the NxN augmented cost matrix exactly as specified in section 3.1.
Eigen::MatrixXd build_cost_matrix(const AssignmentInput& input);

// Solves the resulting minimum-cost bipartite assignment with the forward
// Auction Algorithm, epsilon-scaled for near-optimality, with the per-person
// bidding step parallelized via OpenMP.
AssignmentResult solve_auction(const Eigen::MatrixXd& cost_matrix);

}  // namespace drone_core::assignment
