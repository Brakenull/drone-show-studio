#include "assignment/lap_auction.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace drone_core::assignment {

Eigen::Vector2d initial_heading_velocity(double heading_offset_rad) {
    return Eigen::Vector2d(std::sin(heading_offset_rad), std::cos(heading_offset_rad));
}

namespace {

double angle_between(const Eigen::Vector2d& v_in, const Eigen::Vector2d& d) {
    constexpr double kEps = 1e-9;
    const double denom = std::max(v_in.norm() * d.norm(), kEps);
    double cos_theta = v_in.dot(d) / denom;
    cos_theta = std::clamp(cos_theta, -1.0, 1.0);
    return std::acos(cos_theta);
}

}  // namespace

Eigen::MatrixXd build_cost_matrix(const AssignmentInput& input) {
    const int n = static_cast<int>(input.P.rows());
    Eigen::MatrixXd cost(n, n);

#pragma omp parallel for
    for (int i = 0; i < n; ++i) {
        const Eigen::Vector2d v_in = input.v_in_xy.row(i).transpose();
        for (int j = 0; j < n; ++j) {
            const Eigen::Vector3d diff = input.Q.row(j) - input.P.row(i);
            const double dist = diff.norm();
            const double climb_penalty = std::max(0.0, input.Q(j, 2) - input.P(i, 2));
            const Eigen::Vector2d d_ij(diff.x(), diff.y());
            const double dtheta = angle_between(v_in, d_ij);
            cost(i, j) = input.w_distance * dist + input.w_vertical_climb * climb_penalty +
                         input.w_heading_change * dtheta;
        }
    }
    return cost;
}

AssignmentResult solve_auction(const Eigen::MatrixXd& cost_matrix) {
    const int n = static_cast<int>(cost_matrix.rows());
    AssignmentResult result;
    result.assignment.assign(n, -1);
    if (n == 0) {
        return result;
    }

    // Auction maximizes benefit; the assignment problem here is a min-cost
    // one, so benefit is simply the negated cost.
    const Eigen::MatrixXd benefit = -cost_matrix;

    Eigen::VectorXd price = Eigen::VectorXd::Zero(n);
    std::vector<int> assignment(n, -1);
    std::vector<int> owner(n, -1);

    const double benefit_range = benefit.maxCoeff() - benefit.minCoeff();
    double epsilon = std::max(benefit_range / 4.0, 1e-6);
    const double scaling_factor = 0.25;
    const double epsilon_final = 1.0 / static_cast<double>(n + 1);

    while (true) {
        std::fill(assignment.begin(), assignment.end(), -1);
        std::fill(owner.begin(), owner.end(), -1);

        std::vector<int> unassigned(n);
        for (int i = 0; i < n; ++i) unassigned[i] = i;

        while (!unassigned.empty()) {
            const int num_bidders = static_cast<int>(unassigned.size());
            std::vector<int> bid_object(num_bidders);
            std::vector<double> bid_value(num_bidders);

#pragma omp parallel for
            for (int k = 0; k < num_bidders; ++k) {
                const int i = unassigned[k];
                double best_value = -std::numeric_limits<double>::infinity();
                double second_value = -std::numeric_limits<double>::infinity();
                int best_j = 0;
                for (int j = 0; j < n; ++j) {
                    const double value = benefit(i, j) - price(j);
                    if (value > best_value) {
                        second_value = best_value;
                        best_value = value;
                        best_j = j;
                    } else if (value > second_value) {
                        second_value = value;
                    }
                }
                if (!std::isfinite(second_value)) {
                    second_value = best_value;
                }
                bid_object[k] = best_j;
                bid_value[k] = price(best_j) + (best_value - second_value) + epsilon;
            }

            // Sequential contention resolution: only the highest bidder for
            // each contested object wins it this round.
            std::unordered_map<int, std::pair<int, double>> best_bid_per_object;
            best_bid_per_object.reserve(num_bidders);
            for (int k = 0; k < num_bidders; ++k) {
                const int j = bid_object[k];
                const double v = bid_value[k];
                auto it = best_bid_per_object.find(j);
                if (it == best_bid_per_object.end() || v > it->second.second) {
                    best_bid_per_object[j] = {unassigned[k], v};
                }
            }

            std::vector<int> next_unassigned;
            next_unassigned.reserve(unassigned.size());
            std::vector<bool> resolved(num_bidders, false);
            for (const auto& [j, bid] : best_bid_per_object) {
                const auto [i, v] = bid;
                if (owner[j] != -1) {
                    assignment[owner[j]] = -1;
                    next_unassigned.push_back(owner[j]);
                }
                owner[j] = i;
                assignment[i] = j;
                price(j) = v;
            }
            for (int k = 0; k < num_bidders; ++k) {
                if (assignment[unassigned[k]] == -1 &&
                    std::find(next_unassigned.begin(), next_unassigned.end(), unassigned[k]) ==
                        next_unassigned.end()) {
                    next_unassigned.push_back(unassigned[k]);
                }
            }
            unassigned = std::move(next_unassigned);
        }

        if (epsilon <= epsilon_final) {
            break;
        }
        epsilon = std::max(epsilon * scaling_factor, epsilon_final);
    }

    result.assignment = assignment;
    result.total_cost = 0.0;
    for (int i = 0; i < n; ++i) {
        result.total_cost += cost_matrix(i, assignment[i]);
    }
    return result;
}

}  // namespace drone_core::assignment
