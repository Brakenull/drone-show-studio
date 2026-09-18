// Smoke test: a 3-drone assignment where the least-cost matching is obvious
// (each drone's nearest target, no ties) should be recovered exactly.

#include <cstdio>

#include "assignment/lap_auction.hpp"

using drone_core::assignment::AssignmentInput;
using drone_core::assignment::build_cost_matrix;
using drone_core::assignment::solve_auction;

int main() {
    AssignmentInput input;
    input.P.resize(3, 3);
    input.P << 0, 0, 0, 10, 0, 0, 20, 0, 0;

    input.Q.resize(3, 3);
    input.Q << 0, 0, 1, 10, 0, 1, 20, 0, 1;  // identity is the obvious optimum

    input.v_in_xy.resize(3, 2);
    input.v_in_xy << 0, 1, 0, 1, 0, 1;

    input.w_distance = 1.0;
    input.w_vertical_climb = 2.5;
    input.w_heading_change = 0.5;

    const Eigen::MatrixXd cost = build_cost_matrix(input);
    const auto result = solve_auction(cost);

    bool ok = true;
    for (int i = 0; i < 3; ++i) {
        if (result.assignment[i] != i) {
            std::fprintf(stderr, "expected identity assignment, got assignment[%d]=%d\n", i, result.assignment[i]);
            ok = false;
        }
    }
    return ok ? 0 : 1;
}
