#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

#include "trajectory/quintic_bspline.hpp"

// Kinematic Limits (docs/2-phase_2.md Rev 2.3 section 1.2/3.2): velocity,
// acceleration and jerk are linear combinations of the position control
// points, so v(t) <= v_max etc. reduce to time-independent linear
// inequalities on the control points via the convex-hull property (the
// derivative curve's value at any t lies in the convex hull of its own
// control points, so bounding every derivative control point bounds the
// curve everywhere).
//
// OSQP only supports linear constraints l <= Ax <= u (no SOCP norm cones),
// so the true Euclidean-norm bound ||v||_2 <= v_max is approximated by its
// L_inf *inscribed* box: |v_x|, |v_y|, |v_z| <= v_max/sqrt(3), which
// guarantees ||v||_2 <= v_max at every corner of the box (the doc's other
// listed option, the *circumscribed* box |v_axis| <= v_max, does not).

namespace drone_core::trajectory {

inline double inscribed_axis_limit(double euclidean_limit) { return euclidean_limit / std::sqrt(3.0); }

// T_min lower bound (docs/2-phase_2.md Rev 2.5 section 1.6/3.2): the
// theoretical peak velocity/acceleration/jerk of a rest-to-rest quintic
// move of distance d_max over duration T are v_peak=1.875*d/T,
// a_peak=5.77*d/T^2, j_peak=60*d/T^3 respectively; solving each for the T
// that makes peak == axis limit gives a per-derivative minimum duration.
// The overall floor is the largest of the three, inflated by
// `slack_fraction` extra headroom reserved for collision-avoidance bending
// (the whole point of this function: prevent a too-short animator-supplied
// duration from making the SCP problem kinematically infeasible before any
// collision constraint is even considered).
inline double compute_min_transition_time(double d_max, double v_limit_axis, double a_limit_axis,
                                           double j_limit_axis, double slack_fraction) {
    if (d_max <= 0.0) {
        return 0.0;
    }
    const double t_v = 1.875 * d_max / v_limit_axis;
    const double t_a = std::sqrt(5.77 * d_max / a_limit_axis);
    const double t_j = std::cbrt(60.0 * d_max / j_limit_axis);
    return std::max({t_v, t_a, t_j}) * (1.0 + slack_fraction);
}

// D such that (D * C_axis) gives the next-lower-degree derivative's control
// points for a single axis, where C_axis is the num_control_points-length
// column vector of that axis's position control points.
inline Eigen::MatrixXd single_step_derivative_matrix(const Eigen::VectorXd& knots, int degree,
                                                      int num_control_points) {
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(num_control_points - 1, num_control_points);
    for (int i = 0; i < num_control_points - 1; ++i) {
        const double denom = knots(i + degree + 1) - knots(i + 1);
        const double coeff = denom < 1e-12 ? 0.0 : degree / denom;
        D(i, i) = -coeff;
        D(i, i + 1) = coeff;
    }
    return D;
}

struct DerivativeOperators {
    Eigen::MatrixXd velocity;      // (n-1) x n, maps position CPs -> velocity CPs (degree 4)
    Eigen::MatrixXd acceleration;  // (n-2) x n (degree 3)
    Eigen::MatrixXd jerk;          // (n-3) x n (degree 2)
    Eigen::MatrixXd snap;          // (n-4) x n (degree 1)

    // Each derivative curve's own clamped knot vector, needed to evaluate it
    // or to integrate its squared basis functions (scp_solver.cpp's snap/jerk
    // minimization Gram matrices).
    Eigen::VectorXd velocity_knots;
    Eigen::VectorXd acceleration_knots;
    Eigen::VectorXd jerk_knots;
    Eigen::VectorXd snap_knots;
};

inline DerivativeOperators build_derivative_operators(int num_control_points, double duration) {
    const int d = kDegree;
    const Eigen::VectorXd U = clamped_knot_vector(num_control_points, d, duration);
    const Eigen::MatrixXd D1 = single_step_derivative_matrix(U, d, num_control_points);

    const Eigen::VectorXd U1 = U.segment(1, U.size() - 2);
    const Eigen::MatrixXd D2 = single_step_derivative_matrix(U1, d - 1, num_control_points - 1);

    const Eigen::VectorXd U2 = U1.segment(1, U1.size() - 2);
    const Eigen::MatrixXd D3 = single_step_derivative_matrix(U2, d - 2, num_control_points - 2);

    const Eigen::VectorXd U3 = U2.segment(1, U2.size() - 2);
    const Eigen::MatrixXd D4 = single_step_derivative_matrix(U3, d - 3, num_control_points - 3);

    DerivativeOperators ops;
    ops.velocity = D1;
    ops.acceleration = D2 * D1;
    ops.jerk = D3 * D2 * D1;
    ops.snap = D4 * D3 * D2 * D1;
    ops.velocity_knots = U1;
    ops.acceleration_knots = U2;
    ops.jerk_knots = U3;
    ops.snap_knots = U3.segment(1, U3.size() - 2);
    return ops;
}

// Gram matrix G (num_control_points x num_control_points) of a clamped
// B-spline basis of the given degree: G(a,b) = integral of B_a(t)*B_b(t) dt
// over the curve's domain, evaluated span-by-span with 5-point Gauss-Legendre
// quadrature (exact for polynomials up to degree 9, comfortably covering the
// degree-4 jerk^2 and degree-2 snap^2 integrands used by scp_solver.cpp).
inline Eigen::MatrixXd integrate_basis_gram(const Eigen::VectorXd& knots, int degree, int num_control_points) {
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(num_control_points, num_control_points);
    static const double kGaussX[5] = {-0.9061798459386640, -0.5384693101056831, 0.0, 0.5384693101056831,
                                       0.9061798459386640};
    static const double kGaussW[5] = {0.2369268850561891, 0.4786286704993665, 0.5688888888888889,
                                       0.4786286704993665, 0.2369268850561891};

    for (int span = degree; span < num_control_points; ++span) {
        const double a = knots(span);
        const double b = knots(span + 1);
        if (b - a < 1e-12) continue;
        for (int q = 0; q < 5; ++q) {
            const double t = 0.5 * (b - a) * kGaussX[q] + 0.5 * (b + a);
            const double w = 0.5 * (b - a) * kGaussW[q];
            const Eigen::VectorXd N = basis_funs(span, t, degree, knots);
            for (int r = 0; r <= degree; ++r) {
                for (int c = 0; c <= degree; ++c) {
                    G(span - degree + r, span - degree + c) += w * N(r) * N(c);
                }
            }
        }
    }
    return G;
}

}  // namespace drone_core::trajectory
