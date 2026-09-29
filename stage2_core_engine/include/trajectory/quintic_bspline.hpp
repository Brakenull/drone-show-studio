#pragma once

#include <Eigen/Dense>

// Quintic B-spline Parametrization (docs/2-phase_2.md section 3.2): every
// drone transition is a clamped degree-5 B-spline p_i(t) = sum C_k B_k5(t),
// C^4-continuous, whose control points are the SCP decision variables.

namespace drone_core::trajectory {

inline constexpr int kDegree = 5;

// Open/clamped knot vector for `num_control_points` (>= degree+1) control
// points over [0, duration], with (degree+1) coincident knots at each end
// and uniformly spaced interior knots. Size = num_control_points + degree + 1.
Eigen::VectorXd clamped_knot_vector(int num_control_points, int degree, double duration);

// One step of the standard B-spline derivative-control-point recursion
// (Piegl & Tiller, "The NURBS Book", eq. 3.29):
//   Q_i = degree * (C_{i+1} - C_i) / (U[i+degree+1] - U[i+1])
// `control_points` has degree+1 knot multiplicity at both ends of `knots`;
// the returned curve has one fewer control point, degree-1, and a knot
// vector equal to `knots` with its first and last entries dropped.
struct DerivativeCurve {
    Eigen::MatrixXd control_points;  // (n-1) x 3
    Eigen::VectorXd knots;
    int degree = 0;
};
DerivativeCurve differentiate(const Eigen::MatrixXd& control_points, const Eigen::VectorXd& knots, int degree);

// Cox-de Boor span search + local basis function evaluation (NURBS Book
// algorithms A2.1/A2.2), used to evaluate a control-point curve at time t.
int find_span(int num_control_points, int degree, double t, const Eigen::VectorXd& knots);
Eigen::VectorXd basis_funs(int span, double t, int degree, const Eigen::VectorXd& knots);
Eigen::Vector3d evaluate_curve(const Eigen::MatrixXd& control_points, const Eigen::VectorXd& knots, int degree,
                                double t);

struct BoundaryConditions {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
};

// A drone at rest on the same point at both ends of a transition (a parked
// drone that a formation smaller than the fleet leaves in the holding area):
// the planners hold it there as a fixed obstacle instead of optimizing it
// (docs/2-phase_2.md section 1.15). seed_control_points() of such a pair is
// already the constant path.
inline bool is_stationary_hold(const BoundaryConditions& start, const BoundaryConditions& end) {
    constexpr double kTolM = 1e-6;
    return (end.position - start.position).norm() < kTolM && start.velocity.norm() < kTolM &&
           end.velocity.norm() < kTolM && start.acceleration.norm() < kTolM && end.acceleration.norm() < kTolM;
}

// Solves for the first/last 3 control points (C0,C1,C2 and C_{m-2},C_{m-1},
// C_m) that make a clamped quintic B-spline satisfy `start`/`end` exactly,
// via forward substitution on the derivative recursion above (the end is
// solved by applying the same forward substitution to the time-reversed
// curve). Any interior control points (when num_control_points > 6) are
// seeded by linear interpolation between C2 and C_{m-2} — they are free
// variables left for the SCP optimizer (scp_solver.hpp) to move.
Eigen::MatrixXd seed_control_points(const BoundaryConditions& start, const BoundaryConditions& end, double duration,
                                     int num_control_points);

// A single drone's clamped quintic B-spline transition, with its velocity/
// acceleration/jerk/snap derivative curves precomputed at construction.
class QuinticBSpline {
public:
    QuinticBSpline(Eigen::MatrixXd control_points, double duration);

    Eigen::Vector3d position(double t) const;
    Eigen::Vector3d velocity(double t) const;
    Eigen::Vector3d acceleration(double t) const;
    Eigen::Vector3d jerk(double t) const;
    Eigen::Vector3d snap(double t) const;

    const Eigen::MatrixXd& control_points() const { return control_points_; }
    const Eigen::VectorXd& knots() const { return knots_; }
    double duration() const { return duration_; }

private:
    Eigen::MatrixXd control_points_;
    Eigen::VectorXd knots_;
    double duration_;

    DerivativeCurve velocity_curve_;
    DerivativeCurve acceleration_curve_;
    DerivativeCurve jerk_curve_;
    DerivativeCurve snap_curve_;
};

}  // namespace drone_core::trajectory
