#include "trajectory/quintic_bspline.hpp"

#include <algorithm>
#include <cassert>
#include <vector>

namespace drone_core::trajectory {

Eigen::VectorXd clamped_knot_vector(int num_control_points, int degree, double duration) {
    const int num_knots = num_control_points + degree + 1;
    Eigen::VectorXd U(num_knots);

    for (int i = 0; i <= degree; ++i) {
        U(i) = 0.0;
    }
    const int num_interior = num_control_points - degree - 1;
    for (int i = 1; i <= num_interior; ++i) {
        U(degree + i) = duration * static_cast<double>(i) / static_cast<double>(num_interior + 1);
    }
    for (int i = 0; i <= degree; ++i) {
        U(num_knots - 1 - i) = duration;
    }
    return U;
}

DerivativeCurve differentiate(const Eigen::MatrixXd& control_points, const Eigen::VectorXd& knots, int degree) {
    const int n = static_cast<int>(control_points.rows());
    Eigen::MatrixXd Q(n - 1, control_points.cols());
    for (int i = 0; i < n - 1; ++i) {
        const double denom = knots(i + degree + 1) - knots(i + 1);
        if (denom < 1e-12) {
            Q.row(i).setZero();
        } else {
            Q.row(i) = degree * (control_points.row(i + 1) - control_points.row(i)) / denom;
        }
    }
    Eigen::VectorXd new_knots = knots.segment(1, knots.size() - 2);
    return DerivativeCurve{Q, new_knots, degree - 1};
}

int find_span(int num_control_points, int degree, double t, const Eigen::VectorXd& knots) {
    const int n = num_control_points - 1;
    if (t >= knots(n + 1)) return n;
    if (t <= knots(degree)) return degree;
    int low = degree;
    int high = n + 1;
    int mid = (low + high) / 2;
    while (t < knots(mid) || t >= knots(mid + 1)) {
        if (t < knots(mid)) {
            high = mid;
        } else {
            low = mid;
        }
        mid = (low + high) / 2;
    }
    return mid;
}

Eigen::VectorXd basis_funs(int span, double t, int degree, const Eigen::VectorXd& knots) {
    Eigen::VectorXd N(degree + 1);
    N(0) = 1.0;
    std::vector<double> left(degree + 1, 0.0);
    std::vector<double> right(degree + 1, 0.0);

    for (int j = 1; j <= degree; ++j) {
        left[j] = t - knots(span + 1 - j);
        right[j] = knots(span + j) - t;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            const double temp = N(r) / (right[r + 1] + left[j - r]);
            N(r) = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        N(j) = saved;
    }
    return N;
}

Eigen::Vector3d evaluate_curve(const Eigen::MatrixXd& control_points, const Eigen::VectorXd& knots, int degree,
                                 double t) {
    const int num_control_points = static_cast<int>(control_points.rows());
    if (num_control_points == 0) {
        return Eigen::Vector3d::Zero();
    }
    t = std::clamp(t, knots(degree), knots(num_control_points));
    const int span = find_span(num_control_points, degree, t, knots);
    const Eigen::VectorXd N = basis_funs(span, t, degree, knots);
    Eigen::RowVector3d result = Eigen::RowVector3d::Zero();
    for (int j = 0; j <= degree; ++j) {
        result += N(j) * control_points.row(span - degree + j);
    }
    return result.transpose();
}

namespace {

// Forward-substitutes the standard clamped B-spline derivative recursion to
// find the first 3 control points that reproduce (position, velocity,
// acceleration) at the knot vector's start (t = U(degree)).
Eigen::MatrixXd forward_three_control_points(const BoundaryConditions& bc, const Eigen::VectorXd& U, int degree) {
    Eigen::MatrixXd out(3, 3);
    out.row(0) = bc.position.transpose();

    const double tau1 = U(degree + 1) - U(1);
    const Eigen::RowVector3d q1_0 = bc.velocity.transpose();
    out.row(1) = out.row(0) + q1_0 * (tau1 / degree);

    const double tau2 = U(degree + 1) - U(2);
    const Eigen::RowVector3d q1_1 = q1_0 + bc.acceleration.transpose() * (tau2 / (degree - 1));

    const double tau3 = U(degree + 2) - U(2);
    out.row(2) = out.row(1) + q1_1 * (tau3 / degree);

    return out;
}

}  // namespace

Eigen::MatrixXd seed_control_points(const BoundaryConditions& start, const BoundaryConditions& end, double duration,
                                     int num_control_points) {
    assert(num_control_points >= kDegree + 1);
    const int d = kDegree;
    const Eigen::VectorXd U = clamped_knot_vector(num_control_points, d, duration);
    const int num_knots = num_control_points + d + 1;

    Eigen::MatrixXd C(num_control_points, 3);

    const Eigen::MatrixXd front = forward_three_control_points(start, U, d);
    C.row(0) = front.row(0);
    C.row(1) = front.row(1);
    C.row(2) = front.row(2);

    // Solve the end boundary by forward-substituting the time-reversed
    // curve q(s) = p(duration - s): q'(0) = -p'(duration), q''(0) = p''(duration).
    Eigen::VectorXd U_rev(num_knots);
    for (int i = 0; i < num_knots; ++i) {
        U_rev(i) = duration - U(num_knots - 1 - i);
    }
    BoundaryConditions end_reversed;
    end_reversed.position = end.position;
    end_reversed.velocity = -end.velocity;
    end_reversed.acceleration = end.acceleration;
    const Eigen::MatrixXd back = forward_three_control_points(end_reversed, U_rev, d);

    const int m = num_control_points - 1;
    C.row(m) = back.row(0);
    C.row(m - 1) = back.row(1);
    C.row(m - 2) = back.row(2);

    for (int i = 3; i <= m - 3; ++i) {
        const double s = static_cast<double>(i - 2) / static_cast<double>(m - 4);
        C.row(i) = (1.0 - s) * C.row(2) + s * C.row(m - 2);
    }

    return C;
}

QuinticBSpline::QuinticBSpline(Eigen::MatrixXd control_points, double duration)
    : control_points_(std::move(control_points)),
      knots_(clamped_knot_vector(static_cast<int>(control_points_.rows()), kDegree, duration)),
      duration_(duration),
      velocity_curve_(differentiate(control_points_, knots_, kDegree)),
      acceleration_curve_(differentiate(velocity_curve_.control_points, velocity_curve_.knots, kDegree - 1)),
      jerk_curve_(differentiate(acceleration_curve_.control_points, acceleration_curve_.knots, kDegree - 2)),
      snap_curve_(differentiate(jerk_curve_.control_points, jerk_curve_.knots, kDegree - 3)) {}

Eigen::Vector3d QuinticBSpline::position(double t) const {
    return evaluate_curve(control_points_, knots_, kDegree, t);
}

Eigen::Vector3d QuinticBSpline::velocity(double t) const {
    return evaluate_curve(velocity_curve_.control_points, velocity_curve_.knots, velocity_curve_.degree, t);
}

Eigen::Vector3d QuinticBSpline::acceleration(double t) const {
    return evaluate_curve(acceleration_curve_.control_points, acceleration_curve_.knots, acceleration_curve_.degree,
                           t);
}

Eigen::Vector3d QuinticBSpline::jerk(double t) const {
    return evaluate_curve(jerk_curve_.control_points, jerk_curve_.knots, jerk_curve_.degree, t);
}

Eigen::Vector3d QuinticBSpline::snap(double t) const {
    return evaluate_curve(snap_curve_.control_points, snap_curve_.knots, snap_curve_.degree, t);
}

}  // namespace drone_core::trajectory
