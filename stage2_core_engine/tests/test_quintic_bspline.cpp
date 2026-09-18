// Smoke test (no external test framework, per the effort/dependency budget
// noted to the user): checks that seed_control_points() reproduces the
// requested boundary conditions exactly, for both the minimal 6-point spline
// and a spline with free interior points.

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "trajectory/quintic_bspline.hpp"

using drone_core::trajectory::BoundaryConditions;
using drone_core::trajectory::QuinticBSpline;
using drone_core::trajectory::seed_control_points;

namespace {

bool close(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double tol = 1e-6) {
    return (a - b).norm() < tol;
}

bool check_boundary(int num_control_points) {
    BoundaryConditions start;
    start.position = Eigen::Vector3d(0.0, 0.0, 0.0);
    start.velocity = Eigen::Vector3d(1.0, 0.5, 0.0);
    start.acceleration = Eigen::Vector3d(0.0, 0.0, 0.0);

    BoundaryConditions end;
    end.position = Eigen::Vector3d(10.0, -5.0, 3.0);
    end.velocity = Eigen::Vector3d(0.5, 0.0, 0.2);
    end.acceleration = Eigen::Vector3d(0.0, 0.0, 0.0);

    const double duration = 8.0;
    const Eigen::MatrixXd cp = seed_control_points(start, end, duration, num_control_points);
    const QuinticBSpline spline(cp, duration);

    bool ok = true;
    ok &= close(spline.position(0.0), start.position);
    ok &= close(spline.velocity(0.0), start.velocity);
    ok &= close(spline.acceleration(0.0), start.acceleration);
    ok &= close(spline.position(duration), end.position);
    ok &= close(spline.velocity(duration), end.velocity, 1e-4);
    ok &= close(spline.acceleration(duration), end.acceleration, 1e-4);

    if (!ok) {
        std::fprintf(stderr, "boundary check failed for num_control_points=%d\n", num_control_points);
    }
    return ok;
}

}  // namespace

int main() {
    bool ok = true;
    ok &= check_boundary(6);
    ok &= check_boundary(10);
    return ok ? 0 : 1;
}
