#include "optimizer/scp_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Sparse>
#include <osqp/osqp.h>

#include "collision/spatio_temporal_hash.hpp"
#include "trajectory/apf_seeder.hpp"
#include "trajectory/kinematic_limits.hpp"

namespace drone_core::optimizer {

namespace {

using trajectory::DerivativeOperators;
using trajectory::QuinticBSpline;

// Progress-event timing (measurement only, see ProgressEvent::step_sec).
using Clock = std::chrono::steady_clock;
double seconds_since(Clock::time_point start) { return std::chrono::duration<double>(Clock::now() - start).count(); }

// Adaptive 4D voxel time bucket:
// Delta T_bucket = max(0.5s, T/target_time_windows). A fixed 0.5s bucket
// (the original choice) is fine for an 8s transition (16 windows) but
// explodes to ~100 windows once T_min auto-scaling stretches a long real
// transition to 40-50s — each extra window means another broad-phase pass
// and another round of collision rows per SCP iteration, which is what took
// a real 300-drone/50s transition from a projected 15s budget to 6+ minutes
// of wall time before this fix.
// Altitude floor planning buffer: the QP keeps free control
// points this far above the floor, while the gatekeeper still verifies the
// floor itself (1 mm tolerance). Same idea as collision_margin_fraction for
// separation. The floor rows are hard, but try_solve_qp() accepts OSQP's
// SOLVED_INACCURATE and OSQP's tolerances are relative to coordinates of tens
// of meters, so a "hard" z >= floor came back 1-6 cm underground on real
// ground-level takeoffs (2026-09-29: 100_cone, cube_300), rejecting attempts
// whose separation had passed.
constexpr double kFloorPlanningMarginM = 0.05;

double compute_time_bucket_s(double duration, bool adaptive, int target_windows) {
    constexpr double kMinBucketS = 0.5;
    if (!adaptive || target_windows <= 0) {
        return kMinBucketS;
    }
    return std::max(kMinBucketS, duration / static_cast<double>(target_windows));
}

// Which control-point indices are QP decision variables vs. pinned boundary
// values, for a transition using `num_control_points` control points.
struct FreeIndexMap {
    int num_control_points = 0;
    int free_begin = 3;
    int free_end = 0;  // exclusive

    int num_free() const { return std::max(0, free_end - free_begin); }
    bool is_free(int idx) const { return idx >= free_begin && idx < free_end; }

    explicit FreeIndexMap(int n) : num_control_points(n), free_end(n - 3) {}
};

// Splits a full-length (size == num_control_points) linear-map row into the
// part acting on free control points and the constant contribution from the
// pinned boundary control points (per axis, since the same scalar
// coefficients apply identically to x, y and z).
struct SplitRow {
    Eigen::RowVectorXd free_coeffs;         // length num_free
    Eigen::RowVector3d fixed_contribution;  // constant term from pinned CPs
};

SplitRow split_row(const Eigen::VectorXd& coeffs, const FreeIndexMap& map, const Eigen::MatrixXd& control_points) {
    SplitRow s;
    s.free_coeffs = coeffs.segment(map.free_begin, map.num_free()).transpose();
    s.fixed_contribution.setZero();
    for (int idx = 0; idx < map.num_control_points; ++idx) {
        if (!map.is_free(idx)) {
            s.fixed_contribution += coeffs(idx) * control_points.row(idx);
        }
    }
    return s;
}

Eigen::VectorXd dense_basis_row(int num_control_points, int degree, const Eigen::VectorXd& knots, double t) {
    Eigen::VectorXd row = Eigen::VectorXd::Zero(num_control_points);
    const double clamped_t = std::clamp(t, knots(degree), knots(num_control_points));
    const int span = trajectory::find_span(num_control_points, degree, clamped_t, knots);
    const Eigen::VectorXd basis = trajectory::basis_funs(span, clamped_t, degree, knots);
    for (int j = 0; j <= degree; ++j) {
        row(span - degree + j) = basis(j);
    }
    return row;
}

// Per-drone, precomputed once per transition (duration/num_control_points is
// shared by every drone in a transition).
struct DroneWorkspace {
    int drone_id = 0;
    bool keep_out = false;  // this drone flies the show here
    bool fixed = false;     // parked, held as an obstacle and never solved
    std::optional<double> floor_m;  // this drone's own floor, see drone_floor()
    Eigen::MatrixXd control_points;  // current iterate, num_control_points x 3
    DerivativeOperators ops;
    std::unique_ptr<QuinticBSpline> spline;  // rebuilt after every drone-level solve
};

// The altitude floor a drone keeps: the config's, or its own
// when that is higher. None without a config floor.
std::optional<double> drone_floor(std::optional<double> own, const CoreConfig& config) {
    if (!config.safety.altitude_floor_m) return std::nullopt;
    return own ? std::max(*config.safety.altitude_floor_m, *own) : config.safety.altitude_floor_m;
}

void rebuild_spline(DroneWorkspace& ws, double duration) {
    ws.spline = std::make_unique<QuinticBSpline>(ws.control_points, duration);
}

struct LinearRow {
    Eigen::RowVectorXd coeffs;  // length 3*num_free (x-block, y-block, z-block)
    double lower = -OSQP_INFTY;
    double upper = OSQP_INFTY;
};

// One-sided linearized separation constraint
// contributed to `self`'s QP, treating `other`'s trajectory as fixed at
// whatever value it currently holds (in Gauss-Seidel order, that may already
// be this iteration's updated solution for cluster-mates solved earlier in
// the same sweep).
//
// Regardless of which one of the pair is "self", requiring
// (p_i^(k)-p_j^(k))^T (p_i(t)-p_j(t)) >= d_enforce*||p_i^(k)-p_j^(k)|| with
// the *other* side fixed at other_pos_k reduces, after substituting
// self/other for i/j in either role and simplifying, to the same formula in
// terms of self's own reference point: letting normal = self_pos_k -
// other_pos_k (the direction from other to self right now),
//   normal^T p_self(t) >= d_enforce*||normal|| + normal^T other_pos_k
// i.e. "move — or stay — further out along the direction away from where
// the other drone currently is."
LinearRow build_collision_row(const FreeIndexMap& map, const DroneWorkspace& self_ws, const Eigen::Vector3d& self_pos_k,
                               const Eigen::Vector3d& other_pos_k, double enforced_distance, double t) {
    Eigen::Vector3d normal = self_pos_k - other_pos_k;
    double dist = normal.norm();
    if (dist < 1e-6) {
        // Singular case: inject a deterministic orthogonal separation vector
        // instead of a random one, so results stay reproducible.
        normal = Eigen::Vector3d(1.0, 0.0, 0.0);
        dist = 1.0;
    }
    // n^T(p_i-p_j) >= d_enforce*||n|| is invariant to rescaling n (both sides
    // scale identically), so using the unit normal here (n_hat = n/dist) is
    // the same half-space as the unnormalized n_ij, just far
    // better conditioned: at real show coordinate scales (positions ~O(10 m)
    // apart), the raw n_ij made this row's coefficients and bound ~10-100x
    // larger than the kinematic/trust-region rows it's solved alongside.
    // Dividing through by dist turns the RHS from d_enforce*dist into plain
    // d_enforce — using n_hat but leaving the RHS as d_enforce*dist (an
    // earlier version of this function did exactly that) is a real bug: it
    // makes the *required* separation grow with the drones' current distance
    // instead of staying pinned at d_enforce, which manufactures a runaway,
    // unsatisfiable bound for any pair that isn't already very close —
    // exactly the persistent PRIMAL_INFEASIBLE this revision is meant to
    // eliminate.
    normal /= dist;

    const Eigen::VectorXd basis_row =
        dense_basis_row(map.num_control_points, trajectory::kDegree, self_ws.spline->knots(), t);
    const SplitRow split = split_row(basis_row, map, self_ws.control_points);

    LinearRow row;
    row.coeffs = Eigen::RowVectorXd::Zero(3 * map.num_free());
    row.coeffs.segment(0, map.num_free()) = normal.x() * split.free_coeffs;
    row.coeffs.segment(map.num_free(), map.num_free()) = normal.y() * split.free_coeffs;
    row.coeffs.segment(2 * map.num_free(), map.num_free()) = normal.z() * split.free_coeffs;

    row.lower = enforced_distance + normal.dot(other_pos_k) - normal.dot(split.fixed_contribution);
    row.upper = OSQP_INFTY;
    return row;
}

LinearRow build_box_row(const FreeIndexMap& map, const Eigen::MatrixXd& control_points, const Eigen::VectorXd& d_row,
                         int axis, double limit) {
    const SplitRow split = split_row(d_row, map, control_points);
    LinearRow row;
    row.coeffs = Eigen::RowVectorXd::Zero(3 * map.num_free());
    row.coeffs.segment(axis * map.num_free(), map.num_free()) = split.free_coeffs;
    row.lower = -limit - split.fixed_contribution(axis);
    row.upper = limit - split.fixed_contribution(axis);
    return row;
}

// Trust Region step-limit row: a single decision variable
// pinned to [anchor - delta, anchor + delta], where `anchor` is that
// variable's value at the *start* of this SCP iteration (not the possibly
// already-Gauss-Seidel-updated `ws.control_points`).
LinearRow build_trust_region_row(int flat_index, int dim, double anchor, double delta) {
    LinearRow row;
    row.coeffs = Eigen::RowVectorXd::Zero(dim);
    row.coeffs(flat_index) = 1.0;
    row.lower = anchor - delta;
    row.upper = anchor + delta;
    return row;
}

// ---- Holding-area keep-out zone ----------

Eigen::Vector3d zone_lo(const KeepOutZone& z) { return Eigen::Vector3d(z.lo[0], z.lo[1], z.lo[2]); }
Eigen::Vector3d zone_hi(const KeepOutZone& z) { return Eigen::Vector3d(z.hi[0], z.hi[1], z.hi[2]); }

// Going *under* the holding area is not an option when the altitude floor
// leaves no room for the clearance below it.
bool zone_blocks_underpass(const KeepOutZone& zone, const CoreConfig& config) {
    return config.safety.altitude_floor_m && zone.lo[2] - zone.clearance_m < *config.safety.altitude_floor_m;
}

// The half-space  normal^T x >= normal^T anchor + clearance  lies entirely
// outside the zone (region grown by the clearance) and touches it: for a
// point outside the region, `anchor` is its nearest region point and
// `normal` points from there to it (the region is convex, so the region
// lies behind that plane); for a point inside, the face it is least deep
// behind. A downward normal is replaced by the best lateral/upward one when
// the floor blocks the underpass.
struct KeepOutPlane {
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d anchor = Eigen::Vector3d::Zero();
};

KeepOutPlane keep_out_plane(const Eigen::Vector3d& p, const KeepOutZone& zone, const CoreConfig& config) {
    const Eigen::Vector3d lo = zone_lo(zone), hi = zone_hi(zone);
    const bool no_underpass = zone_blocks_underpass(zone, config);
    const Eigen::Vector3d nearest = p.cwiseMax(lo).cwiseMin(hi);
    Eigen::Vector3d offset = p - nearest;
    if (no_underpass && offset.z() < 0.0) offset.z() = 0.0;
    if (offset.norm() > 1e-6) {
        KeepOutPlane plane;
        plane.normal = offset.normalized();
        plane.anchor = nearest;
        return plane;
    }
    // Inside the region (or straight under it with no underpass): leave
    // through the face needing the least travel.
    KeepOutPlane best;
    double best_travel = std::numeric_limits<double>::infinity();
    for (int axis = 0; axis < 3; ++axis) {
        for (int side = 0; side < 2; ++side) {
            if (axis == 2 && side == 0 && no_underpass) continue;  // the bottom face
            const double face = side == 0 ? lo(axis) : hi(axis);
            const double travel = side == 0 ? p(axis) - face : face - p(axis);
            if (travel < best_travel) {
                best_travel = travel;
                best.normal = Eigen::Vector3d::Zero();
                best.normal(axis) = side == 0 ? -1.0 : 1.0;
                best.anchor = p;
                best.anchor(axis) = face;
            }
        }
    }
    return best;
}

// Sampled keep-out rows for one show drone, like the collision rows (soft,
// verified afterwards by the 100 Hz gatekeeper): at every 0.25 s sample
// where the current iterate is within the clearance plus an activation
// margin of the region, require the path to stay in keep_out_plane()'s
// half-space.
constexpr double kKeepOutSampleDtS = 0.25;
constexpr double kKeepOutActivationM = 3.0;

std::vector<LinearRow> collect_keep_out_rows(const FreeIndexMap& map, const DroneWorkspace& ws, double duration,
                                             const CoreConfig& config) {
    std::vector<LinearRow> rows;
    if (!config.safety.keep_out || !ws.keep_out || map.num_free() == 0) return rows;
    const KeepOutZone& zone = *config.safety.keep_out;
    const int samples = std::max(2, static_cast<int>(std::ceil(duration / kKeepOutSampleDtS)) + 1);
    for (int s = 0; s < samples; ++s) {
        const double t = duration * s / (samples - 1);
        const Eigen::Vector3d p = ws.spline->position(t);
        if (distance_to_keep_out_region(p, zone) >= zone.clearance_m + kKeepOutActivationM) continue;
        const KeepOutPlane plane = keep_out_plane(p, zone, config);
        const Eigen::VectorXd basis_row = dense_basis_row(map.num_control_points, trajectory::kDegree, ws.spline->knots(), t);
        const SplitRow split = split_row(basis_row, map, ws.control_points);
        LinearRow row;
        row.coeffs = Eigen::RowVectorXd::Zero(3 * map.num_free());
        for (int axis = 0; axis < 3; ++axis) {
            row.coeffs.segment(axis * map.num_free(), map.num_free()) = plane.normal(axis) * split.free_coeffs;
        }
        row.lower = zone.clearance_m + plane.normal.dot(plane.anchor) - plane.normal.dot(split.fixed_contribution);
        row.upper = OSQP_INFTY;
        if (row.coeffs.cwiseAbs().sum() > 1e-12) rows.push_back(std::move(row));
    }
    return rows;
}

// How far the show drones' current iterates reach into the zone (0 = clear),
// sampled like the rows above. Used to rank SCP iterates.
double keep_out_intrusion(const std::vector<DroneWorkspace>& workspaces, double duration, const CoreConfig& config) {
    if (!config.safety.keep_out) return 0.0;
    const KeepOutZone& zone = *config.safety.keep_out;
    const int samples = std::max(2, static_cast<int>(std::ceil(duration / kKeepOutSampleDtS)) + 1);
    double worst = 0.0;
    for (const auto& ws : workspaces) {
        if (!ws.keep_out) continue;
        for (int s = 0; s < samples; ++s) {
            const double d = distance_to_keep_out_region(ws.spline->position(duration * s / (samples - 1)), zone);
            worst = std::max(worst, zone.clearance_m - d);
        }
    }
    return worst;
}

// Assembles and solves the QP for one candidate row set, returning the
// updated free control points on OSQP_SOLVED/OSQP_SOLVED_INACCURATE, or
// std::nullopt otherwise (in particular OSQP_PRIMAL_INFEASIBLE — an
// over-constrained row set, e.g. too many simultaneous collision rows for
// the trust region to also fit within).
std::optional<Eigen::MatrixXd> try_solve_qp(const FreeIndexMap& map, const Eigen::MatrixXd& control_points,
                                             const Eigen::MatrixXd& P_dense, const Eigen::VectorXd& q,
                                             const std::vector<LinearRow>& rows) {
    const int num_free = map.num_free();
    // Dim is no longer always 3*num_free -- the soft-slack
    // collision formulation appends one extra decision
    // variable per collision row, so P_dense/q/every row here may be wider
    // than the position-only block. Only the first 3*num_free columns of the
    // solution (the positions) are read back out below.
    const int dim = static_cast<int>(P_dense.rows());
    const int m = static_cast<int>(rows.size());


    // Build sparse CSC for P (upper triangular only, as OSQP requires).
    std::vector<Eigen::Triplet<double>> p_triplets;
    for (int c = 0; c < dim; ++c) {
        for (int r = 0; r <= c; ++r) {
            const double v = P_dense(r, c);
            if (std::abs(v) > 1e-14) p_triplets.emplace_back(r, c, v);
        }
    }
    Eigen::SparseMatrix<double> P_sparse(dim, dim);
    P_sparse.setFromTriplets(p_triplets.begin(), p_triplets.end());
    P_sparse.makeCompressed();

    std::vector<Eigen::Triplet<double>> a_triplets;
    Eigen::VectorXd l(m), u(m);
    for (int r = 0; r < m; ++r) {
        for (int c = 0; c < dim; ++c) {
            const double v = rows[r].coeffs(c);
            if (std::abs(v) > 1e-14) a_triplets.emplace_back(r, c, v);
        }
        l(r) = rows[r].lower;
        u(r) = rows[r].upper;
    }
    Eigen::SparseMatrix<double> A_sparse(m, dim);
    A_sparse.setFromTriplets(a_triplets.begin(), a_triplets.end());
    A_sparse.makeCompressed();

    std::vector<OSQPFloat> p_x(P_sparse.valuePtr(), P_sparse.valuePtr() + P_sparse.nonZeros());
    std::vector<OSQPInt> p_i(P_sparse.innerIndexPtr(), P_sparse.innerIndexPtr() + P_sparse.nonZeros());
    std::vector<OSQPInt> p_p(P_sparse.outerIndexPtr(), P_sparse.outerIndexPtr() + P_sparse.outerSize() + 1);

    std::vector<OSQPFloat> a_x(A_sparse.valuePtr(), A_sparse.valuePtr() + A_sparse.nonZeros());
    std::vector<OSQPInt> a_i(A_sparse.innerIndexPtr(), A_sparse.innerIndexPtr() + A_sparse.nonZeros());
    std::vector<OSQPInt> a_p(A_sparse.outerIndexPtr(), A_sparse.outerIndexPtr() + A_sparse.outerSize() + 1);

    OSQPCscMatrix P_csc;
    OSQPCscMatrix_set_data(&P_csc, dim, dim, static_cast<OSQPInt>(p_x.size()), p_x.data(), p_i.data(), p_p.data());
    OSQPCscMatrix A_csc;
    OSQPCscMatrix_set_data(&A_csc, m, dim, static_cast<OSQPInt>(a_x.size()), a_x.data(), a_i.data(), a_p.data());

    std::vector<OSQPFloat> q_vec(q.data(), q.data() + q.size());
    std::vector<OSQPFloat> l_vec(l.data(), l.data() + l.size());
    std::vector<OSQPFloat> u_vec(u.data(), u.data() + u.size());

    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.warm_starting = 1;
    settings.polishing = 1;

    OSQPSolver* solver = nullptr;
    const OSQPInt exit_flag =
        osqp_setup(&solver, &P_csc, q_vec.data(), &A_csc, l_vec.data(), u_vec.data(), m, dim, &settings);

    std::optional<Eigen::MatrixXd> result;
    if (exit_flag == 0 && solver != nullptr) {
        osqp_solve(solver);
        if (solver->info->status_val == OSQP_SOLVED || solver->info->status_val == OSQP_SOLVED_INACCURATE) {
            Eigen::MatrixXd updated = control_points;
            for (int axis = 0; axis < 3; ++axis) {
                for (int i = 0; i < num_free; ++i) {
                    updated(map.free_begin + i, axis) = solver->solution->x[axis * num_free + i];
                }
            }
            result = std::move(updated);
        }
    }
    if (solver != nullptr) {
        osqp_cleanup(solver);
    }
    return result;
}

// Deterministic "orthogonal perturbation jitter" (infeasibility fallback,
// tier 2): nudges each free control point by
// +-jitter_magnitude_m perpendicular to the drone's own nominal direction of
// travel (end - start, read off the pinned boundary control points), sign
// alternating by (control-point index + drone_id) parity. A fixed,
// reproducible pattern is used instead of true randomness so a given show
// always optimizes to the same result.
void apply_orthogonal_jitter(const FreeIndexMap& map, DroneWorkspace& ws, double jitter_magnitude_m) {
    const int n = map.num_control_points;
    const Eigen::Vector3d d = ws.control_points.row(n - 1) - ws.control_points.row(0);
    const double horizontal_norm = std::hypot(d.x(), d.y());
    const Eigen::Vector3d n_ortho = horizontal_norm > 1e-3 ? Eigen::Vector3d(-d.y(), d.x(), 0.0) / horizontal_norm
                                                            : Eigen::Vector3d(1.0, 0.0, 0.0);
    for (int k = map.free_begin; k < map.free_end; ++k) {
        const double sign = ((k + ws.drone_id) % 2 == 0) ? 1.0 : -1.0;
        ws.control_points.row(k) += (sign * jitter_magnitude_m * n_ortho).transpose();
    }
}

// Widens a hard row (kinematic or trust-region, coeffs of length `dim`) to
// `total_dim` columns by zero-padding the slack block, leaving lower/upper
// untouched.
LinearRow pad_hard_row(const LinearRow& row, int total_dim) {
    LinearRow padded = row;
    padded.coeffs = Eigen::RowVectorXd::Zero(total_dim);
    padded.coeffs.segment(0, row.coeffs.size()) = row.coeffs;
    return padded;
}

// converts one hard collision row (coeffs of length
// `dim`, `normal^T p >= lower`) into its soft-slack form by appending a
// dedicated nonnegative slack column at `slack_col`:
// `normal^T p + s_k >= lower, s_k >= 0` — returns both the softened
// separation row and its slack lower-bound row.
std::pair<LinearRow, LinearRow> soften_collision_row(const LinearRow& row, int total_dim, int slack_col) {
    LinearRow soft = row;
    soft.coeffs = Eigen::RowVectorXd::Zero(total_dim);
    soft.coeffs.segment(0, row.coeffs.size()) = row.coeffs;
    soft.coeffs(slack_col) = 1.0;

    LinearRow bound;
    bound.coeffs = Eigen::RowVectorXd::Zero(total_dim);
    bound.coeffs(slack_col) = 1.0;
    bound.lower = 0.0;
    bound.upper = OSQP_INFTY;
    return {soft, bound};
}

// Assembles the full-width (position + one slack variable per collision row)
// P/q/row-set for one candidate solve attempt: `hard_rows` (kinematic and/or
// trust-region, width `dim`) stay hard; every row in `collision_rows` (also
// width `dim`, as produced by build_collision_row) is softened per
// soften_collision_row() above, with a quadratic w_slack_collision penalty on
// its slack variable added to the objective. This is what makes the
// collision rows' lower bound always satisfiable regardless of how contested
// the local geometry is: OSQP can no longer report PRIMAL_INFEASIBLE from a
// collision row itself, only from the still-hard kinematic-box/trust-region
// rows, which is what the tiered fallback below still exists to recover
// from.
struct SoftQp {
    Eigen::MatrixXd P;
    Eigen::VectorXd q;
    std::vector<LinearRow> rows;
};

SoftQp assemble_soft_qp(const Eigen::MatrixXd& P_position, const Eigen::VectorXd& q_position,
                         const std::vector<LinearRow>& hard_rows, const std::vector<LinearRow>& collision_rows,
                         double w_slack_collision) {
    const int dim = static_cast<int>(P_position.rows());
    const int k_slack = static_cast<int>(collision_rows.size());
    const int total_dim = dim + k_slack;

    SoftQp qp;
    qp.P = Eigen::MatrixXd::Zero(total_dim, total_dim);
    qp.P.block(0, 0, dim, dim) = P_position;
    for (int k = 0; k < k_slack; ++k) {
        qp.P(dim + k, dim + k) = w_slack_collision;
    }
    qp.q = Eigen::VectorXd::Zero(total_dim);
    qp.q.segment(0, dim) = q_position;

    qp.rows.reserve(hard_rows.size() + 2 * collision_rows.size());
    for (const LinearRow& r : hard_rows) {
        qp.rows.push_back(pad_hard_row(r, total_dim));
    }
    for (int k = 0; k < k_slack; ++k) {
        auto [soft, bound] = soften_collision_row(collision_rows[k], total_dim, dim + k);
        qp.rows.push_back(std::move(soft));
        qp.rows.push_back(std::move(bound));
    }
    return qp;
}

// The hard rows every drone QP carries: the per-axis
// speed/acceleration/jerk box on the derivative control points (rows that
// only involve pinned control points are left out: no free variable can
// change them) and, with an altitude floor, z >= floor + kFloorPlanningMarginM
// on every free control point.
std::vector<LinearRow> build_hard_kinematic_rows(const FreeIndexMap& map, const DroneWorkspace& ws,
                                                 const CoreConfig& config) {
    const int num_free = map.num_free();
    const int dim = 3 * num_free;
    const double v_limit = trajectory::inscribed_axis_limit(config.kinematics.v_max_mps);
    const double a_limit = trajectory::inscribed_axis_limit(config.kinematics.a_max_mps2);
    const double j_limit = trajectory::inscribed_axis_limit(config.kinematics.j_max_mps3);

    std::vector<LinearRow> rows;
    rows.reserve(3 * (ws.ops.velocity.rows() + ws.ops.acceleration.rows() + ws.ops.jerk.rows()) + num_free);
    const auto add_box = [&](const Eigen::MatrixXd& op, double limit) {
        for (int k = 0; k < op.rows(); ++k) {
            for (int axis = 0; axis < 3; ++axis) {
                LinearRow r = build_box_row(map, ws.control_points, op.row(k), axis, limit);
                if (r.coeffs.cwiseAbs().sum() > 1e-12) rows.push_back(std::move(r));
            }
        }
    };
    add_box(ws.ops.velocity, v_limit);
    add_box(ws.ops.acceleration, a_limit);
    add_box(ws.ops.jerk, j_limit);
    // Altitude floor. Hard, like the kinematic box; the convex
    // hull property then keeps the whole spline above it (the pinned control
    // points are floor-safe by construction, see floor_safe_velocity()).
    if (const std::optional<double> floor = drone_floor(ws.floor_m, config)) {
        for (int i = 0; i < num_free; ++i) {
            LinearRow r;
            r.coeffs = Eigen::RowVectorXd::Zero(dim);
            r.coeffs(2 * num_free + i) = 1.0;
            r.lower = *floor + kFloorPlanningMarginM;
            r.upper = OSQP_INFTY;
            rows.push_back(std::move(r));
        }
    }
    return rows;
}

// Moves a drone's free control points to the closest (least
// squares) path that satisfies its hard rows, before the SCP starts.
// The basic seed pins the first and last three control points from the
// boundary states and interpolates the middle linearly; where that meets the
// pinned ends the path needs more acceleration or jerk than allowed (on a
// real 150-drone show, every starting path; also with APF seeding off), so
// the first step's QPs with a trust region were infeasible and that step was
// an unbounded jump.
//
// This small QP (no collision rows) is solved accurately, unlike the SCP's
// own QPs: for the step d from the seed x0 (bounds shifted by each row's
// value at x0), with every row normalized by its largest coefficient, tight
// tolerances, and the result accepted only if every normalized row holds
// within kRepairRowTolerance.
enum SeedRepair { kSeedFlyable = 0, kSeedRepaired = 1, kSeedUnrepairable = 2 };

SeedRepair repair_to_flyable(const FreeIndexMap& map, DroneWorkspace& ws, double duration, const CoreConfig& config) {
    constexpr double kRepairRowTolerance = 1e-4;
    const int num_free = map.num_free();
    if (num_free == 0) return kSeedFlyable;
    const int dim = 3 * num_free;
    const std::vector<LinearRow> rows = build_hard_kinematic_rows(map, ws, config);
    const int m = static_cast<int>(rows.size());

    Eigen::VectorXd x0(dim);
    for (int axis = 0; axis < 3; ++axis) {
        for (int i = 0; i < num_free; ++i) x0(axis * num_free + i) = ws.control_points(map.free_begin + i, axis);
    }
    Eigen::MatrixXd A(m, dim);
    Eigen::VectorXd l(m), u(m);
    bool holds = true;
    for (int r = 0; r < m; ++r) {
        const double largest = rows[r].coeffs.cwiseAbs().maxCoeff();
        const double scale = largest > 1e-14 ? 1.0 / largest : 1.0;
        A.row(r) = rows[r].coeffs * scale;
        const double at_x0 = rows[r].coeffs.dot(x0);
        l(r) = rows[r].lower <= -OSQP_INFTY ? -OSQP_INFTY : (rows[r].lower - at_x0) * scale;
        u(r) = rows[r].upper >= OSQP_INFTY ? OSQP_INFTY : (rows[r].upper - at_x0) * scale;
        holds = holds && l(r) <= kRepairRowTolerance && u(r) >= -kRepairRowTolerance;  // d = 0 satisfies it
    }
    if (holds) return kSeedFlyable;

    // min 1/2 |d|^2 subject to l <= A d <= u.
    Eigen::SparseMatrix<double> P_sparse(dim, dim);
    P_sparse.setIdentity();
    P_sparse.makeCompressed();
    Eigen::SparseMatrix<double> A_sparse = A.sparseView(1e-14, 1.0);
    A_sparse.makeCompressed();
    std::vector<OSQPFloat> p_x(P_sparse.valuePtr(), P_sparse.valuePtr() + P_sparse.nonZeros());
    std::vector<OSQPInt> p_i(P_sparse.innerIndexPtr(), P_sparse.innerIndexPtr() + P_sparse.nonZeros());
    std::vector<OSQPInt> p_p(P_sparse.outerIndexPtr(), P_sparse.outerIndexPtr() + P_sparse.outerSize() + 1);
    std::vector<OSQPFloat> a_x(A_sparse.valuePtr(), A_sparse.valuePtr() + A_sparse.nonZeros());
    std::vector<OSQPInt> a_i(A_sparse.innerIndexPtr(), A_sparse.innerIndexPtr() + A_sparse.nonZeros());
    std::vector<OSQPInt> a_p(A_sparse.outerIndexPtr(), A_sparse.outerIndexPtr() + A_sparse.outerSize() + 1);
    OSQPCscMatrix P_csc;
    OSQPCscMatrix_set_data(&P_csc, dim, dim, static_cast<OSQPInt>(p_x.size()), p_x.data(), p_i.data(), p_p.data());
    OSQPCscMatrix A_csc;
    OSQPCscMatrix_set_data(&A_csc, m, dim, static_cast<OSQPInt>(a_x.size()), a_x.data(), a_i.data(), a_p.data());
    std::vector<OSQPFloat> q_vec(dim, 0.0);
    std::vector<OSQPFloat> l_vec(l.data(), l.data() + m);
    std::vector<OSQPFloat> u_vec(u.data(), u.data() + m);

    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.polishing = 1;
    settings.eps_abs = 1e-6;
    settings.eps_rel = 1e-6;
    settings.max_iter = 20000;

    SeedRepair outcome = kSeedUnrepairable;
    OSQPSolver* solver = nullptr;
    if (osqp_setup(&solver, &P_csc, q_vec.data(), &A_csc, l_vec.data(), u_vec.data(), m, dim, &settings) == 0 &&
        solver != nullptr) {
        osqp_solve(solver);
        const OSQPInt status = solver->info->status_val;
        if (status == OSQP_SOLVED || status == OSQP_SOLVED_INACCURATE) {
            const Eigen::Map<const Eigen::VectorXd> step(solver->solution->x, dim);
            const Eigen::VectorXd values = A * step;
            bool rows_hold = true;
            for (int r = 0; r < m && rows_hold; ++r) {
                rows_hold = values(r) >= l(r) - kRepairRowTolerance && values(r) <= u(r) + kRepairRowTolerance;
            }
            if (rows_hold) {
                for (int axis = 0; axis < 3; ++axis) {
                    for (int i = 0; i < num_free; ++i) {
                        ws.control_points(map.free_begin + i, axis) = x0(axis * num_free + i) + step(axis * num_free + i);
                    }
                }
                rebuild_spline(ws, duration);
                outcome = kSeedRepaired;
            }
        }
    }
    if (solver != nullptr) osqp_cleanup(solver);
    return outcome;
}

// Solves one drone's QP: minimize w_smoothness_snap*snap^2 +
// w_smoothness_jerk*jerk^2 + 1/2*w_slack_collision*sum(s_k^2), subject to
// kinematic box constraints, the soft-slack collision rows already assembled
// for it, and a trust-region step limit anchored at `trust_region_anchor`
// (this iteration's starting point).
//
// Two-tier infeasibility self-healing (kept alongside the soft-slack
// collision rows added below), tried in order whenever OSQP reports
// anything other than OSQP_SOLVED/OSQP_SOLVED_INACCURATE (this function's
// earlier, wrong behavior was to silently keep the drone's unmodified
// previous iterate on *any* non-solved status):
//   Tier 0: kinematic + collision (soft) + trust region.
//   Tier 1: drop the trust region (a soft stabilizer, not a physical
//     requirement) and retry with kinematic + collision (soft) only.
//   Tier 2: if still infeasible (now only possible from the hard kinematic-
//     box rows themselves, e.g. too tight relative to the trust region),
//     jitter the free control points orthogonally, rebuild this drone's own
//     linearization (`recollect_collision_rows`) against that jittered
//     state, and retry once more without a trust region.
Eigen::MatrixXd solve_drone_qp(const FreeIndexMap& map, DroneWorkspace& ws, double duration, const CoreConfig& config,
                                const Eigen::MatrixXd& H, const std::vector<LinearRow>& collision_rows,
                                const Eigen::MatrixXd& trust_region_anchor, double trust_region_delta_m,
                                const std::function<std::vector<LinearRow>()>& recollect_collision_rows,
                                int* tier_used = nullptr) {
    // `tier_used` (optional): 0-2 = the tier that solved, 3 = all tiers failed.
    const auto used = [&](int tier) {
        if (tier_used) *tier_used = tier;
    };
    const int num_free = map.num_free();
    if (num_free == 0) {
        used(0);
        return ws.control_points;  // fully pinned spline, nothing to optimize
    }
    const int n = map.num_control_points;
    Eigen::VectorXi free_idx(num_free);
    for (int i = 0; i < num_free; ++i) free_idx(i) = map.free_begin + i;

    const Eigen::MatrixXd H_ff = H(free_idx, free_idx);
    Eigen::MatrixXd H_fc(num_free, n);
    for (int i = 0; i < num_free; ++i) H_fc.row(i) = H.row(free_idx(i));

    const int dim = 3 * num_free;
    Eigen::MatrixXd P_dense = Eigen::MatrixXd::Zero(dim, dim);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(dim);
    for (int axis = 0; axis < 3; ++axis) {
        P_dense.block(axis * num_free, axis * num_free, num_free, num_free) = 2.0 * H_ff;
        Eigen::VectorXd fixed_axis(n);
        for (int idx = 0; idx < n; ++idx) fixed_axis(idx) = ws.control_points(idx, axis);
        q.segment(axis * num_free, num_free) = 2.0 * H_fc * fixed_axis;
    }
    // q depends only on the pinned boundary control points (fixed_axis
    // above), which tier 2's jitter never touches, so P_dense/q stay valid
    // across every tier.

    std::vector<LinearRow> kinematic_rows = build_hard_kinematic_rows(map, ws, config);
    // Kinematic box rows are also unaffected by tier 2's jitter: their free
    // coefficients come from the structural D-matrix and their constant
    // offset comes from the (untouched) pinned boundary control points.

    std::vector<LinearRow> soft_collision_rows;
    for (const LinearRow& r : collision_rows) {
        if (r.coeffs.cwiseAbs().sum() > 1e-12) soft_collision_rows.push_back(r);
    }

    std::vector<LinearRow> hard_rows_with_trust_region = kinematic_rows;
    hard_rows_with_trust_region.reserve(kinematic_rows.size() + static_cast<size_t>(dim));
    for (int axis = 0; axis < 3; ++axis) {
        for (int i = 0; i < num_free; ++i) {
            const int flat_index = axis * num_free + i;
            const double anchor = trust_region_anchor(map.free_begin + i, axis);
            hard_rows_with_trust_region.push_back(
                build_trust_region_row(flat_index, dim, anchor, trust_region_delta_m));
        }
    }

    const SoftQp tier0 = assemble_soft_qp(P_dense, q, hard_rows_with_trust_region, soft_collision_rows,
                                           config.solver.w_slack_collision);
    if (auto solved = try_solve_qp(map, ws.control_points, tier0.P, tier0.q, tier0.rows)) {
        used(0);
        return *solved;  // Tier 0
    }
    const SoftQp tier1 = assemble_soft_qp(P_dense, q, kinematic_rows, soft_collision_rows,
                                           config.solver.w_slack_collision);
    if (auto solved = try_solve_qp(map, ws.control_points, tier1.P, tier1.q, tier1.rows)) {
        used(1);
        return *solved;  // Tier 1
    }

    apply_orthogonal_jitter(map, ws, config.solver.jitter_magnitude_m);
    rebuild_spline(ws, duration);
    std::vector<LinearRow> jittered_collision_rows;
    for (const LinearRow& r : recollect_collision_rows()) {
        if (r.coeffs.cwiseAbs().sum() > 1e-12) jittered_collision_rows.push_back(r);
    }
    const SoftQp tier2 =
        assemble_soft_qp(P_dense, q, kinematic_rows, jittered_collision_rows, config.solver.w_slack_collision);
    if (auto solved = try_solve_qp(map, ws.control_points, tier2.P, tier2.q, tier2.rows)) {
        used(2);
        return *solved;  // Tier 2
    }
    // All 3 tiers infeasible (now only possible from the hard kinematic-box
    // rows): keep the jittered (unoptimized) iterate rather than the
    // pre-jitter one — it has already broken the exact symmetric alignment
    // that caused the deadlock, giving the *next* SCP iteration fresh
    // geometry to converge from instead of retrying the same deadlock.
    used(3);
    return ws.control_points;
}

// Sampled Euclidean distance between a CandidatePair's two drones at their
// window's midpoint time, reading each drone's *current* spline. Shared by
// the conflict-edge builder (below) and the iteration's worst-case
// separation bookkeeping in solve_single_stage().
double pair_window_distance(const collision::CandidatePair& pair, const std::vector<DroneWorkspace>& workspaces,
                             const std::map<int, int>& drone_index, double duration, double time_bucket_s) {
    auto it_i = drone_index.find(pair.drone_i);
    auto it_j = drone_index.find(pair.drone_j);
    if (it_i == drone_index.end() || it_j == drone_index.end()) {
        return std::numeric_limits<double>::infinity();
    }
    const double t0 = pair.window_index * time_bucket_s;
    const double t1 = std::min(duration, (pair.window_index + 1) * time_bucket_s);
    const double t_sample = 0.5 * (t0 + t1);
    const Eigen::Vector3d pi = workspaces[it_i->second].spline->position(t_sample);
    const Eigen::Vector3d pj = workspaces[it_j->second].spline->position(t_sample);
    return (pi - pj).norm();
}

// Conflict-graph edges for clustering/coloring purposes: collapses `pairs`
// (broad-phase, possibly several windows per drone pair) down to one edge
// per unique drone pair, keeping its minimum sampled distance across all
// matching windows.
//
// SAFETY-CRITICAL INVARIANT: this must produce an edge for *every* drone
// pair that collect_collision_rows() below will build a mutual QP row for
// (i.e. every pair appearing anywhere in `pairs`), with no additional
// filtering — never a strict subset. Graph coloring/clustering only
// guarantees two same-color (or cross-cluster) drones are safe to solve on
// different OpenMP threads concurrently *because* neither one's collision
// rows read the other's DroneWorkspace; if a genuine collision-row pair were
// ever missing from this edge set, the two drones could end up in different
// colors/clusters and race on each other's `control_points`/`spline` (a
// std::unique_ptr rebuilt mid-solve by the other thread) while this
// function's own caller is mid-write — undefined behavior, not just a
// weaker constraint. This is not a hypothetical: an earlier version of this
// function filtered edges to distance < cluster_distance_threshold_m (a
// literal reading of "don't wire an
// edge just because bounding boxes overlap at long range") while
// collect_collision_rows kept using the full unfiltered `pairs` list for QP
// rows — that mismatch let two drones sharing a real collision row land in
// different colors and run concurrently, and manifested on a real 300-drone
// show as two drones ending up 0.029 m apart (required >= 1.5 m) at
// t=34.7s. Section 4's cluster_distance_threshold_m/max_cluster_size
// weak-edge-cut is therefore NOT applied here (see the connected_components
// call site below) until/unless collect_collision_rows is also restricted
// to the identical filtered/capped edge set — flagged to the user as a
// deliberate deviation from a literal reading of that section, made to fix
// this safety regression; both config keys are still parsed and available
// for whoever revisits this in a way that keeps the two in lockstep.
std::vector<collision::ConflictEdge> build_conflict_edges(const std::vector<collision::CandidatePair>& pairs,
                                                           const std::vector<DroneWorkspace>& workspaces,
                                                           const std::map<int, int>& drone_index, double duration,
                                                           double time_bucket_s) {
    std::map<std::pair<int, int>, double> min_distance_by_pair;
    for (const auto& pair : pairs) {
        const double d = pair_window_distance(pair, workspaces, drone_index, duration, time_bucket_s);
        const auto key = std::make_pair(pair.drone_i, pair.drone_j);
        auto it = min_distance_by_pair.find(key);
        if (it == min_distance_by_pair.end() || d < it->second) {
            min_distance_by_pair[key] = d;
        }
    }
    std::vector<collision::ConflictEdge> edges;
    edges.reserve(min_distance_by_pair.size());
    for (const auto& [key, distance] : min_distance_by_pair) {
        edges.push_back(collision::ConflictEdge{key.first, key.second, distance});
    }
    return edges;
}

// Unordered-pair key for dynamic-collocation lookups, independent of which
// drone happens to be "self" vs. "other" when collect_collision_rows() below
// looks a pair up.
std::pair<int, int> ordered_pair_key(int a, int b) { return a < b ? std::make_pair(a, b) : std::make_pair(b, a); }

using DynamicCollocationMap = std::map<std::pair<int, int>, std::vector<double>>;

// Collects the collision-row set for `self_ws` against every candidate
// partner in `pairs`, reading each partner's *current* spline (which, under
// Gauss-Seidel ordering within a cluster, may already reflect this same
// iteration's update).
//
// Adaptive Cutting-Plane Collocation: beyond the one per-window midpoint
// sample below, also emits one
// extra hard row at every dynamic-collocation timestamp `dynamic_collocations`
// carries for this pair (populated by scan_pair_separation() when the
// iterate this iteration starts from was evaluated) — these are the actual detected near-miss
// extrema, so OSQP constrains the true worst point of a close pass instead
// of only its window midpoint.
std::vector<LinearRow> collect_collision_rows(const FreeIndexMap& map, const DroneWorkspace& self_ws,
                                               const std::vector<collision::CandidatePair>& pairs,
                                               const std::vector<DroneWorkspace>& workspaces,
                                               const std::map<int, int>& drone_index, double duration,
                                               double time_bucket_s, double enforced_distance,
                                               const DynamicCollocationMap& dynamic_collocations = {}) {
    std::vector<LinearRow> collision_rows;
    for (const auto& pair : pairs) {
        if (pair.drone_i != self_ws.drone_id && pair.drone_j != self_ws.drone_id) continue;
        const bool self_is_i = (pair.drone_i == self_ws.drone_id);
        const int other_id = self_is_i ? pair.drone_j : pair.drone_i;
        auto it = drone_index.find(other_id);
        if (it == drone_index.end()) continue;
        const DroneWorkspace& other_ws = workspaces[it->second];

        // One linearization sample per window (the collocation formula
        // is written for a single collocation time t_c per constraint); too
        // many samples per pair pile up more half-space rows than 12 free
        // variables can jointly satisfy, which is its own source of spurious
        // PRIMAL_INFEASIBLE independent of the seeding used.
        const double t0 = pair.window_index * time_bucket_s;
        const double t1 = std::min(duration, (pair.window_index + 1) * time_bucket_s);
        {
            const double t_sample = 0.5 * (t0 + t1);
            const Eigen::Vector3d self_pos_k = self_ws.spline->position(t_sample);
            const Eigen::Vector3d other_pos_k = other_ws.spline->position(t_sample);
            collision_rows.push_back(
                build_collision_row(map, self_ws, self_pos_k, other_pos_k, enforced_distance, t_sample));
        }

        const auto dyn_it = dynamic_collocations.find(ordered_pair_key(self_ws.drone_id, other_id));
        if (dyn_it != dynamic_collocations.end()) {
            for (const double t_star : dyn_it->second) {
                const Eigen::Vector3d self_pos_star = self_ws.spline->position(t_star);
                const Eigen::Vector3d other_pos_star = other_ws.spline->position(t_star);
                collision_rows.push_back(
                    build_collision_row(map, self_ws, self_pos_star, other_pos_star, enforced_distance, t_star));
            }
        }
    }
    return collision_rows;
}

// Dense scan of the real (non-linearized) separation between drones `a` and
// `b` over [t_lo, t_hi] (NOT necessarily the whole transition — see the
// caller's window-bounding comment), at `scan_frequency_hz` (the gatekeeper's
// own rate).
//
// `min_distance` is the pair's smallest sampled distance: what the gatekeeper
// will measure for this pair, so the solver ranks its iterates by it
// instead of by one midpoint sample per window. `dips` are the
// local minima strictly under `d_enforce`, closest first, capped at
// `max_points` (<= 0: all of them): each gets its own collision row next
// iteration. A boundary sample (s == 0 or the last sample) counts as a local
// minimum if it's <= its one neighbor, so a near-miss still closing right at
// the scanned range's edge isn't missed.
struct PairScan {
    double min_distance = std::numeric_limits<double>::infinity();
    std::vector<double> dips;
};

PairScan scan_pair_separation(const DroneWorkspace& a, const DroneWorkspace& b, double t_lo, double t_hi,
                              double scan_frequency_hz, double d_enforce, int max_points) {
    PairScan result;
    if (t_hi <= t_lo) return result;
    const double dt = 1.0 / std::max(1.0, scan_frequency_hz);
    const double span = t_hi - t_lo;
    const int num_samples = std::max(2, static_cast<int>(std::ceil(span / dt)) + 1);

    std::vector<double> times(num_samples);
    std::vector<double> dist(num_samples);
    for (int s = 0; s < num_samples; ++s) {
        times[s] = std::min(t_hi, t_lo + s * dt);
        dist[s] = (a.spline->position(times[s]) - b.spline->position(times[s])).norm();
        result.min_distance = std::min(result.min_distance, dist[s]);
    }

    std::vector<std::pair<double, double>> minima;  // (distance, time)
    for (int s = 0; s < num_samples; ++s) {
        if (dist[s] >= d_enforce) continue;
        const bool left_ok = (s == 0) || dist[s] <= dist[s - 1];
        const bool right_ok = (s == num_samples - 1) || dist[s] <= dist[s + 1];
        if (left_ok && right_ok) minima.emplace_back(dist[s], times[s]);
    }
    std::sort(minima.begin(), minima.end());
    for (const auto& [d, t] : minima) {
        if (max_points > 0 && static_cast<int>(result.dips.size()) >= max_points) break;
        result.dips.push_back(t);
    }
    return result;
}

// Adaptive control-point count:
// N_ctrl = max(min_points, min_points + 2*floor(D_max/15m)). Longer
// transitions get more free interior points (in pairs, to keep the 6
// boundary-pinned + N-6 free split even) so there is enough geometric
// freedom to bend around collisions without the free-variable count
// exploding for the common short-hop case.
int adaptive_num_control_points(double d_max, int min_points) {
    return std::max(min_points, min_points + 2 * static_cast<int>(std::floor(d_max / 15.0)));
}

}  // namespace

// One independent Cluster-based Gauss-Seidel SCP solve over `problems` for
// exactly `duration` seconds, returning each drone's control points in the
// same order as `problems`. This is the whole of what `solve()` used to be
// before mega-cluster decomposition was added on top: the
// public `solve()` below is now a thin orchestrator that calls this once per
// sub-stage when a transition is too long to solve as a single problem.
//
// `forced_num_control_points`, when > 0, skips this call's own
// adaptive_num_control_points and uses that count instead. A mega-cluster
// sub-stage's own d_max is only its short slice of the whole transition, so
// letting each sub-stage compute its control-point count independently
// starves it of the free interior points a single whole-duration solve
// would have had for the same geometry — confirmed via
// tools/scripts/smoke_test_stage2.py's symmetric-crossing test, whose
// closest approach measured 1.499 m (< the 1.5 m requirement) when split
// into 3 sub-stages (10 control points each, d_max/3 per stage) vs. exactly
// 1.500 m solved whole (14 control points, from the full d_max). solve()
// below passes the whole transition's own adaptive count through this
// parameter for every sub-stage so splitting a transition never reduces its
// geometric degrees of freedom below what the undecomposed solve would have
// used.
// `min_separation` is the worst-case real (non-linearized) pairwise
// separation this function tracks internally, purely to pick the best
// iterate to return (best_min_separation below) when Gauss-Seidel SCP
// doesn't converge monotonically. This is no longer the
// caller's final safety verdict — solve()'s Decoupled Continuous Gatekeeper
// re-derives that independently from the returned control
// points, on purpose, because this field is built from the same
// per-iteration candidate-pair snapshot the QP rows come from and can
// under-report the true continuous-time worst case for a pair that drifts
// closer between collocation samples without being re-flagged that same
// iteration (see stage2_nway_conflict_limitation memory). +infinity means
// no candidate pairs ever existed (nothing to be unsafe about).
struct SingleStageResult {
    std::vector<Eigen::MatrixXd> control_points;
    double min_separation = std::numeric_limits<double>::infinity();
};

// The caller's progress callback plus the attempt /
// sub-stage fields of the ScpIteration events solve_single_stage() emits.
// Passed as a pointer that is null when no callback is set, so the no-callback
// path builds no event at all.
struct IterationProgress {
    const ProgressCallback* callback = nullptr;
    ProgressEvent event;  // kind, attempt, max_attempts, duration_sec, substage, substage_count preset
};

SingleStageResult solve_single_stage(const std::vector<DroneTransitionProblem>& problems, double duration,
                                      const CoreConfig& config, int forced_num_control_points = 0,
                                      const IterationProgress* progress = nullptr) {
    const int num_drones = static_cast<int>(problems.size());

    double d_max = 0.0;
    for (const auto& p : problems) {
        d_max = std::max(d_max, (p.end.position - p.start.position).norm());
    }
    const int n = forced_num_control_points > 0
                      ? forced_num_control_points
                  : config.solver.adaptive_control_points
                      ? adaptive_num_control_points(d_max, config.solver.num_control_points_min)
                      : config.solver.num_control_points_min;
    FreeIndexMap map(n);
    const double enforced_min_distance = config.safety.min_distance_m * (1.0 + config.solver.collision_margin_fraction);

    // one joint N-body APF simulation across every
    // drone in this stage (not per-drone independent like the retired
    // parity-bow seeder), so a locally dense region balloons outward as a
    // whole instead of relying on a two-group parity split.
    const Clock::time_point setup_start = Clock::now();
    std::vector<trajectory::BoundaryConditions> starts(num_drones), ends(num_drones);
    for (int i = 0; i < num_drones; ++i) {
        starts[i] = problems[i].start;
        ends[i] = problems[i].end;
    }
    const std::vector<Eigen::MatrixXd> seeded_control_points =
        trajectory::seed_control_points_with_apf(starts, ends, duration, n, config.solver.apf_seeding);

    std::vector<DroneWorkspace> workspaces(num_drones);
    for (int i = 0; i < num_drones; ++i) {
        workspaces[i].drone_id = problems[i].drone_id;
        workspaces[i].keep_out = problems[i].keep_out;
        // A prescribed (held) drone is fixed like a parked one,
        // with the control points it was given for this window.
        const bool held = problems[i].held_control_points.rows() > 0;
        if (held && problems[i].held_control_points.rows() != n) {
            throw std::logic_error("held control points of drone " + std::to_string(problems[i].drone_id) + ": " +
                                   std::to_string(problems[i].held_control_points.rows()) + " rows, expected " +
                                   std::to_string(n));
        }
        workspaces[i].fixed = held || trajectory::is_stationary_hold(problems[i].start, problems[i].end);
        workspaces[i].floor_m = problems[i].floor_m;
        workspaces[i].control_points = held ? problems[i].held_control_points : seeded_control_points[i];
        if (const std::optional<double> floor = drone_floor(workspaces[i].floor_m, config)) {
            // Lift a seed that dips below the floor's planning buffer onto
            // it, so the first iteration's trust region already contains a
            // floor-feasible point. A fixed (parked) drone keeps its constant
            // path on the pad, which is at or above the floor.
            if (!workspaces[i].fixed) {
                for (int idx = map.free_begin; idx < map.free_end; ++idx) {
                    double& z = workspaces[i].control_points(idx, 2);
                    z = std::max(z, *floor + kFloorPlanningMarginM);
                }
            }
        }
        workspaces[i].ops = trajectory::build_derivative_operators(n, duration);
        rebuild_spline(workspaces[i], duration);
    }

    // Objective Hessian (identical across drones: same duration, same
    // control-point count) built once: H = w_snap * D_snap^T G_snap D_snap +
    // w_jerk * D_jerk^T G_jerk D_jerk.
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n, n);
    if (num_drones > 0) {
        const DerivativeOperators& ops0 = workspaces[0].ops;
        const Eigen::MatrixXd G_snap =
            trajectory::integrate_basis_gram(ops0.snap_knots, trajectory::kDegree - 4, n - 4);
        const Eigen::MatrixXd G_jerk =
            trajectory::integrate_basis_gram(ops0.jerk_knots, trajectory::kDegree - 3, n - 3);
        H = config.weights.w_smoothness_snap * (ops0.snap.transpose() * G_snap * ops0.snap) +
            config.weights.w_smoothness_jerk * (ops0.jerk.transpose() * G_jerk * ops0.jerk);
    }

    std::map<int, int> drone_index;
    for (int i = 0; i < num_drones; ++i) drone_index[workspaces[i].drone_id] = i;

    const double time_bucket_s =
        compute_time_bucket_s(duration, config.solver.adaptive_time_bucketing, config.solver.target_time_windows);
    const int num_windows = std::max(1, static_cast<int>(std::ceil(duration / time_bucket_s)));

    // Every iterate is judged by what the gatekeeper will
    // measure, not by one midpoint sample per broad-phase window. Before, the
    // collision rows, the "best iterate" choice and the reported separation
    // all used those midpoints (~0.6 s apart, 2-4 m of relative motion), and
    // on real shows the solver believed >= 1.45 m in 21 of 28 attempts while
    // the gatekeeper failed 16 (e.g. 1.509 m believed, 1.281 m real).
    struct IterateEvaluation {
        std::vector<collision::CandidatePair> pairs;
        // Tight broad phase: pair_gaps[k] is the gap between the two drones'
        // (unpadded) sampled boxes in pairs[k]'s window, which lets each step
        // keep only the pairs its own trust region can reach. 0 in the legacy
        // broad phase (every pair is kept).
        std::vector<double> pair_gaps;
        std::vector<collision::ConflictEdge> conflict_edges;
        DynamicCollocationMap dips;  // pair -> times of dips under enforced_min_distance
        double min_separation = std::numeric_limits<double>::infinity();  // dense scan, gatekeeper rate
        double separation_deficit = 0.0;  // sum over pairs of max(0, enforced - pair minimum)
        double intrusion = 0.0;
        double smoothness = 0.0;          // the QPs' own objective, summed over drones
        double broad_phase_sec = 0.0;     // progress-event timing only
        double scan_sec = 0.0;
    };

    // Tight broad phase: the farthest box gap at which a pair
    // still needs a collision row in a step with trust-region radius
    // `radius`. The radius bounds each free control point per axis, so a
    // control point (and, by the convex hull property, every point of the
    // spline) would move at most sqrt(3) * radius; both drones of a pair can
    // move in the same Gauss-Seidel sweep, so a pair farther apart than
    // enforced + 2 sqrt(3) radius gets no row. This is not a guarantee: OSQP's
    // loose solutions overshoot the radius (measured 2026-10-02:
    // ~0.2 m median and over 0.7 m in 5 % of steps at radii under 0.1 m) and
    // the fallback tiers have none. A step that overshoots into a pair without
    // a row is rejected by the evaluation below, which rebuilds the pairs from
    // scratch. An extra 0.75 m per drone for the overshoot was tried and kept
    // nothing better. Each box is built from 5
    // samples per window, so each side also gets curve_margin for the spline
    // bulging between samples.
    const bool tight_broad_phase = config.solver.tight_broad_phase;
    const double curve_margin = config.solver.broad_phase_curve_margin_m;
    const auto pair_reach = [&](double radius) {
        return enforced_min_distance + 2.0 * std::sqrt(3.0) * radius + 2.0 * curve_margin;
    };
    // An evaluated iterate doesn't move: a pair whose boxes stay farther
    // apart than this can't be under enforced_min_distance anywhere.
    const double scan_reach = enforced_min_distance + 2.0 * curve_margin;

    // `radius`: the largest trust region a step built on this iterate will
    // use (ignored by the legacy broad phase).
    const auto evaluate_iterate = [&](double radius) {
        IterateEvaluation ev;
        const Clock::time_point broad_phase_start = Clock::now();
        // Rebuild the 4D spatio-temporal conflict graph from the *current*
        // trajectories (pseudocode
        // calls find_conflict_clusters() inside the loop).
        collision::SpatioTemporalHash hash(enforced_min_distance, time_bucket_s);
        std::vector<Eigen::Vector3d> box_lo(static_cast<size_t>(num_drones) * num_windows);
        std::vector<Eigen::Vector3d> box_hi(box_lo.size());
        const double reach = pair_reach(radius);
        for (int i = 0; i < num_drones; ++i) {
            const DroneWorkspace& ws = workspaces[i];
            for (int w = 0; w < num_windows; ++w) {
                const double t0 = w * time_bucket_s;
                const double t1 = std::min(duration, (w + 1) * time_bucket_s);
                Eigen::Vector3d bbox_min = ws.spline->position(t0);
                Eigen::Vector3d bbox_max = bbox_min;
                for (double frac : {0.25, 0.5, 0.75, 1.0}) {
                    const Eigen::Vector3d p = ws.spline->position(t0 + frac * (t1 - t0));
                    bbox_min = bbox_min.cwiseMin(p);
                    bbox_max = bbox_max.cwiseMax(p);
                }
                box_lo[static_cast<size_t>(i) * num_windows + w] = bbox_min;
                box_hi[static_cast<size_t>(i) * num_windows + w] = bbox_max;
                // Legacy: pad by the full enforced separation, not just the
                // visual safety_radius_m: the broad phase must keep flagging a
                // pair as a candidate for as long as their true point-wise
                // distance could plausibly be under enforced_min_distance.
                // Tight: half the reach on each box, so two boxes overlap
                // exactly when their gap is under the reach (the hash returns
                // a superset, filtered by the exact gap below).
                const double pad =
                    tight_broad_phase ? 0.5 * reach : config.safety.safety_radius_m + enforced_min_distance;
                const Eigen::Vector3d margin = Eigen::Vector3d::Constant(pad);
                hash.insert(collision::DroneWindow{ws.drone_id, w, bbox_min - margin, bbox_max + margin});
            }
        }
        // Euclidean gap between two axis-aligned boxes (0 when they overlap).
        const auto box_gap = [&](const collision::CandidatePair& p) {
            const size_t a = static_cast<size_t>(drone_index.at(p.drone_i)) * num_windows + p.window_index;
            const size_t b = static_cast<size_t>(drone_index.at(p.drone_j)) * num_windows + p.window_index;
            const Eigen::Vector3d sep =
                (box_lo[a] - box_hi[b]).cwiseMax(box_lo[b] - box_hi[a]).cwiseMax(Eigen::Vector3d::Zero());
            return sep.norm();
        };

        // Which pairs (and windows) get the dense scan below.
        std::vector<collision::CandidatePair> scan_pairs;
        if (!tight_broad_phase) {
            ev.pairs = hash.find_candidate_pairs();
            ev.pair_gaps.assign(ev.pairs.size(), 0.0);
            // Conflict-graph edges for clustering/coloring: one per unique drone
            // pair appearing in `pairs` (see build_conflict_edges()'s safety-
            // critical invariant comment: this must stay a 1:1 mirror of every
            // pair collect_collision_rows() can generate a mutual QP row for).
            ev.conflict_edges = build_conflict_edges(ev.pairs, workspaces, drone_index, duration, time_bucket_s);
            scan_pairs = ev.pairs;
        } else {
            std::map<std::pair<int, int>, double> min_gap_by_pair;
            for (const auto& p : hash.find_candidate_pairs()) {
                const double gap = box_gap(p);
                if (gap >= reach) continue;
                ev.pairs.push_back(p);
                ev.pair_gaps.push_back(gap);
                if (gap < scan_reach) scan_pairs.push_back(p);
                auto [it, inserted] = min_gap_by_pair.try_emplace(std::make_pair(p.drone_i, p.drone_j), gap);
                if (!inserted) it->second = std::min(it->second, gap);
            }
            // Only for the conflict_pairs count: each step builds its own
            // coloring edges from the pairs it keeps (see the SCP loop).
            ev.conflict_edges.reserve(min_gap_by_pair.size());
            for (const auto& [key, gap] : min_gap_by_pair) {
                ev.conflict_edges.push_back(collision::ConflictEdge{key.first, key.second, gap});
            }
        }
        ev.broad_phase_sec = seconds_since(broad_phase_start);
        const Clock::time_point scan_start = Clock::now();

        // Dense scan of every conflict pair, bounded to the time range the
        // broad phase flags it in (padded by one time_bucket_s each side, to
        // catch a minimum straddling a window edge): a pair flagged nowhere
        // near a time range can't come within enforced_min_distance there.
        // Tight broad phase: only the pairs whose boxes come within
        // enforced_min_distance (+ curve margins) are scanned, over those
        // windows; a pair that never does adds nothing to the deficit.
        std::map<std::pair<int, int>, std::pair<int, int>> window_range_by_pair;
        for (const auto& p : scan_pairs) {
            const auto key = ordered_pair_key(p.drone_i, p.drone_j);
            auto it = window_range_by_pair.find(key);
            if (it == window_range_by_pair.end()) {
                window_range_by_pair[key] = {p.window_index, p.window_index};
            } else {
                it->second.first = std::min(it->second.first, p.window_index);
                it->second.second = std::max(it->second.second, p.window_index);
            }
        }
        std::vector<std::pair<int, int>> scan_keys;
        if (!tight_broad_phase) {
            // Same order as before (the conflict edges), so results don't move.
            for (const auto& edge : ev.conflict_edges) scan_keys.push_back(ordered_pair_key(edge.drone_i, edge.drone_j));
        } else {
            for (const auto& [key, range] : window_range_by_pair) scan_keys.push_back(key);
        }
        const int num_scans = static_cast<int>(scan_keys.size());
        std::vector<PairScan> scans(num_scans);
        const double scan_hz = std::max(config.solver.cutting_plane.detection_frequency_hz,
                                        config.solver.continuous_gatekeeper.verification_frequency_hz);
#pragma omp parallel for schedule(dynamic)
        for (int e = 0; e < num_scans; ++e) {
            const auto& key = scan_keys[e];
            const auto range = window_range_by_pair.at(key);
            const double t_lo = std::max(0.0, range.first * time_bucket_s - time_bucket_s);
            const double t_hi = std::min(duration, (range.second + 1) * time_bucket_s + time_bucket_s);
            scans[e] = scan_pair_separation(workspaces[drone_index.at(key.first)],
                                            workspaces[drone_index.at(key.second)], t_lo, t_hi, scan_hz,
                                            enforced_min_distance,
                                            config.solver.cutting_plane.max_dynamic_collocations_per_pair);
        }
        for (int e = 0; e < num_scans; ++e) {
            ev.min_separation = std::min(ev.min_separation, scans[e].min_distance);
            ev.separation_deficit += std::max(0.0, enforced_min_distance - scans[e].min_distance);
            if (config.solver.cutting_plane.enabled && !scans[e].dips.empty()) {
                ev.dips[scan_keys[e]] = std::move(scans[e].dips);
            }
        }
        ev.scan_sec = seconds_since(scan_start);
        ev.intrusion = keep_out_intrusion(workspaces, duration, config);
        for (const auto& ws : workspaces) {
            for (int axis = 0; axis < 3; ++axis) {
                const Eigen::VectorXd c = ws.control_points.col(axis);
                ev.smoothness += c.dot(H * c);
            }
        }
        return ev;
    };

    // Merit, lexicographic: keep-out intrusion, then the total
    // separation deficit, then smoothness. `a` must be strictly better.
    const auto better_than = [](const IterateEvaluation& a, const IterateEvaluation& b) {
        constexpr double kTieM = 1e-6;
        if (a.intrusion < b.intrusion - kTieM) return true;
        if (a.intrusion > b.intrusion + kTieM) return false;
        if (a.separation_deficit < b.separation_deficit - kTieM) return true;
        if (a.separation_deficit > b.separation_deficit + kTieM) return false;
        return a.smoothness < b.smoothness - 1e-9 * std::max(1.0, std::abs(b.smoothness));
    };

    // Adaptive trust region: a step that doesn't make the
    // iterate better by the merit above is rejected (back to the best
    // iterate, radius halved), and an accepted step lets the radius grow back
    // up to trust_region_delta_m. With a fixed radius and no rejection the
    // Gauss-Seidel sweep (each drone linearized against neighbours that move
    // later in the same sweep) never settled: every step on real shows sat
    // at the radius (1.1-1.7 m), 0 of 1 875 iterations converged, and the
    // result was one frame of an oscillation.
    // Start from flyable paths (see repair_to_flyable()).
    int seed_repair_counts[3] = {0, 0, 0};  // already flyable, repaired, unrepairable
    if (config.solver.repair_seed) {
#pragma omp parallel for schedule(dynamic)
        for (int i = 0; i < num_drones; ++i) {
            if (workspaces[i].fixed) continue;
            const SeedRepair outcome = repair_to_flyable(map, workspaces[i], duration, config);
#pragma omp atomic
            seed_repair_counts[outcome] += 1;
        }
    }

    const double max_trust_region = config.solver.trust_region_delta_m;
    double trust_region = max_trust_region;
    std::vector<Eigen::MatrixXd> best_control_points(num_drones);
    for (int i = 0; i < num_drones; ++i) best_control_points[i] = workspaces[i].control_points;
    IterateEvaluation best = evaluate_iterate(trust_region);
    const double setup_sec = seconds_since(setup_start);
    int iterations_without_progress = 0;  // SolverOptions::scp_stall_iterations

    std::vector<int> solved_ids;
    solved_ids.reserve(num_drones);
    for (const auto& ws : workspaces) {
        if (!ws.fixed) solved_ids.push_back(ws.drone_id);
    }

    for (int iter = 0; iter < config.solver.max_scp_iterations; ++iter) {
        const Clock::time_point step_start = Clock::now();
        // The iteration starts from the best iterate (the last accepted one),
        // with its own pairs and dips.
        const DynamicCollocationMap& dynamic_collocations = best.dips;
        // Keep the pairs this step's trust region can reach.
        // best was evaluated for a radius >= this one (the radius only grows
        // after an accepted step, and that step's candidate was evaluated for
        // the grown radius), so this is a subset of best.pairs. The coloring
        // edges below are built from this same list, keeping
        // build_conflict_edges()'s invariant (every row pair is an edge).
        std::vector<collision::CandidatePair> step_pairs;
        std::vector<collision::ConflictEdge> step_edges;
        if (tight_broad_phase) {
            const double reach = pair_reach(trust_region);
            std::set<std::pair<int, int>> seen;
            for (size_t k = 0; k < best.pairs.size(); ++k) {
                if (best.pair_gaps[k] >= reach) continue;
                const auto& p = best.pairs[k];
                step_pairs.push_back(p);
                if (seen.insert({p.drone_i, p.drone_j}).second) {
                    step_edges.push_back(collision::ConflictEdge{p.drone_i, p.drone_j, best.pair_gaps[k]});
                }
            }
        }
        const std::vector<collision::CandidatePair>& pairs = tight_broad_phase ? step_pairs : best.pairs;
        const std::vector<collision::ConflictEdge>& conflict_edges =
            tight_broad_phase ? step_edges : best.conflict_edges;

        // A fixed (parked) drone is never solved, so it can't
        // race anyone and needs no cluster or color. Its pairs stay in
        // `pairs`, so the drones moving past it still get collision rows
        // against it. max_cluster_size is deliberately NOT passed to
        // connected_components (see build_conflict_edges()).
        std::vector<collision::ConflictEdge> solved_edges;
        solved_edges.reserve(conflict_edges.size());
        for (const auto& edge : conflict_edges) {
            if (!workspaces[drone_index.at(edge.drone_i)].fixed && !workspaces[drone_index.at(edge.drone_j)].fixed) {
                solved_edges.push_back(edge);
            }
        }
        const std::vector<std::vector<int>> clusters =
            collision::connected_components(solved_ids, solved_edges, /*max_cluster_size=*/0);

        std::vector<Eigen::MatrixXd> trust_region_anchor(num_drones);
        for (int i = 0; i < num_drones; ++i) trust_region_anchor[i] = workspaces[i].control_points;
        int tier_counts[4] = {0, 0, 0, 0};  // drone QPs solved by tier 0/1/2, or all failed
        double rows_cpu_sec = 0.0;
        double qp_cpu_sec = 0.0;
        int collision_row_count = 0;
        int color_count = 0;

        auto solve_one_drone = [&](int drone_id) {
            const int idx = drone_index.at(drone_id);
            DroneWorkspace& self_ws = workspaces[idx];
            const Clock::time_point rows_start = Clock::now();
            std::vector<LinearRow> collision_rows =
                collect_collision_rows(map, self_ws, pairs, workspaces, drone_index, duration, time_bucket_s,
                                        enforced_min_distance, dynamic_collocations);
            // Keep-out rows join the soft collision rows.
            for (LinearRow& r : collect_keep_out_rows(map, self_ws, duration, config)) collision_rows.push_back(std::move(r));
            const double rows_sec = seconds_since(rows_start);
            const int row_count = static_cast<int>(collision_rows.size());
            const auto recollect = [&]() {
                std::vector<LinearRow> rows =
                    collect_collision_rows(map, self_ws, pairs, workspaces, drone_index, duration, time_bucket_s,
                                           enforced_min_distance, dynamic_collocations);
                for (LinearRow& r : collect_keep_out_rows(map, self_ws, duration, config)) rows.push_back(std::move(r));
                return rows;
            };
            int tier = 0;
            const Clock::time_point qp_start = Clock::now();
            self_ws.control_points = solve_drone_qp(map, self_ws, duration, config, H, collision_rows,
                                                    trust_region_anchor[idx], trust_region, recollect, &tier);
            const double qp_sec = seconds_since(qp_start);
#pragma omp atomic
            tier_counts[tier] += 1;
#pragma omp atomic
            rows_cpu_sec += rows_sec;
#pragma omp atomic
            qp_cpu_sec += qp_sec;
#pragma omp atomic
            collision_row_count += row_count;
            if (const std::optional<double> floor = drone_floor(self_ws.floor_m, config)) {
                // Whatever the QP tier returned (an inaccurate
                // OSQP solution, or the unoptimized jittered iterate when all
                // tiers failed), lift any free control point still under the
                // floor onto it, so the floor holds by the convex hull property.
                for (int k = map.free_begin; k < map.free_end; ++k) {
                    double& z = self_ws.control_points(k, 2);
                    z = std::max(z, *floor);
                }
            }
            rebuild_spline(self_ws, duration);
        };

        const Clock::time_point sweep_start = Clock::now();
        if (config.solver.enable_graph_coloring) {
            // greedy graph coloring within each cluster;
            // same-color batches have no conflict edge between any two
            // members and are solved together via OpenMP. Batches are merged
            // across clusters by color index, so independent single-drone
            // clusters all join batch 0.
            std::unordered_map<int, int> cluster_of;
            cluster_of.reserve(solved_ids.size());
            for (int c = 0; c < static_cast<int>(clusters.size()); ++c) {
                for (int drone_id : clusters[c]) cluster_of[drone_id] = c;
            }
            std::vector<std::vector<collision::ConflictEdge>> edges_by_cluster(clusters.size());
            for (const auto& edge : solved_edges) {
                auto it_i = cluster_of.find(edge.drone_i);
                auto it_j = cluster_of.find(edge.drone_j);
                if (it_i != cluster_of.end() && it_j != cluster_of.end() && it_i->second == it_j->second) {
                    edges_by_cluster[it_i->second].push_back(edge);
                }
            }

            std::vector<std::vector<int>> global_batches;  // global_batches[color]
            for (int c = 0; c < static_cast<int>(clusters.size()); ++c) {
                const std::vector<std::vector<int>> colors =
                    collision::greedy_graph_coloring(clusters[c], edges_by_cluster[c]);
                if (colors.size() > global_batches.size()) global_batches.resize(colors.size());
                for (size_t color = 0; color < colors.size(); ++color) {
                    global_batches[color].insert(global_batches[color].end(), colors[color].begin(),
                                                  colors[color].end());
                }
            }

            color_count = static_cast<int>(global_batches.size());
            for (const auto& batch : global_batches) {
#pragma omp parallel for schedule(dynamic)
                for (int b = 0; b < static_cast<int>(batch.size()); ++b) {
                    solve_one_drone(batch[b]);
                }
                // The parallel-for region is a barrier: every thread finishes
                // this color's updates before the next color's batch reads them.
            }
        } else {
            // Legacy (pre-2.7) fallback: parallel across clusters, strictly
            // sequential (ascending drone_id) within a cluster.
            color_count = static_cast<int>(clusters.size());
#pragma omp parallel for schedule(dynamic)
            for (int c = 0; c < static_cast<int>(clusters.size()); ++c) {
                std::vector<int> ordered = clusters[c];
                std::sort(ordered.begin(), ordered.end());  // deterministic priority ordering
                for (int drone_id : ordered) {
                    solve_one_drone(drone_id);
                }
            }
        }

        const double sweep_sec = seconds_since(sweep_start);
        const int candidate_pair_count = static_cast<int>(pairs.size());

        double max_delta = 0.0;
        for (int i = 0; i < num_drones; ++i) {
            for (int idx = map.free_begin; idx < map.free_end; ++idx) {
                max_delta =
                    std::max(max_delta, (workspaces[i].control_points.row(idx) - trust_region_anchor[i].row(idx)).norm());
            }
        }

        // Evaluated for the radius the next step uses if it's accepted.
        IterateEvaluation candidate = evaluate_iterate(std::min(max_trust_region, 2.0 * trust_region));
        const double candidate_min_separation = candidate.min_separation;
        const int candidate_conflict_pairs = static_cast<int>(candidate.conflict_edges.size());
        const double candidate_broad_phase_sec = candidate.broad_phase_sec;  // candidate is moved below
        const double candidate_scan_sec = candidate.scan_sec;
        const bool accepted = better_than(candidate, best);
        // Stall stop: progress means meaningfully less zone
        // intrusion or separation deficit; smoothness alone doesn't count.
        const double stall_tol = config.solver.scp_stall_tol_m;
        const bool progressed =
            accepted && (candidate.intrusion < best.intrusion - stall_tol ||
                         candidate.separation_deficit < best.separation_deficit - stall_tol);
        iterations_without_progress = progressed ? 0 : iterations_without_progress + 1;
        const double step_trust_region = trust_region;
        if (accepted) {
            best = std::move(candidate);
            for (int i = 0; i < num_drones; ++i) best_control_points[i] = workspaces[i].control_points;
            trust_region = std::min(max_trust_region, 2.0 * trust_region);
        } else {
            for (int i = 0; i < num_drones; ++i) {
                if (workspaces[i].fixed) continue;
                workspaces[i].control_points = best_control_points[i];
                rebuild_spline(workspaces[i], duration);
            }
            trust_region *= 0.5;
        }

        // Converged: the QPs moved nothing (a fixed point, whether or not that
        // counted as better), or no better iterate within a trust region
        // that small.
        const bool converged =
            max_delta < config.solver.convergence_tol || trust_region < config.solver.convergence_tol;
        if (progress) {
            ProgressEvent e = progress->event;
            e.iteration = iter + 1;
            e.max_iterations = config.solver.max_scp_iterations;
            e.conflict_pairs = candidate_conflict_pairs;
            e.max_delta_m = max_delta;
            e.min_separation_m = candidate_min_separation;
            e.converged = converged;
            e.step_accepted = accepted;
            e.trust_region_m = step_trust_region;
            e.best_min_separation_m = best.min_separation;
            for (int k = 0; k < 4; ++k) e.qp_tier_counts[k] = tier_counts[k];
            for (int k = 0; k < 3; ++k) e.seed_repair_counts[k] = seed_repair_counts[k];
            e.step_sec = seconds_since(step_start);
            e.sweep_sec = sweep_sec;
            e.broad_phase_sec = candidate_broad_phase_sec;
            e.scan_sec = candidate_scan_sec;
            e.rows_cpu_sec = rows_cpu_sec;
            e.qp_cpu_sec = qp_cpu_sec;
            e.setup_sec = setup_sec;
            e.candidate_pairs = candidate_pair_count;
            e.collision_rows = collision_row_count;
            e.color_count = color_count;
            (*progress->callback)(e);
        }
        if (converged) {
            break;
        }
        if (config.solver.scp_stall_iterations > 0 &&
            iterations_without_progress >= config.solver.scp_stall_iterations) {
            break;  // stalled: the best iterate is returned, as after max_scp_iterations
        }
    }

    SingleStageResult result;
    result.control_points = std::move(best_control_points);
    result.min_separation = best.min_separation;
    return result;
}

// Mega-Cluster Decomposition threshold: transitions longer than this are
// split into sub-stages instead of
// solved as one problem spanning the whole duration.
constexpr double kMegaClusterThresholdS = 15.0;

// Linearly-interpolated position at `waypoint_time` along the straight line
// from `start` to `end` over `total_duration`, sampled at this waypoint's
// fraction of the transition — a reasonable single-drone starting guess for
// a sub-stage boundary, refined pairwise across the whole fleet by
// declash_waypoints() below before it becomes a hard pinned constraint.
//
// Drone_id-parity bow seeding is retired project-wide (see
// trajectory/apf_seeder.hpp's incident writeup): a parity split only ever
// pushes two groups apart, which does not scale to a real N-way squeeze of
// waypoints converging on the same region, and this function's own prior
// version already carried a comment documenting that exact failure (drones
// 4 and 6, both "even", still measured 1.477 m apart at a sub-stage seam
// even with the bow term applied independently to each). Rather than adapt
// APF's full N-body simulation to a one-off boundary-waypoint guess, this
// now returns the plain straight-line interpolation and leaves *all*
// separation work to declash_waypoints()'s pairwise repulsion pass below,
// which already handles "whichever two drones actually end up too close,
// regardless of parity" — unlike a bow, it reacts to the *actual* resulting
// positions rather than guessing a direction upfront.
Eigen::Vector3d bowed_waypoint_position(const Eigen::Vector3d& start, const Eigen::Vector3d& end, double frac) {
    return start + frac * (end - start);
}

// Because a sub-stage boundary is a *hard* pinned position (unlike an
// ordinary interior control point, neither sub-stage's SCP can move it), an
// unsafe interpolated waypoint there cannot be fixed by either sub-stage's
// own collision constraints. This runs a small iterative pairwise repulsion
// pass (standard Gauss-Seidel constraint relaxation) directly on the batch
// of one sub-stage boundary's positions across the whole fleet, splitting
// the deficit between whichever two drones actually end up too close. A
// `fixed` (parked) drone never moves: the other one takes the
// whole deficit, and two fixed drones are left as they are.
void declash_waypoints(std::vector<Eigen::Vector3d>& positions, const std::vector<char>& fixed,
                       double enforced_min_distance) {
    constexpr int kMaxIterations = 8;
    const int num_drones = static_cast<int>(positions.size());
    for (int iter = 0; iter < kMaxIterations; ++iter) {
        bool any_violation = false;
        for (int i = 0; i < num_drones; ++i) {
            for (int j = i + 1; j < num_drones; ++j) {
                if (fixed[i] && fixed[j]) continue;
                const Eigen::Vector3d diff = positions[i] - positions[j];
                const double dist = diff.norm();
                if (dist >= enforced_min_distance) continue;
                any_violation = true;
                const double deficit = enforced_min_distance - dist;
                const Eigen::Vector3d dir = dist > 1e-6 ? Eigen::Vector3d(diff / dist) : Eigen::Vector3d(1.0, 0.0, 0.0);
                if (fixed[j]) {
                    positions[i] += dir * deficit;
                } else if (fixed[i]) {
                    positions[j] -= dir * deficit;
                } else {
                    positions[i] += dir * (deficit / 2.0);
                    positions[j] -= dir * (deficit / 2.0);
                }
            }
        }
        if (!any_violation) break;
    }
}

// Same rationale as SingleStageResult above (an internal diagnostic, not
// the final safety verdict — see that struct's comment), but for a whole
// transition (which may be several mega-cluster-decomposed sub-stages) —
// `min_separation` is the minimum over every sub-stage's own
// SingleStageResult::min_separation.
struct TransitionSolveResult {
    std::vector<DroneTrajectorySolution> trajectories;
    double min_separation = std::numeric_limits<double>::infinity();
};

int substage_count(double duration, const CoreConfig& config) {
    if (duration <= kMegaClusterThresholdS) return 1;
    return std::max(1, static_cast<int>(std::ceil(duration / config.solver.max_substage_duration_s)));
}

int transition_num_control_points(const std::vector<DroneTransitionProblem>& problems, const CoreConfig& config) {
    double d_max = 0.0;
    for (const auto& p : problems) d_max = std::max(d_max, (p.end.position - p.start.position).norm());
    return config.solver.adaptive_control_points ? adaptive_num_control_points(d_max, config.solver.num_control_points_min)
                                                  : config.solver.num_control_points_min;
}

// `progress` is null when no callback is set; otherwise its event carries this
// attempt's fields and gets the sub-stage fields filled in here.
TransitionSolveResult solve_transition_once(const std::vector<DroneTransitionProblem>& problems, double duration,
                                             const CoreConfig& config, IterationProgress* progress = nullptr) {
    const int num_drones = static_cast<int>(problems.size());
    TransitionSolveResult result;
    result.trajectories.resize(num_drones);
    for (int i = 0; i < num_drones; ++i) result.trajectories[i].drone_id = problems[i].drone_id;

    // Computed from the *whole* transition's displacement so every
    // sub-stage gets the same control-point budget an undecomposed solve
    // over the full duration would have used (see solve_single_stage()'s
    // forced_num_control_points doc comment). A single stage computes the
    // same count itself; it is passed so prescribed paths are
    // built with it.
    const int whole_transition_num_control_points = transition_num_control_points(problems, config);

    if (substage_count(duration, config) == 1) {
        if (progress) {
            progress->event.substage = 1;
            progress->event.substage_count = 1;
        }
        std::vector<DroneTransitionProblem> held_problems = problems;
        for (auto& p : held_problems) {
            if (p.prescribed) {
                p.held_control_points = p.prescribed(0.0, duration, duration, whole_transition_num_control_points);
            }
        }
        const SingleStageResult stage_result = solve_single_stage(held_problems, duration, config,
                                                                  whole_transition_num_control_points, progress);
        for (int i = 0; i < num_drones; ++i) {
            result.trajectories[i].stages.push_back(
                DroneTrajectorySolution::Stage{stage_result.control_points[i], duration});
        }
        result.min_separation = stage_result.min_separation;
        return result;
    }

    // Mega-Cluster Decomposition:
    // solving one drone's-worth-of-conflicts QP thousands of times
    // sequentially in a single fleet-spanning Gauss-Seidel cluster is what
    // took a real 300-drone/~50s transition from a projected 15s budget to
    // 6+ minutes of wall time. Splitting into independent sub-stages of
    // ~max_substage_duration_s each keeps each sub-stage's own conflict
    // clusters — and hence its longest sequential chain — bounded to
    // whatever actually conflicts within that shorter time window, letting
    // OpenMP parallelize across sub-stages' many small/singleton clusters
    // again instead of one fleet-wide chain.
    const int num_substages = substage_count(duration, config);
    const double substage_duration = duration / static_cast<double>(num_substages);

    // A prescribed drone's control points per window. Its
    // windows meet at rest, so its waypoint at boundary k is the first
    // control point of window k.
    std::vector<std::vector<Eigen::MatrixXd>> held(num_drones);
    for (int i = 0; i < num_drones; ++i) {
        if (!problems[i].prescribed) continue;
        held[i].resize(num_substages);
        for (int stage = 0; stage < num_substages; ++stage) {
            held[i][stage] = problems[i].prescribed(stage * substage_duration, (stage + 1) * substage_duration, duration,
                                                    whole_transition_num_control_points);
        }
    }

    const double enforced_min_distance =
        config.safety.min_distance_m * (1.0 + config.solver.collision_margin_fraction);

    // waypoints[i][k] is drone i's pinned boundary condition at the end of
    // sub-stage k-1 / start of sub-stage k (k=0 is the transition's real
    // start, k=num_substages is its real end). Built one boundary time k at
    // a time (rather than one drone at a time) so declash_waypoints() can
    // see and correct the whole fleet's positions at that instant together.
    std::vector<std::vector<trajectory::BoundaryConditions>> waypoints(num_drones);
    std::vector<char> fixed(num_drones);
    for (int i = 0; i < num_drones; ++i) {
        fixed[i] = !held[i].empty() || trajectory::is_stationary_hold(problems[i].start, problems[i].end);
    }
    // Where a fixed drone is at boundary k, at rest.
    const auto fixed_waypoint = [&](int i, int k) {
        if (held[i].empty()) return problems[i].start;  // stays at rest on its pad
        trajectory::BoundaryConditions bc;
        bc.position = held[i][k].row(0).transpose();
        bc.velocity = Eigen::Vector3d::Zero();
        bc.acceleration = Eigen::Vector3d::Zero();
        return bc;
    };
    for (int i = 0; i < num_drones; ++i) {
        waypoints[i].resize(num_substages + 1);
        waypoints[i].front() = problems[i].start;
        waypoints[i].back() = problems[i].end;
    }
    for (int k = 1; k < num_substages; ++k) {
        const double waypoint_time = k * substage_duration;
        const double frac = duration > 1e-9 ? waypoint_time / duration : 0.0;
        std::vector<Eigen::Vector3d> positions(num_drones);
        for (int i = 0; i < num_drones; ++i) {
            positions[i] = fixed[i] ? fixed_waypoint(i, k).position
                                    : bowed_waypoint_position(problems[i].start.position, problems[i].end.position, frac);
        }
        declash_waypoints(positions, fixed, enforced_min_distance);
        const double remaining_time = duration - waypoint_time;
        for (int i = 0; i < num_drones; ++i) {
            if (fixed[i]) {
                waypoints[i][k] = fixed_waypoint(i, k);
                continue;
            }
            trajectory::BoundaryConditions wp;
            wp.position = positions[i];
            if (config.safety.keep_out && problems[i].keep_out) {
                // A pinned boundary inside the zone could never be fixed by
                // either sub-stage: move it onto the zone's surface plus a
                // small margin, the shortest way out.
                constexpr double kWaypointMarginM = 0.5;
                const KeepOutZone& zone = *config.safety.keep_out;
                const double d = distance_to_keep_out_region(wp.position, zone);
                if (d < zone.clearance_m + kWaypointMarginM) {
                    const KeepOutPlane plane = keep_out_plane(wp.position, zone, config);
                    const double deficit =
                        zone.clearance_m + kWaypointMarginM - plane.normal.dot(wp.position - plane.anchor);
                    if (deficit > 0.0) wp.position += deficit * plane.normal;
                }
            }
            if (const std::optional<double> floor = drone_floor(problems[i].floor_m, config)) {
                wp.position.z() = std::max(wp.position.z(), *floor);
            }
            wp.velocity = remaining_time > 1e-6
                              ? Eigen::Vector3d((problems[i].end.position - wp.position) / remaining_time)
                              : Eigen::Vector3d::Zero();
            wp.acceleration = Eigen::Vector3d::Zero();
            waypoints[i][k] = wp;
        }
        // One shared velocity at the boundary. With each drone
        // heading straight for its own end point, two drones passing each
        // other at a boundary were exactly at the planning distance there
        // (declash_waypoints() only fixes that instant) but closer just
        // before or after, inside the part of both sub-stages that the pinned
        // boundary state fixes (200_cube: 1.418 m, 0.46 s before a boundary).
        // With one shared velocity the pinned pieces around
        // the boundary move as a rigid copy of the de-clashed positions.
        if (config.solver.shared_substage_velocity) {
            Eigen::Vector3d shared = Eigen::Vector3d::Zero();
            int moving = 0;
            for (int i = 0; i < num_drones; ++i) {
                if (fixed[i]) continue;
                shared += waypoints[i][k].velocity;
                ++moving;
            }
            if (moving > 0) shared /= moving;
            for (int i = 0; i < num_drones; ++i) {
                if (!fixed[i]) waypoints[i][k].velocity = shared;
            }
        }
        // Near the floor, a boundary is passed level; this
        // is per drone, so a drone close to the ground may differ from the
        // shared velocity.
        for (int i = 0; i < num_drones; ++i) {
            if (fixed[i]) continue;
            waypoints[i][k].velocity = floor_safe_velocity(waypoints[i][k].position, waypoints[i][k].velocity, config);
        }
    }

    for (int stage = 0; stage < num_substages; ++stage) {
        std::vector<DroneTransitionProblem> substage_problems(num_drones);
        for (int i = 0; i < num_drones; ++i) {
            substage_problems[i] = DroneTransitionProblem{problems[i].drone_id, waypoints[i][stage],
                                                            waypoints[i][stage + 1], problems[i].keep_out};
            substage_problems[i].floor_m = problems[i].floor_m;
            if (!held[i].empty()) substage_problems[i].held_control_points = held[i][stage];
        }
        if (progress) {
            progress->event.substage = stage + 1;
            progress->event.substage_count = num_substages;
        }
        const SingleStageResult stage_result = solve_single_stage(
            substage_problems, substage_duration, config, whole_transition_num_control_points, progress);
        for (int i = 0; i < num_drones; ++i) {
            result.trajectories[i].stages.push_back(
                DroneTrajectorySolution::Stage{stage_result.control_points[i], substage_duration});
        }
        result.min_separation = std::min(result.min_separation, stage_result.min_separation);
    }
    return result;
}

// Decoupled Continuous Gatekeeper:
// replaces the earlier Slack Rejection Gatekeeper. That mechanism derived its
// safety verdict from SingleStageResult::min_separation — solve_single_stage's
// own best-iterate worst-case tracking — which is built from the *same*
// per-iteration candidate-pair snapshot the QP rows come from, so it could
// under-report the true continuous-time worst case for a pair that drifted
// closer between collocation samples without being re-flagged that same
// iteration (see stage2_nway_conflict_limitation memory /
// apf_seeder.hpp's incident writeup for the root-caused N=40 case this
// caused). This gatekeeper never reads that bookkeeping, OSQP's slack
// variables, or any SCP-iteration candidate-pair snapshot at all: it builds
// a brand-new spline per drone directly from the final converged control
// points and re-detects candidate pairs from scratch via its own
// SpatioTemporalHash, at a fixed (not solver-adaptive) time bucket of
// 1/verification_frequency_hz — >= 100 Hz, i.e. Δt <= 10ms
// requirement, deliberately finer than the SCP loop's own
// adaptive_time_bucketing.
struct ContinuousSafetyResult {
    double min_distance = std::numeric_limits<double>::infinity();
};

// Per unordered drone pair, the single worst sample below the gatekeeper
// floor. Bounded by the number of distinct
// violating pairs, not by duration * frequency, so it keeps
// evaluate_continuous_clearance()'s O(num_drones)-per-instant memory story.
using ViolationsByPair = std::map<std::pair<int, int>, SeparationViolation>;

// `violations`, when non-null, collects every pair closer than
// `report_below_m`, with `time_offset` added to the stage-local sample time
// (so multi-stage transitions report transition-local time).
ContinuousSafetyResult evaluate_continuous_clearance(const std::vector<Eigen::MatrixXd>& control_points,
                                                       const std::vector<int>& drone_ids, double duration,
                                                       double enforced_min_distance,
                                                       double verification_frequency_hz,
                                                       ViolationsByPair* violations = nullptr,
                                                       double report_below_m = 0.0, double time_offset = 0.0) {
    ContinuousSafetyResult report;
    const int num_drones = static_cast<int>(control_points.size());
    if (num_drones < 2 || duration <= 0.0) return report;

    std::vector<QuinticBSpline> splines;
    splines.reserve(num_drones);
    for (const auto& cps : control_points) {
        splines.emplace_back(cps, duration);
    }

    std::map<int, int> drone_index;
    for (int i = 0; i < num_drones; ++i) drone_index[drone_ids[i]] = i;

    const double dt = 1.0 / std::max(1.0, verification_frequency_hz);
    const int num_windows = std::max(1, static_cast<int>(std::ceil(duration / dt)));
    const Eigen::Vector3d margin = Eigen::Vector3d::Constant(enforced_min_distance);

    // One fresh, single-instant hash per time step (window_index always 0)
    // instead of one hash accumulating every drone's position at every
    // 100Hz timestep for the *whole* transition: that accumulation is
    // O(num_drones * duration / dt) with no bound on duration, and was
    // observed to exhaust system memory on a real 300-drone show export
    // whose auto-scaled duration ran into the tens of seconds (2026-09-17;
    // see stage2_nway_conflict_limitation memory). Processing one instant
    // at a time bounds peak memory to O(num_drones), independent of
    // duration or verification_frequency_hz, while still checking every
    // sample at the required >= 100 Hz rate.
    for (int w = 0; w < num_windows; ++w) {
        const double t = std::min(duration, w * dt);
        collision::SpatioTemporalHash hash(enforced_min_distance, dt);
        for (int i = 0; i < num_drones; ++i) {
            const Eigen::Vector3d p = splines[i].position(t);
            hash.insert(collision::DroneWindow{drone_ids[i], 0, p - margin, p + margin});
        }
        for (const auto& pair : hash.find_candidate_pairs()) {
            auto it_i = drone_index.find(pair.drone_i);
            auto it_j = drone_index.find(pair.drone_j);
            if (it_i == drone_index.end() || it_j == drone_index.end()) continue;
            const Eigen::Vector3d p_i = splines[it_i->second].position(t);
            const Eigen::Vector3d p_j = splines[it_j->second].position(t);
            const double d = (p_i - p_j).norm();
            report.min_distance = std::min(report.min_distance, d);

            if (violations && d < report_below_m) {
                const bool i_first = pair.drone_i < pair.drone_j;
                const std::pair<int, int> key = i_first ? std::make_pair(pair.drone_i, pair.drone_j)
                                                        : std::make_pair(pair.drone_j, pair.drone_i);
                auto [it, inserted] = violations->try_emplace(key);
                if (inserted || d < it->second.distance_m) {
                    it->second = SeparationViolation{key.first, key.second, time_offset + t, d,
                                                     i_first ? p_i : p_j, i_first ? p_j : p_i};
                }
            }
        }
    }
    return report;
}

// Staggered Wave Takeoff cross-row race check (see scp_solver.hpp's
// declaration comment for the full rationale). Generalizes
// evaluate_continuous_clearance() above from "one spline per drone over
// [0, duration]" to "a chronological chain of spline stages per drone,
// implicitly starting at a shared time 0" — each per-drone chain's own
// stages are expected to sum to `total_duration` with no gaps (the caller's
// pre-delay/maneuver/post-gap segments always connect end-to-end by
// construction), so a query at global time t within [0, total_duration]
// unambiguously falls into exactly one stage per drone.
double evaluate_worst_case_separation_over_stages(
    const std::vector<std::vector<DroneTrajectorySolution::Stage>>& per_drone_stages,
    const std::vector<int>& drone_ids, double total_duration, double enforced_min_distance,
    double verification_frequency_hz) {
    const int num_drones = static_cast<int>(per_drone_stages.size());
    if (num_drones < 2 || total_duration <= 0.0) return std::numeric_limits<double>::infinity();

    struct Chain {
        std::vector<QuinticBSpline> stage_splines;
        std::vector<double> stage_start_times;  // cumulative, parallel to stage_splines
        double total = 0.0;
    };
    std::vector<Chain> chains(num_drones);
    for (int i = 0; i < num_drones; ++i) {
        double t = 0.0;
        chains[i].stage_splines.reserve(per_drone_stages[i].size());
        chains[i].stage_start_times.reserve(per_drone_stages[i].size());
        for (const auto& stage : per_drone_stages[i]) {
            chains[i].stage_start_times.push_back(t);
            chains[i].stage_splines.emplace_back(stage.control_points, stage.duration);
            t += stage.duration;
        }
        chains[i].total = t;
    }

    auto position_at = [&](int i, double t) -> Eigen::Vector3d {
        const Chain& chain = chains[i];
        if (chain.stage_splines.empty()) return Eigen::Vector3d::Zero();
        const double clamped_t = std::clamp(t, 0.0, chain.total);
        size_t stage = 0;
        for (size_t s = 0; s < chain.stage_start_times.size(); ++s) {
            if (chain.stage_start_times[s] <= clamped_t) {
                stage = s;
            } else {
                break;
            }
        }
        return chain.stage_splines[stage].position(clamped_t - chain.stage_start_times[stage]);
    };

    std::map<int, int> drone_index;
    for (int i = 0; i < num_drones; ++i) drone_index[drone_ids[i]] = i;

    const double dt = 1.0 / std::max(1.0, verification_frequency_hz);
    const int num_windows = std::max(1, static_cast<int>(std::ceil(total_duration / dt)));
    const Eigen::Vector3d margin = Eigen::Vector3d::Constant(enforced_min_distance);

    // Same one-instant-at-a-time, freshly-discarded-hash approach as
    // evaluate_continuous_clearance() above, for the same O(num_drones)
    // peak-memory reason (see its comment).
    double worst = std::numeric_limits<double>::infinity();
    for (int w = 0; w < num_windows; ++w) {
        const double t = std::min(total_duration, w * dt);
        collision::SpatioTemporalHash hash(enforced_min_distance, dt);
        for (int i = 0; i < num_drones; ++i) {
            const Eigen::Vector3d p = position_at(i, t);
            hash.insert(collision::DroneWindow{drone_ids[i], 0, p - margin, p + margin});
        }
        for (const auto& pair : hash.find_candidate_pairs()) {
            auto it_i = drone_index.find(pair.drone_i);
            auto it_j = drone_index.find(pair.drone_j);
            if (it_i == drone_index.end() || it_j == drone_index.end()) continue;
            const double d = (position_at(it_i->second, t) - position_at(it_j->second, t)).norm();
            worst = std::min(worst, d);
        }
    }
    return worst;
}

// After solve_transition_once() settles, this independently re-verifies
// every stage's final splines (see evaluate_continuous_clearance() above)
// against continuous_gatekeeper.min_allowable_distance_m. A rejected
// transition is retried (when auto_retry_with_expansion is set) with a
// longer duration and stronger APF repulsion — both of which give the
// geometry more room to actually separate, unlike simply raising
// w_slack_collision (a bigger penalty on the same geometry cannot conjure
// separation distance that quintic/kinematic limits don't allow within the
// existing duration) — up to max_retry_count attempts before giving up.
//
// Retrying re-derives the *entire* transition from scratch (re-seeding,
// re-decomposing into mega-cluster sub-stages if applicable) rather than
// patching just the offending sub-stage: a mega-cluster sub-stage's pinned
// boundary waypoints are computed from the whole transition's own duration
// (see bowed_waypoint_position()/declash_waypoints() above), so expanding
// one sub-stage's duration in isolation would desync it from its neighbors'
// already-computed boundary conditions. Re-deriving from scratch keeps the
// whole transition internally consistent at the cost of redoing the earlier
// sub-stages' work too, which is acceptable since a gatekeeper rejection is
// meant to be a rare, severely-congested-passage event, not a routine path.
//
// PERFORMANCE SHORTFALL (found 2026-09-17, substantially fixed 2026-09-18):
// the N=40 ring-swap benchmark requires <= 20s on an
// 8-core CPU; measured ~137s (~7x over budget) on the same synthetic
// harness that verified this gatekeeper's correctness fix (see
// stage2_nway_conflict_limitation memory). Ablation had ruled out
// find_inter_sample_minima()'s cutting-plane scan as the dominant cost
// (disabling cutting_plane entirely only dropped it to ~110s) but never
// profiled collision::SpatioTemporalHash::find_candidate_pairs() itself,
// which this SCP loop's outer iteration rebuilds from scratch up to
// max_scp_iterations times per stage, and evaluate_continuous_clearance()
// above rebuilds again once per gatekeeper timestep (i.e. potentially
// thousands of calls for one transition). That function had its own
// separate, real inefficiency (see spatio_temporal_hash.cpp's
// find_candidate_pairs() comment: an O(V) redundant-rescan multiplier per
// drone-window, V being how many voxels a padded bbox spans) which this
// loop's every call paid on top of the genuine candidate-pair cost.
// Re-measured on the same N=40, R=15m ring-swap harness after that fix
// (fresh repro script, original ad-hoc one was job-scratch and gone — see
// stage2_nway_conflict_limitation memory for the recreated harness and
// exact numbers): the ring->antipodal transition itself now solves in
// ~11s, under budget, with no safety regression (worst continuous
// separation 1.73 m, comfortably above the 1.45 m gatekeeper floor). Not
// claimed universally fixed for every scenario — a transition whose own
// D_max/duration forces many dense time windows can still be slow (a
// separate holding-area-departure stage in the same re-test, whose D_max
// and duration were incidentally large, took ~60s) — but the specific,
// named N=40 ring-swap acceptance benchmark this comment used to fail by
// ~7x now passes.
double distance_to_keep_out_region(const Eigen::Vector3d& p, const KeepOutZone& zone) {
    const Eigen::Vector3d lo(zone.lo[0], zone.lo[1], zone.lo[2]);
    const Eigen::Vector3d hi(zone.hi[0], zone.hi[1], zone.hi[2]);
    return (p - p.cwiseMax(lo).cwiseMin(hi)).norm();
}

double max_pinned_lead_time_s(const CoreConfig& config) {
    // Longest stage: a single stage is at most kMegaClusterThresholdS long,
    // a mega-cluster sub-stage at most max_substage_duration_s; every stage
    // has >= num_control_points_min control points, i.e. >= n-5 knot spans.
    const double longest_stage_s = std::max(kMegaClusterThresholdS, config.solver.max_substage_duration_s);
    const int spans = std::max(1, config.solver.num_control_points_min - trajectory::kDegree);
    const double max_knot_span_s = longest_stage_s / spans;
    return 3.0 * max_knot_span_s / 5.0;  // the farther pinned point, p + v*3h/5
}

Eigen::Vector3d floor_safe_velocity(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
                                    const CoreConfig& config) {
    if (!config.safety.altitude_floor_m) return velocity;
    const double clearance = position.z() - *config.safety.altitude_floor_m;
    if (std::abs(velocity.z()) * max_pinned_lead_time_s(config) <= clearance) return velocity;
    Eigen::Vector3d level = velocity;
    level.z() = 0.0;
    return level;
}

std::vector<DroneTrajectorySolution> solve(const std::vector<DroneTransitionProblem>& problems, double duration,
                                            const CoreConfig& config, const ProgressCallback& progress,
                                            SolveStats* stats) {
    const auto& gatekeeper = config.solver.continuous_gatekeeper;
    const double enforced_min_distance =
        config.safety.min_distance_m * (1.0 + config.solver.collision_margin_fraction);

    std::vector<int> drone_ids;
    drone_ids.reserve(problems.size());
    for (const auto& p : problems) drone_ids.push_back(p.drone_id);

    CoreConfig attempt_config = config;
    double attempt_duration = duration;
    std::vector<GatekeeperAttempt> attempt_history;

    // Retries only happen with auto_retry_with_expansion; see the give-up test below.
    const int max_attempts = gatekeeper.auto_retry_with_expansion ? gatekeeper.max_retry_count + 1 : 1;

    for (int attempt = 0;; ++attempt) {
        std::optional<IterationProgress> iteration_progress;
        if (progress) {
            ProgressEvent e;
            e.kind = ProgressEvent::Kind::AttemptStart;
            e.attempt = attempt + 1;
            e.max_attempts = max_attempts;
            e.duration_sec = attempt_duration;
            progress(e);
            e.kind = ProgressEvent::Kind::ScpIteration;
            iteration_progress = IterationProgress{&progress, e};
        }
        TransitionSolveResult result = solve_transition_once(
            problems, attempt_duration, attempt_config, iteration_progress ? &*iteration_progress : nullptr);

        // Violations are collected on every attempt (cheap: bounded by the
        // number of violating pairs) but only the rejected final attempt's
        // set is reported.
        ViolationsByPair violations_by_pair;
        double worst_continuous_distance = std::numeric_limits<double>::infinity();
        double stage_offset = 0.0;
        const size_t num_stages = result.trajectories.empty() ? 0 : result.trajectories[0].stages.size();
        for (size_t stage = 0; stage < num_stages; ++stage) {
            std::vector<Eigen::MatrixXd> stage_control_points;
            stage_control_points.reserve(result.trajectories.size());
            for (const auto& traj : result.trajectories) {
                stage_control_points.push_back(traj.stages[stage].control_points);
            }
            const double stage_duration = result.trajectories[0].stages[stage].duration;
            const ContinuousSafetyResult report = evaluate_continuous_clearance(
                stage_control_points, drone_ids, stage_duration, enforced_min_distance,
                gatekeeper.verification_frequency_hz, &violations_by_pair, gatekeeper.min_allowable_distance_m,
                stage_offset);
            worst_continuous_distance = std::min(worst_continuous_distance, report.min_distance);
            stage_offset += stage_duration;
        }
        // Altitude floor safety net: the QP rows (planned
        // kFloorPlanningMarginM above the floor), the post-QP clamp and the
        // floor-safe boundaries make this hold by construction; a result
        // that still dips below is not accepted. Checked on the control points: by the
        // convex hull property their lowest z bounds the whole path.
        double lowest_control_point_z = std::numeric_limits<double>::infinity();
        for (const auto& traj : result.trajectories) {
            for (const auto& st : traj.stages) {
                lowest_control_point_z = std::min(lowest_control_point_z, st.control_points.col(2).minCoeff());
            }
        }
        constexpr double kFloorToleranceM = 1e-3;
        const bool floor_ok = !config.safety.altitude_floor_m ||
                              lowest_control_point_z >= *config.safety.altitude_floor_m - kFloorToleranceM;

        // Keep-out zone, verified like separation: every show
        // drone sampled at verification_frequency_hz, exact distance to the
        // holding region (the planner's rows are only sampled every 0.25 s).
        KeepOutViolation closest_to_zone;
        closest_to_zone.distance_m = std::numeric_limits<double>::infinity();
        if (config.safety.keep_out) {
            const KeepOutZone& zone = *config.safety.keep_out;
            for (size_t i = 0; i < result.trajectories.size(); ++i) {
                if (!problems[i].keep_out) continue;
                double stage_start = 0.0;
                for (const auto& st : result.trajectories[i].stages) {
                    const trajectory::QuinticBSpline spline(st.control_points, st.duration);
                    const int samples =
                        std::max(2, static_cast<int>(std::ceil(st.duration * gatekeeper.verification_frequency_hz)) + 1);
                    for (int s = 0; s < samples; ++s) {
                        const double t = st.duration * s / (samples - 1);
                        const Eigen::Vector3d p = spline.position(t);
                        const double d = distance_to_keep_out_region(p, zone);
                        if (d < closest_to_zone.distance_m) {
                            closest_to_zone = KeepOutViolation{result.trajectories[i].drone_id, stage_start + t, d, p};
                        }
                    }
                    stage_start += st.duration;
                }
            }
        }
        constexpr double kKeepOutToleranceM = 1e-3;
        const bool zone_ok = !config.safety.keep_out ||
                             closest_to_zone.distance_m >= config.safety.keep_out->clearance_m - kKeepOutToleranceM;
        const bool separation_ok = worst_continuous_distance >= gatekeeper.min_allowable_distance_m;
        const double reported_lowest_z =
            config.safety.altitude_floor_m ? lowest_control_point_z : std::numeric_limits<double>::infinity();
        attempt_history.push_back(
            GatekeeperAttempt{attempt_duration, worst_continuous_distance, floor_ok, zone_ok, reported_lowest_z});
        if (progress) {
            ProgressEvent e;
            e.kind = ProgressEvent::Kind::AttemptEnd;
            e.attempt = attempt + 1;
            e.max_attempts = max_attempts;
            e.duration_sec = attempt_duration;
            e.worst_separation_m = worst_continuous_distance;
            e.required_separation_m = gatekeeper.min_allowable_distance_m;
            e.passed = separation_ok && floor_ok && zone_ok;
            e.separation_ok = separation_ok;
            e.floor_ok = floor_ok;
            e.zone_ok = zone_ok;
            e.lowest_z_m = reported_lowest_z;
            progress(e);
        }

        // Fail fast: a separation miss deeper than min_retry_separation_m is
        // not retried (see ContinuousGatekeeperConfig), nor is anything
        // after the retry budget.
        const bool hopeless = !separation_ok && worst_continuous_distance < gatekeeper.min_retry_separation_m;
        const bool give_up = !gatekeeper.auto_retry_with_expansion || attempt >= gatekeeper.max_retry_count || hopeless;
        if (worst_continuous_distance >= gatekeeper.min_allowable_distance_m && floor_ok && zone_ok) {
            if (stats) {
                stats->attempts = attempt + 1;
                stats->flown_duration_sec = attempt_duration;
                stats->worst_separation_m = worst_continuous_distance;
            }
            return result.trajectories;
        }
        if (give_up &&
            worst_continuous_distance >= gatekeeper.min_allowable_distance_m && floor_ok) {
            // Separation and floor are fine; only the keep-out zone failed.
            throw KeepOutViolationError(
                "Holding-area clearance violated: drone " + std::to_string(closest_to_zone.drone_id) +
                    " comes within " + std::to_string(closest_to_zone.distance_m) + " m of the holding area (required " +
                    std::to_string(config.safety.keep_out->clearance_m) + " m), after " + std::to_string(attempt + 1) +
                    " attempt(s)",
                closest_to_zone, attempt + 1);
        }
        if (give_up &&
            worst_continuous_distance >= gatekeeper.min_allowable_distance_m) {
            // Separation is fine; only the floor failed, on every attempt.
            throw std::runtime_error("Altitude floor violated: a planned path goes down to z = " +
                                     std::to_string(lowest_control_point_z) + " m, below the ground at z = " +
                                     std::to_string(*config.safety.altitude_floor_m) + " m, after " +
                                     std::to_string(attempt + 1) + " attempt(s)");
        }
        if (give_up) {
            SafetyViolationReport report;
            report.worst_separation_m = worst_continuous_distance;
            report.required_separation_m = gatekeeper.min_allowable_distance_m;
            report.enforced_min_distance_m = enforced_min_distance;
            report.verification_frequency_hz = gatekeeper.verification_frequency_hz;
            report.attempts = std::move(attempt_history);
            report.violating_pair_count = static_cast<int>(violations_by_pair.size());
            report.violations.reserve(violations_by_pair.size());
            for (auto& [key, violation] : violations_by_pair) report.violations.push_back(violation);
            std::sort(report.violations.begin(), report.violations.end(),
                      [](const SeparationViolation& a, const SeparationViolation& b) {
                          return a.distance_m < b.distance_m;
                      });
            if (report.violations.size() > static_cast<size_t>(kMaxReportedViolations)) {
                report.violations.resize(kMaxReportedViolations);
            }
            report.rejected_solutions = std::move(result.trajectories);
            // The other checks' failures per attempt, so an attempt rejected
            // with enough separation (floor or zone only) doesn't read as a
            // separation failure.
            std::string other_checks;
            for (size_t a = 0; a < report.attempts.size(); ++a) {
                const GatekeeperAttempt& h = report.attempts[a];
                std::string failed;
                if (!h.floor_ok) failed += "altitude floor (lowest point z = " + std::to_string(h.lowest_z_m) + " m)";
                if (!h.zone_ok) failed += std::string(failed.empty() ? "" : ", ") + "holding-area clearance";
                if (failed.empty()) continue;
                other_checks += "; attempt " + std::to_string(a + 1) + " (worst separation " +
                                std::to_string(h.worst_separation_m) + " m) also failed: " + failed;
            }
            throw SafetyViolationError(
                "CRITICAL: Safety violation detected by 100Hz continuous gatekeeper! (worst separation " +
                    std::to_string(worst_continuous_distance) + " m, required " +
                    std::to_string(gatekeeper.min_allowable_distance_m) + " m, after " +
                    std::to_string(attempt + 1) + " attempt(s)" +
                    (hopeless && gatekeeper.auto_retry_with_expansion && attempt < gatekeeper.max_retry_count
                         ? "; not retried: below " + std::to_string(gatekeeper.min_retry_separation_m) +
                               " m, more time can't fix a miss this deep"
                         : std::string()) +
                    ")" + other_checks,
                std::move(report));
        }
        attempt_duration *= gatekeeper.expansion_factor;
        attempt_config.solver.apf_seeding.k_repulsion *= 2.0;
    }
}

}  // namespace drone_core::optimizer
