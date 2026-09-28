#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "config.hpp"
#include "progress.hpp"
#include "trajectory/quintic_bspline.hpp"

// Cluster-based Gauss-Seidel Sequential Convex Programming optimizer
// (docs/2-phase_2.md Rev 2.9 section 3.3): minimizes integrated
// w_smoothness_snap*snap^2 + w_smoothness_jerk*jerk^2 +
// 1/2*w_slack_collision*sum(s_k^2) subject to kinematic limits (section
// 1.2/3.2) and linearized inter-drone separation constraints, with a
// trust-region step limit. As of Rev 2.7 section 1.8, separation is a soft
// constraint: each collision row gets its own nonnegative slack variable
// s_k instead of a hard bound, so OSQP always has a feasible solution
// regardless of local geometry (the still-hard kinematic-box/trust-region
// rows can still trigger the 2-tier infeasibility fallback in section 1.4,
// just not the collision rows themselves anymore).
//
// Every SCP iteration rebuilds the 4D spatio-temporal candidate-pair index
// and, from it, a conflict graph with one edge per unique drone pair sharing
// a candidate pair (this graph is deliberately NOT filtered by
// cluster_distance_threshold_m or capped by max_cluster_size — section 4
// describes both, but applying either here would let two drones that still
// share a soft collision QP row land in different clusters/colors and race
// on each other's workspace state across OpenMP threads; see
// scp_solver.cpp's build_conflict_edges() for the real-incident writeup).
// Singleton components (no conflict) solve independently in parallel over
// OpenMP. Components with >1 drone are, as of Rev 2.7 section 1.8, split
// into color batches by greedy graph coloring instead of swept with a pure
// ascending-drone-id Gauss-Seidel order: drones sharing a color have no edge
// between them and solve together in parallel, while each color still
// reacts to the previous colors' already-updated solutions this same
// iteration, preserving the anti-stalemate benefit a fully Jacobi (all-
// previous-iteration) sweep lacks. `enable_graph_coloring = false` reverts
// to a pure sequential-within-cluster sweep (still over the same uncapped,
// unfiltered clusters, for the same safety reason).
//
// ADDRESSED KNOWN LIMITATION (Rev 2.7 -> Rev 2.8 -> Rev 2.9): the parity-bow
// seeding this solver used to rely on did not reliably converge to a safe
// separation for real *N-way* simultaneous conflicts (many drones
// converging on the same region at once), as opposed to isolated pairwise
// crossings — confirmed against a real 300-drone show (~692 violating
// pairs) and a synthetic ring-swap stress test, and reproducible regardless
// of enable_graph_coloring, iteration budget, slack weight, or collision
// time-window density (i.e. a seeding/local-optimum problem, not a solver-
// tuning one). Rev 2.8 replaced that seeding with a joint N-body APF
// warm-start simulation (trajectory/apf_seeder.hpp's incident writeup has
// the full root-cause evidence) and added a Slack Rejection Gatekeeper, but
// that gatekeeper did *not* independently catch a residual N=40 case,
// because its worst-case tracking shared the same per-iteration
// candidate-pair bookkeeping the QP rows come from (see SingleStageResult's
// comment below), which could under-report the true continuous-time worst
// case. Rev 2.9 closes that gap from two directions: Adaptive Cutting-Plane
// Collocation (section 1.10, scp_solver.cpp's find_inter_sample_minima())
// inserts extra hard collocation rows exactly at detected near-miss extrema
// so the *optimizer* reacts to them every iteration, and the Decoupled
// Continuous Gatekeeper (section 1.9, below) replaces the Slack Rejection
// Gatekeeper with a verification pass that reuses none of the solver's own
// bookkeeping. solve() can still throw std::runtime_error if a transition
// remains unsafe after the retry budget — callers (pipeline.cpp and,
// through it, the pybind11 binding) must be prepared for that.
//
// Decoupled Continuous Gatekeeper (docs/2-phase_2.md Rev 2.9 section 1.9):
// a converged soft-slack solution (section 1.8) is only ever
// OSQP-*feasible*, never automatically flight-*safe*. After solving,
// `solve()` independently re-verifies the final splines from scratch (see
// scp_solver.cpp's evaluate_continuous_clearance()) — a fresh
// SpatioTemporalHash at continuous_gatekeeper.verification_frequency_hz
// (>= 100 Hz), reading neither OSQP's slack variables nor any SCP-iteration
// candidate-pair snapshot; if the worst real distance found falls under
// continuous_gatekeeper.min_allowable_distance_m, it retries the *whole*
// transition (re-seeding from scratch, including re-decomposing into
// mega-cluster sub-stages if applicable) with a longer duration and
// stronger APF repulsion, up to continuous_gatekeeper.max_retry_count
// times, before throwing.

namespace drone_core::optimizer {

struct DroneTransitionProblem {
    int drone_id = 0;
    trajectory::BoundaryConditions start;
    trajectory::BoundaryConditions end;
};

// Rev 2.6 section 1.7: a transition longer than the mega-cluster threshold
// is decomposed into consecutive Gauss-Seidel sub-stages (each an
// independent SCP problem, chained by pinning a sub-stage's solved end state
// as the next sub-stage's start), so `stages` may hold more than one piece.
struct DroneTrajectorySolution {
    int drone_id = 0;

    struct Stage {
        // Rev 2.5: adaptive control-point count (section 1.1) — rows() may
        // exceed config.solver.num_control_points_min for long transitions.
        // Callers needing the knot vector should derive n from
        // control_points.rows() rather than assuming the configured minimum.
        Eigen::MatrixXd control_points;  // n x 3
        double duration = 0.0;
    };

    std::vector<Stage> stages;  // chronological; size 1 unless mega-cluster-decomposed
};

// Structured diagnostics for a Decoupled Continuous Gatekeeper rejection
// (docs/5-studio_gui.md section 5.1, B1). Every time here is
// transition-local (0 = the start of the rejected transition's synchronized,
// unstaggered solve); pipeline.cpp shifts them to show time.
struct SeparationViolation {
    int drone_a = 0;  // drone_a < drone_b, persistent show drone IDs
    int drone_b = 0;
    double time_sec = 0.0;  // instant of this pair's worst sample
    double distance_m = 0.0;
    Eigen::Vector3d position_a = Eigen::Vector3d::Zero();
    Eigen::Vector3d position_b = Eigen::Vector3d::Zero();
};

struct GatekeeperAttempt {
    double duration_sec = 0.0;
    double worst_separation_m = 0.0;
};

struct SafetyViolationReport {
    double worst_separation_m = 0.0;
    double required_separation_m = 0.0;  // continuous_gatekeeper.min_allowable_distance_m
    double enforced_min_distance_m = 0.0;
    double verification_frequency_hz = 0.0;
    std::vector<GatekeeperAttempt> attempts;  // chronological; the last one is the rejected attempt
    // Final attempt only: each violating pair's single worst sample, sorted
    // by ascending distance and capped at kMaxReportedViolations entries.
    // violating_pair_count is the uncapped count.
    std::vector<SeparationViolation> violations;
    int violating_pair_count = 0;
    // The final attempt's splines, one per problem, in `problems` order.
    std::vector<DroneTrajectorySolution> rejected_solutions;
};

inline constexpr int kMaxReportedViolations = 200;

// Thrown by solve() below. what() keeps the exact pre-B1 message so callers
// matching on the text are unaffected; the report is shared (not copied) so
// the exception stays cheap and nothrow-copyable.
class SafetyViolationError : public std::runtime_error {
public:
    SafetyViolationError(const std::string& message, SafetyViolationReport report)
        : std::runtime_error(message), report_(std::make_shared<const SafetyViolationReport>(std::move(report))) {}
    const SafetyViolationReport& report() const noexcept { return *report_; }

private:
    std::shared_ptr<const SafetyViolationReport> report_;
};

// `duration` and `config` (kinematics/weights/safety/solver, all already
// resolved through the 4-tier / 3-tier fallback in config.hpp) fully
// determine solver behavior; there is no longer a separate options struct.
//
// Throws SafetyViolationError (a std::runtime_error; Rev 2.9 section 1.9's
// Decoupled Continuous Gatekeeper) if this transition is still unsafe after
// its retry budget. pipeline.cpp adds transition context and rethrows it as
// io::PipelineSafetyError, which the binding maps to
// drone_core.SafetyViolationError (a Python RuntimeError subclass).
//
// `progress` (docs/5-studio_gui.md B2), when set, receives AttemptStart /
// ScpIteration / AttemptEnd events with only the attempt, sub-stage and
// iteration fields filled; pipeline.cpp adds the transition fields.
//
// The returned splines are the attempt that passed, which after a retry is
// longer than `duration` (x expansion_factor per retry). `stats`, when set,
// receives that attempt's number and duration: callers must time the
// transition by `flown_duration_sec`, not by `duration` (bug-report P2-03).
// Altitude floor (docs/2-phase_2.md section 1.13). A quintic B-spline stays
// inside the convex hull of its control points, so "every control point at or
// above the floor" guarantees the whole path is. The free control points get a
// hard QP row for it; the pinned boundary ones follow from the boundary
// state: with v = start velocity and a = 0 they sit at p + v*h/5 and
// p + v*3h/5 (h = knot span), mirrored at the end. floor_safe_velocity()
// levels a boundary velocity (drops its vertical part) whenever that lead
// could reach below the floor, using the longest knot span any stage can
// have. Callers must apply it to every non-rest boundary they create.
double max_pinned_lead_time_s(const CoreConfig& config);
Eigen::Vector3d floor_safe_velocity(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
                                    const CoreConfig& config);

struct SolveStats {
    int attempts = 0;                  // 1 = passed first time
    double flown_duration_sec = 0.0;   // duration of the passing attempt
};
std::vector<DroneTrajectorySolution> solve(const std::vector<DroneTransitionProblem>& problems, double duration,
                                            const CoreConfig& config, const ProgressCallback& progress = {},
                                            SolveStats* stats = nullptr);

// Staggered Wave Takeoff cross-row race check (docs/2-phase_2.md section
// 1.7; fixed 2026-09-18): pipeline.cpp re-times each launch row's
// already-solved maneuver (from a single joint solve() call above, which
// assumes every drone starts at the same local time 0) into a different
// absolute-time window by wrapping it with a pre-delay hold and/or
// post-gap hold. That re-timing is never itself checked by solve()'s own
// Decoupled Continuous Gatekeeper (section 1.9), which only ever verified
// the synchronized, local-time-0 solve. This function independently
// re-verifies the REAL, per-drone chained timeline the caller actually
// intends to fly — using the exact same decoupled, no-solver-bookkeeping
// method as evaluate_continuous_clearance() inside scp_solver.cpp,
// generalized from one spline per drone over [0, duration] to a
// chronological chain of spline stages per drone (e.g. hold, maneuver,
// hold), all implicitly starting at a shared time 0 for this check. Returns
// the worst (minimum) point-wise separation found by sampling at
// >= verification_frequency_hz; +infinity if there are fewer than 2 drones
// or `total_duration` is non-positive. Callers should compare the result
// against continuous_gatekeeper.min_allowable_distance_m and react (e.g.
// fall back to an unstaggered build) the same way solve()'s own gatekeeper
// reacts to a rejection.
double evaluate_worst_case_separation_over_stages(
    const std::vector<std::vector<DroneTrajectorySolution::Stage>>& per_drone_stages,
    const std::vector<int>& drone_ids, double total_duration, double enforced_min_distance,
    double verification_frequency_hz);

}  // namespace drone_core::optimizer
