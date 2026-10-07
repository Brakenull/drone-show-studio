#include "io/pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <optional>
#include <stdexcept>

#include "assignment/lap_auction.hpp"
#include "optimizer/scp_solver.hpp"
#include "trajectory/kinematic_limits.hpp"
#include "trajectory/quintic_bspline.hpp"

namespace drone_core::io {

namespace {

constexpr double kPi = 3.14159265358979323846;

Eigen::MatrixXd points_by_index(const Keyframe& kf, int fleet_size) {
    Eigen::MatrixXd Q(fleet_size, 3);
    std::vector<bool> seen(fleet_size, false);
    for (const auto& pt : kf.points) {
        if (pt.index < 0 || pt.index >= fleet_size) {
            throw std::runtime_error("keyframe '" + kf.shape_name + "' has a point index outside [0, fleet_size)");
        }
        Q.row(pt.index) = pt.position.transpose();
        seen[pt.index] = true;
    }
    for (bool s : seen) {
        if (!s) {
            throw std::runtime_error("keyframe '" + kf.shape_name +
                                      "' does not cover every point index in [0, fleet_size)");
        }
    }
    return Q;
}

// Nx3 int matrix of each point's RGB color, indexed the same way as
// points_by_index(). Validity (full index coverage) is already checked by
// points_by_index() on the same keyframe.
Eigen::MatrixXi colors_by_index(const Keyframe& kf, int fleet_size) {
    Eigen::MatrixXi colors(fleet_size, 3);
    for (const auto& pt : kf.points) {
        colors.row(pt.index) = pt.color.transpose();
    }
    return colors;
}

// Staggered Wave Takeoff prepends/appends a
// constant-velocity "hold" segment around a staggered row's real maneuver
// (see run_pipeline()'s use). `state`'s acceleration is always zero in this
// pipeline, so a hold that keeps velocity constant and ends exactly
// `hold_duration` later at `state.position + state.velocity*hold_duration`
// is just seed_control_points() with matching (zero-acceleration) boundary
// conditions at both ends — no bow/collision-avoidance needed for a
// deliberately trivial straight/stationary hold.
Eigen::MatrixXd build_hold_segment_control_points(const trajectory::BoundaryConditions& state, double hold_duration,
                                                   int num_control_points) {
    trajectory::BoundaryConditions hold_end;
    hold_end.position = state.position + state.velocity * hold_duration;
    hold_end.velocity = state.velocity;
    hold_end.acceleration = Eigen::Vector3d::Zero();
    return trajectory::seed_control_points(state, hold_end, hold_duration, num_control_points);
}

// Landing drones first fly to a hover point
// `height` straight above their slot and then all descend vertically
// together. Flown straight onto a ground slot, a drone glided in low and
// sideways (down to 2-6 cm above the ground for metres) and any downward
// push (downwash from the stacked slots above, a gust) put it on the ground
// short of its slot.
//
// The descent (and the takeoff's climb), rest to rest
// and straight up or down by `rise` (negative = down): seed_control_points()
// pins the first and last three control points to the two rest states and
// puts the others on the straight line between them. Its duration is the
// shortest (in 5 % steps from the quintic T_min estimate, no slack) whose
// velocity, acceleration and jerk stay within the per-axis limits the solver
// uses; it's the same up and down.
TrajectorySegment vertical_segment(const Eigen::Vector3d& from, double rise, double t_start, double duration,
                                   int num_control_points, const Eigen::Vector3i& color) {
    trajectory::BoundaryConditions start;
    start.position = from;
    start.velocity = Eigen::Vector3d::Zero();
    start.acceleration = Eigen::Vector3d::Zero();
    trajectory::BoundaryConditions end = start;
    end.position = from + Eigen::Vector3d(0.0, 0.0, rise);
    TrajectorySegment segment;
    segment.start_time_sec = t_start;
    segment.end_time_sec = t_start + duration;
    segment.control_points = trajectory::seed_control_points(start, end, duration, num_control_points);
    segment.knot_vector =
        trajectory::clamped_knot_vector(static_cast<int>(segment.control_points.rows()), trajectory::kDegree, duration);
    segment.color_keyframes = {ColorKeyframe{segment.start_time_sec, color}, ColorKeyframe{segment.end_time_sec, color}};
    return segment;
}

TrajectorySegment landing_descent_segment(const Eigen::Vector3d& slot, double height, double t_start, double duration,
                                          int num_control_points) {
    return vertical_segment(slot + Eigen::Vector3d(0.0, 0.0, height), -height, t_start, duration, num_control_points,
                            Eigen::Vector3i::Zero());
}

double vertical_move_duration(double height, const CoreConfig& config) {
    const double v = trajectory::inscribed_axis_limit(config.kinematics.v_max_mps);
    const double a = trajectory::inscribed_axis_limit(config.kinematics.a_max_mps2);
    const double j = trajectory::inscribed_axis_limit(config.kinematics.j_max_mps3);
    const int num_control_points = config.solver.num_control_points_min;
    double duration = std::max(trajectory::compute_min_transition_time(height, v, a, j, 0.0), 0.1);
    constexpr int kSamples = 200;
    for (int attempt = 0; attempt < 200; ++attempt, duration *= 1.05) {
        const TrajectorySegment s = vertical_segment(Eigen::Vector3d::Zero(), -height, 0.0, duration, num_control_points,
                                                     Eigen::Vector3i::Zero());
        const trajectory::QuinticBSpline spline(s.control_points, duration);
        bool within = true;
        for (int i = 0; i <= kSamples && within; ++i) {
            const double t = duration * i / kSamples;
            within = std::abs(spline.velocity(t).z()) <= v && std::abs(spline.acceleration(t).z()) <= a &&
                     std::abs(spline.jerk(t).z()) <= j;
        }
        if (within) return duration;
    }
    return duration;
}

// Drones making the same vertical move together keep the distance between
// their slots (lockstep groups), so the rule is only used when
// the slots keep the gatekeeper's distance.
bool slots_keep_distance(const Eigen::MatrixXd& slots, double min_distance) {
    for (int a = 0; a < slots.rows(); ++a) {
        for (int b = a + 1; b < slots.rows(); ++b) {
            if ((slots.row(a) - slots.row(b)).norm() < min_distance) return false;
        }
    }
    return true;
}

// ---- Every pad visit vertical ----------------------------------------------

enum class PadKind { kAir, kHover, kOnPad };

// Where a drone is at a transition boundary, relative to the holding area.
struct PadState {
    PadKind kind = PadKind::kAir;
    Eigen::Vector3d pad = Eigen::Vector3d::Zero();  // its slot, when hovering above it or on it
};

// What a parked drone does in one transition (rule table).
enum class PadMove {
    kNone,          // not parked: flown (it may leave or reach a pad, see Leave / Arrive)
    kStay,          // stationary, on its pad or at its hover point
    kDescend,       // prescribed: down from its hover point at the start, then on the pad
    kClimb,         // prescribed: on the pad, up to its hover point at the end
    kDescendClimb,  // prescribed: both, resting on the pad in between
};
enum class Leave { kNone, kFromHover, kPrelude, kOldWay };  // a flown drone leaving a pad
enum class Arrive { kNone, kToHover, kOldWay };             // a flown drone reaching a pad

struct PadPlan {
    PadMove move = PadMove::kNone;
    Leave leave = Leave::kNone;
    Arrive arrive = Arrive::kNone;
    Eigen::Vector3d start = Eigen::Vector3d::Zero();  // the solver problem's boundary positions
    Eigen::Vector3d end = Eigen::Vector3d::Zero();
    Eigen::Vector3d pad = Eigen::Vector3d::Zero();    // the pad it leaves, stays on or reaches
    bool final_descent = false;  // last transition: descends onto its pad after it
    PadState next;               // its state at the end of the transition (after any final descent)

    bool pad_related() const { return move != PadMove::kNone || leave != Leave::kNone || arrive != Arrive::kNone; }
    bool prescribed() const {
        return move == PadMove::kDescend || move == PadMove::kClimb || move == PadMove::kDescendClimb;
    }
};

// A parked drone's prescribed control points for one window [t0, t1] of a
// transition of `duration` (DroneTransitionProblem::prescribed): on its pad,
// or descending from its hover point at the start of the transition and/or
// climbing back to it at the end. Rest-ramp-rest control points (the first
// and last three equal: at rest; six equal pad points between two ramps: a
// real stop on the pad); each ramp gets the fewest control points whose
// vertical velocity, acceleration and jerk stay within the per-axis limits.
// The ramp's points follow the minimum-jerk S-curve 10x^3 - 15x^4 + 6x^5, not
// a straight line: a straight ramp meets the rests with a corner whose jerk
// grows with 1/span^3, and with 20 control points in a 10 s window (150_cone,
// 2026-10-02) it was 4-30 m/s^3 against 2.89, so every mid-show descent and
// climb there was given up. Empty when the window can't hold that.
Eigen::MatrixXd pad_window_control_points(const Eigen::Vector3d& pad, double height, bool descend_at_start,
                                          bool climb_at_end, double t0, double t1, double duration, int n,
                                          const CoreConfig& config) {
    const bool down = descend_at_start && t0 <= 1e-9;
    const bool up = climb_at_end && t1 >= duration - 1e-9;
    if (!down && !up) return pad.transpose().replicate(n, 1);
    const Eigen::Vector3d hover = pad + Eigen::Vector3d(0.0, 0.0, height);
    const double v = trajectory::inscribed_axis_limit(config.kinematics.v_max_mps);
    const double a = trajectory::inscribed_axis_limit(config.kinematics.a_max_mps2);
    const double j = trajectory::inscribed_axis_limit(config.kinematics.j_max_mps3);
    const double window = t1 - t0;
    const auto s_curve = [](double x) { return x * x * x * (10.0 - 15.0 * x + 6.0 * x * x); };
    for (int r = 1;; ++r) {
        const int rest = n - (down ? 3 + r : 0) - (up ? 3 + r : 0);
        if (rest < ((down && up) ? 6 : 3)) return {};
        Eigen::MatrixXd cp(n, 3);
        int row = 0;
        const auto put = [&](const Eigen::Vector3d& p) { cp.row(row++) = p.transpose(); };
        if (down) {
            for (int k = 0; k < 3; ++k) put(hover);
            for (int k = 1; k <= r; ++k) put(hover + (pad - hover) * s_curve(static_cast<double>(k) / (r + 1)));
        }
        for (int k = 0; k < rest; ++k) put(pad);
        if (up) {
            for (int k = 1; k <= r; ++k) put(pad + (hover - pad) * s_curve(static_cast<double>(k) / (r + 1)));
            for (int k = 0; k < 3; ++k) put(hover);
        }
        const trajectory::QuinticBSpline spline(cp, window);
        bool within = true;
        constexpr int kSamples = 200;
        for (int i = 0; i <= kSamples && within; ++i) {
            const double t = window * i / kSamples;
            within = std::abs(spline.velocity(t).z()) <= v && std::abs(spline.acceleration(t).z()) <= a &&
                     std::abs(spline.jerk(t).z()) <= j;
        }
        if (within) return cp;
    }
}

// One drone's vertical range near its pad in one phase of a transition (a
// point when lo == hi). Phases: 0 the first takeoff's climb before the solved
// part, 1 the start, 2 the end, 3 the descent after the last transition.
// `group` > 0: a move several drones make in lockstep (same phase, same
// profile), so their spacing is their pads'; pairs within one group aren't
// checked against each other. Group 0: a drone that stays still.
struct Occupancy {
    int plan = 0;
    int phase = 0;
    Eigen::Vector2d xy = Eigen::Vector2d::Zero();
    double lo = 0.0;
    double hi = 0.0;
    int group = 0;
};

enum OccupancyGroup {
    kStill = 0,
    kDescendGroup = 1,
    kDescendClimbDownGroup = 2,
    kClimbGroup = 3,
    kDescendClimbUpGroup = 4,
    kPreludeGroup = 5,
    kFinalGroup = 6,
    kArriveHoverGroup = 7,
    kLeaveHoverGroup = 8,
};

std::vector<Occupancy> pad_occupancies(const std::vector<PadPlan>& plans, double height) {
    std::vector<Occupancy> out;
    for (int i = 0; i < static_cast<int>(plans.size()); ++i) {
        const PadPlan& p = plans[i];
        if (!p.pad_related()) continue;
        const Eigen::Vector3d hover = p.pad + Eigen::Vector3d(0.0, 0.0, height);
        const auto at = [&](int phase, const Eigen::Vector3d& x, int group) {
            out.push_back({i, phase, x.head<2>(), x.z(), x.z(), group});
        };
        const auto column = [&](int phase, int group) {
            out.push_back({i, phase, p.pad.head<2>(), p.pad.z(), p.pad.z() + height, group});
        };
        switch (p.move) {
            case PadMove::kStay:
                for (int phase = 0; phase < 3; ++phase) at(phase, p.start, kStill);
                if (p.final_descent) column(3, kFinalGroup); else at(3, p.start, kStill);
                break;
            case PadMove::kDescend:
                column(1, kDescendGroup);
                at(2, p.pad, kStill);
                at(3, p.pad, kStill);
                break;
            case PadMove::kClimb:
                at(0, p.pad, kStill);
                at(1, p.pad, kStill);
                column(2, kClimbGroup);
                break;
            case PadMove::kDescendClimb:
                column(1, kDescendClimbDownGroup);
                column(2, kDescendClimbUpGroup);
                break;
            case PadMove::kNone:
                switch (p.leave) {
                    case Leave::kPrelude:
                        column(0, kPreludeGroup);
                        at(1, hover, kLeaveHoverGroup);
                        break;
                    case Leave::kFromHover:
                        at(1, hover, kLeaveHoverGroup);
                        break;
                    case Leave::kOldWay:
                        at(0, p.start, kStill);
                        at(1, p.start, kStill);
                        break;
                    case Leave::kNone:
                        break;
                }
                switch (p.arrive) {
                    case Arrive::kToHover:
                        at(2, hover, kArriveHoverGroup);
                        if (p.final_descent) column(3, kFinalGroup);
                        break;
                    case Arrive::kOldWay:
                        at(2, p.end, kStill);
                        at(3, p.end, kStill);
                        break;
                    case Arrive::kNone:
                        break;
                }
                break;
        }
    }
    return out;
}

double occupancy_distance(const Occupancy& a, const Occupancy& b) {
    const double gap = std::max(0.0, std::max(a.lo, b.lo) - std::min(a.hi, b.hi));
    return std::hypot((a.xy - b.xy).norm(), gap);
}

// Drones whose vertical move (or hover point) comes closer than
// `min_distance` to another pad-related drone in the same phase, outside its
// own lockstep group. When both move, the later one gives way.
std::vector<int> pad_conflicts(const std::vector<PadPlan>& plans, double height, double min_distance) {
    const std::vector<Occupancy> occ = pad_occupancies(plans, height);
    std::vector<int> conflicted;
    for (size_t a = 0; a < occ.size(); ++a) {
        for (size_t b = a + 1; b < occ.size(); ++b) {
            const Occupancy& x = occ[a];
            const Occupancy& y = occ[b];
            if (x.plan == y.plan || x.phase != y.phase) continue;
            if (x.group == kStill && y.group == kStill) continue;  // the slots' own spacing
            if (x.group == y.group) continue;
            if (occupancy_distance(x, y) >= min_distance) continue;
            conflicted.push_back(y.group != kStill ? y.plan : x.plan);
        }
    }
    std::sort(conflicted.begin(), conflicted.end());
    conflicted.erase(std::unique(conflicted.begin(), conflicted.end()), conflicted.end());
    return conflicted;
}

// Gives up a drone's vertical move: fallback, the old way.
// False when it has none left to give up.
bool downgrade_pad_plan(PadPlan& p, bool last) {
    switch (p.move) {
        case PadMove::kDescend:
        case PadMove::kDescendClimb:
            p.move = PadMove::kStay;  // stays at its hover point
            p.end = p.start;
            p.final_descent = last;
            p.next = {last ? PadKind::kOnPad : PadKind::kHover, p.pad};
            return true;
        case PadMove::kClimb:
            p.move = PadMove::kStay;  // stays on its pad, leaves it the old way next time
            p.end = p.start;
            p.next = {PadKind::kOnPad, p.pad};
            return true;
        case PadMove::kStay:
            return false;
        case PadMove::kNone:
            if (p.arrive == Arrive::kToHover) {
                p.arrive = Arrive::kOldWay;
                p.final_descent = false;
                p.end = p.pad;
                p.next = {PadKind::kOnPad, p.pad};
                return true;
            }
            if (p.leave == Leave::kPrelude) {
                p.leave = Leave::kOldWay;
                p.start = p.pad;
                return true;
            }
            return false;
    }
    return false;
}

// The pad-visit rule table for one transition. `state` and `P` are per slot
// (the solver's drone order), `Q`, `target_is_pad` and `next_leaves` per
// target: next_leaves = 1 when the drone parked there leaves in the next
// transition. `first`: the show's first transition (a drone leaving its pad
// climbs before the solved part). `last`: the show's last transition (drones
// ending at a hover point descend after it).
std::vector<PadPlan> plan_pad_moves(const std::vector<PadState>& state, const Eigen::MatrixXd& P,
                                    const Eigen::MatrixXd& Q, const std::vector<int>& assignment,
                                    const std::vector<char>& target_is_pad, const std::vector<int>& next_leaves,
                                    bool first, bool last, double height, double min_distance) {
    const int n = static_cast<int>(state.size());
    const Eigen::Vector3d up(0.0, 0.0, height);
    std::vector<PadPlan> plans(n);
    for (int i = 0; i < n; ++i) {
        const int j = assignment[i];
        PadPlan& p = plans[i];
        const PadState& s = state[i];
        const Eigen::Vector3d q = Q.row(j).transpose();
        p.start = P.row(i).transpose();
        p.end = q;
        p.next = {PadKind::kAir, Eigen::Vector3d::Zero()};
        if (s.kind != PadKind::kAir && target_is_pad[j] && (q - s.pad).norm() < 1e-6) {
            // Stays parked on (or above) its pad.
            p.pad = s.pad;
            const bool leaves_next = !last && next_leaves[j] == 1;
            if (s.kind == PadKind::kHover) {
                p.move = leaves_next ? PadMove::kDescendClimb : PadMove::kDescend;
                p.end = leaves_next ? p.start : s.pad;
                p.next = {leaves_next ? PadKind::kHover : PadKind::kOnPad, s.pad};
            } else if (leaves_next) {
                p.move = PadMove::kClimb;
                p.end = s.pad + up;
                p.next = {PadKind::kHover, s.pad};
            } else {
                p.move = PadMove::kStay;
                p.end = p.start;
                p.next = {PadKind::kOnPad, s.pad};
            }
            continue;
        }
        if (s.kind != PadKind::kAir && target_is_pad[j]) {
            // A move to another pad: prohibited in the slot assignment and
            // never seen; flown the old way, start to end.
            p.pad = q;
            p.leave = Leave::kOldWay;
            p.arrive = Arrive::kOldWay;
            p.next = {PadKind::kOnPad, q};
            continue;
        }
        if (s.kind == PadKind::kHover) {
            p.pad = s.pad;
            p.leave = Leave::kFromHover;
        } else if (s.kind == PadKind::kOnPad) {
            p.pad = s.pad;
            if (first) {
                p.leave = Leave::kPrelude;
                p.start = s.pad + up;
            } else {
                p.leave = Leave::kOldWay;  // its climb was given up earlier
            }
        } else if (target_is_pad[j]) {
            p.pad = q;
            p.arrive = Arrive::kToHover;
            p.end = q + up;
            p.next = {PadKind::kHover, q};
            if (last) {
                p.final_descent = true;
                p.next = {PadKind::kOnPad, q};
            }
        }
    }
    // The column check: give up vertical moves until none comes too close to
    // another pad-related drone.
    for (int round = 0; round <= n; ++round) {
        bool changed = false;
        for (int i : pad_conflicts(plans, height, min_distance)) changed = downgrade_pad_plan(plans[i], last) || changed;
        if (!changed) break;
    }
    return plans;
}

// Whether a prescribed pad move fits every window the solver may use for a
// transition of `duration`: the planned one and every gatekeeper retry's
// longer one (sub-stage windows, `count` control points each).
bool prescribed_fits(const PadPlan& plan, double height, double duration, int count, const CoreConfig& config) {
    const bool down = plan.move == PadMove::kDescend || plan.move == PadMove::kDescendClimb;
    const bool up = plan.move == PadMove::kClimb || plan.move == PadMove::kDescendClimb;
    std::vector<double> totals{duration};
    const ContinuousGatekeeperConfig& gk = config.solver.continuous_gatekeeper;
    if (gk.auto_retry_with_expansion) {
        for (int r = 1; r <= gk.max_retry_count; ++r) totals.push_back(totals.back() * gk.expansion_factor);
    }
    for (double total : totals) {
        const int windows = optimizer::substage_count(total, config);
        for (int w = 0; w < windows; ++w) {
            if (pad_window_control_points(plan.pad, height, down, up, w * total / windows, (w + 1) * total / windows,
                                          total, count, config)
                    .rows() == 0) {
                return false;
            }
        }
    }
    return true;
}

// Gives up prescribed moves that don't fit their windows and vertical moves
// that fail the column check, rebuilding the solver problems after each
// round (`rebuild`), until nothing changes.
void settle_pad_plans(std::vector<PadPlan>& plans, const std::vector<optimizer::DroneTransitionProblem>& problems,
                      const std::function<void()>& rebuild, double height, double duration, double min_distance,
                      bool last, const CoreConfig& config) {
    for (size_t round = 0; round <= plans.size(); ++round) {
        const int count = optimizer::transition_num_control_points(problems, config);
        bool changed = false;
        for (PadPlan& plan : plans) {
            if (plan.prescribed() && !prescribed_fits(plan, height, duration, count, config)) {
                changed = downgrade_pad_plan(plan, last) || changed;
            }
        }
        for (int i : pad_conflicts(plans, height, min_distance)) changed = downgrade_pad_plan(plans[i], last) || changed;
        if (!changed) return;
        rebuild();
    }
}

// The planning distance (min_distance_m plus the collision margin).
double meta_min_distance(const CoreConfig& config) {
    return config.safety.min_distance_m * (1.0 + config.solver.collision_margin_fraction);
}

// A drone waiting at rest at `at` (no vertical move).
TrajectorySegment still_segment(const Eigen::Vector3d& at, double t_start, double duration, int num_control_points,
                                const Eigen::Vector3i& color) {
    return vertical_segment(at, 0.0, t_start, duration, num_control_points, color);
}

// The report of one transition's pad moves (TransitionTiming::pad_moves).
PadMoves count_pad_moves(const std::vector<PadPlan>& plans) {
    PadMoves m;
    for (const PadPlan& p : plans) {
        if (p.arrive == Arrive::kToHover) ++m.parked;
        if (p.move == PadMove::kDescend || p.move == PadMove::kDescendClimb || p.final_descent) ++m.landed;
        if (p.move == PadMove::kClimb || p.move == PadMove::kDescendClimb || p.leave == Leave::kPrelude) ++m.climbed;
        if (p.move == PadMove::kStay && p.start.z() > p.pad.z() + 1e-6) ++m.hovered;
        if (p.leave == Leave::kOldWay) ++m.old_way;
        if (p.arrive == Arrive::kOldWay) ++m.old_way;
    }
    return m;
}

// Every holding area's region, and each drone's takeoff area (the area of
// its takeoff slot). A drone lands, parks and returns on any free pad of any
// area: the assignment picks the nearest.
struct HoldingAreas {
    std::vector<HoldingRegion> regions;
    std::vector<int> takeoff;  // per drone_id

    explicit HoldingAreas(const ProjectData& project)
        : regions(compute_holding_regions(project.metadata.holding_areas)),
          takeoff(compute_takeoff_areas(project.metadata.holding_areas)) {}

    // The area whose region holds `p`, or -1.
    int area_of(const Eigen::Vector3d& p) const {
        for (int k = 0; k < static_cast<int>(regions.size()); ++k) {
            if ((p.array() >= regions[k].lo.array() - 1e-6).all() && (p.array() <= regions[k].hi.array() + 1e-6).all()) {
                return k;
            }
        }
        return -1;
    }
    bool contains(const Eigen::Vector3d& p) const { return area_of(p) >= 0; }

    // How messages name area `k`.
    std::string name(int k) const {
        return regions.size() == 1 ? "the holding area" : "holding area " + std::to_string(k + 1);
    }
};


// The design's own safety settings on top of the resolved config, shared by
// the show and its return paths.
CoreConfig show_config(const ProjectData& project, const CoreConfig& base_config, const HoldingAreas& holding) {
    // Altitude floor: the
    // design's ground, when the file declares one.
    CoreConfig config = base_config;
    config.safety.altitude_floor_m = project.metadata.ground_z_m;
    // Holding-area keep-out zones: the
    // designer's safe distance around each holding region, when declared.
    config.safety.keep_out.clear();
    for (int k = 0; k < static_cast<int>(holding.regions.size()); ++k) {
        const std::optional<double>& clearance = project.metadata.holding_areas[k].show_clearance_m;
        if (!clearance) continue;
        KeepOutZone zone;
        for (int axis = 0; axis < 3; ++axis) {
            zone.lo[axis] = holding.regions[k].lo(axis);
            zone.hi[axis] = holding.regions[k].hi(axis);
        }
        zone.clearance_m = *clearance;
        zone.area = k;
        config.safety.keep_out.push_back(zone);
    }
    return config;
}

// The largest declared safe distance (the output's holding_clearance_m).
std::optional<double> largest_clearance(const CoreConfig& config) {
    std::optional<double> largest;
    for (const KeepOutZone& zone : config.safety.keep_out) largest = std::max(largest.value_or(0.0), zone.clearance_m);
    return largest;
}

// The holding area a drone may enter in a transition (exempt from its
// keep-out zone): the one it starts or ends in, or, for a pad-related drone,
// its pad's (its hover point may be above the region); -1 = none.
int exempt_area(const HoldingAreas& holding, const trajectory::BoundaryConditions& start,
                const trajectory::BoundaryConditions& end, const std::optional<Eigen::Vector3d>& pad) {
    if (pad) return holding.area_of(*pad);
    const int from = holding.area_of(start.position);
    return from >= 0 ? from : holding.area_of(end.position);
}

Eigen::Vector3i lerp_color(const Eigen::Vector3i& c0, const Eigen::Vector3i& c1, double frac) {
    const Eigen::Vector3d result = c0.cast<double>() + frac * (c1.cast<double>() - c0.cast<double>());
    return result.array().round().matrix().cast<int>();
}

}  // namespace

PipelineResult run_pipeline(const ProjectData& project, const CoreConfig& base_config,
                            const ProgressCallback& progress) {
    const int n = project.metadata.fleet_size;
    const HoldingAreas holding(project);
    const CoreConfig config = show_config(project, base_config, holding);
    // A point inside a holding region is a launch slot or a parked drone:
    // drones starting or ending a transition there are exempt from that
    // area's zone.
    auto in_holding_region = [&](const Eigen::Vector3d& p) { return holding.contains(p); };
    // Waiting areas (Phase 1 schema 1.7.0):
    // a target in a waiting region is a spare drone waiting in the air. It
    // arrives at rest (and, staying, is a fixed hold), is left
    // out of the shared formation velocity, and is not a pad (no vertical pad
    // moves). Which waiting slots the spare drones take is chosen per
    // transition among all of them (assign_spec below).
    std::vector<HoldingRegion> waiting_regions;
    for (const WaitingArea& area : project.metadata.waiting_areas) waiting_regions.push_back(compute_waiting_region(area));
    const Eigen::MatrixXd waiting_slots = compute_all_waiting_slots(project.metadata.waiting_areas);
    auto in_waiting_region = [&](const Eigen::Vector3d& p) {
        return std::any_of(waiting_regions.begin(), waiting_regions.end(), [&](const HoldingRegion& r) {
            return (p.array() >= r.lo.array() - 1e-6).all() && (p.array() <= r.hi.array() + 1e-6).all();
        });
    };
    PipelineResult result;
    result.metadata.altitude_floor_m = config.safety.altitude_floor_m;
    result.metadata.holding_clearance_m = largest_clearance(config);
    result.metadata.holding_areas = project.metadata.holding_areas;
    result.metadata.takeoff_area = holding.takeoff;
    result.metadata.fleet_size = n;
    result.metadata.spline_degree = trajectory::kDegree;
    result.metadata.min_distance_enforced_m =
        config.safety.min_distance_m * (1.0 + config.solver.collision_margin_fraction);

    Eigen::MatrixXd P = compute_all_holding_positions(project.metadata.holding_areas);
    std::vector<int> drone_id_by_slot(n);
    for (int i = 0; i < n; ++i) drone_id_by_slot[i] = i;

    // Staggered Wave Takeoff applies only to the very
    // first transition (holding area -> keyframes[0]) — every parked slot's
    // "Launch Row Index" within the launch grid, used to delay farther rows'
    // takeoff by row_index * staggered_wave_delay_s so the whole fleet lifts
    // off in a wave instead of simultaneously (the launch-neighbor collision
    // risk this addresses is worst at t=0, before any separation has had
    // time to develop).
    //
    // FIXED BUG (found 2026-09-17, fixed 2026-09-18): every row's maneuver
    // control points come from ONE joint optimizer::solve() call below that
    // assumes every drone executes its own maneuver starting at the SAME
    // local time 0. The per-row delay applied further down re-times each
    // row's *already solved* maneuver to start later in absolute wall-clock
    // time — that re-timing was never itself re-verified, so a delayed
    // row's real position at absolute time T (that maneuver's solved
    // position at local time T - delay) was never checked against an
    // undelayed row's position at local time T. Confirmed via ablation on a
    // synthetic N=10 ring-swap holding-area departure: enabling staggering
    // measured 1.001 m worst-case separation; disabling it (no other
    // change) raised that to 1.773 m. Fixed below (see build_slot_outcomes'
    // comment): the staggered build is independently re-verified with
    // optimizer::evaluate_worst_case_separation_over_stages() and discarded
    // in favor of the (safe-by-construction) unstaggered build if it fails.
    // See stage2_nway_conflict_limitation memory for the full investigation
    // and re-test results.
    const bool stagger_takeoff = config.solver.enable_staggered_takeoff;
    // Row indices within each area and layer: wave r launches row r of
    // every holding area together.
    const std::vector<int> launch_row_indices =
        stagger_takeoff ? compute_all_holding_row_indices(project.metadata.holding_areas) : std::vector<int>();
    const int max_launch_row =
        launch_row_indices.empty() ? 0 : *std::max_element(launch_row_indices.begin(), launch_row_indices.end());

    // Real physical velocity at the start of the next transition; zero for
    // every drone at the launch pad (rest state).
    Eigen::MatrixXd actual_velocity = Eigen::MatrixXd::Zero(n, 3);

    // LEDs are off while parked in the holding area (not defined by the
    // Phase 1 schema, which has no color for holding_area) — an assumption
    // worth confirming once Phase 1/3 color handling at launch is settled.
    Eigen::MatrixXi actual_color = Eigen::MatrixXi::Zero(n, 3);

    // Cost-function heading vector: initialized to a unit
    // vector derived from heading_offset_deg only for the very first
    // transition (no prior motion to measure an angle from); every
    // subsequent transition uses the real outgoing velocity computed above.
    const double heading_rad = project.metadata.heading_offset_deg * kPi / 180.0;
    const Eigen::Vector2d initial_heading = assignment::initial_heading_velocity(heading_rad);
    Eigen::MatrixXd v_in_xy(n, 2);
    for (int i = 0; i < n; ++i) v_in_xy.row(i) = initial_heading.transpose();

    double t_cursor = 0.0;
    // T_min auto-scaling can push a transition's end
    // time later than the animator's nominal keyframe time_sec; this
    // accumulates that stretch so every later keyframe's nominal timestamp
    // shifts by the same amount, preserving their relative spacing.
    double time_stretch_offset = 0.0;

    const double v_limit_axis = trajectory::inscribed_axis_limit(config.kinematics.v_max_mps);
    const double a_limit_axis = trajectory::inscribed_axis_limit(config.kinematics.a_max_mps2);
    const double j_limit_axis = trajectory::inscribed_axis_limit(config.kinematics.j_max_mps3);

    std::map<int, DroneTrajectory> trajectories_by_drone;
    for (int i = 0; i < n; ++i) trajectories_by_drone[i].drone_id = i;

    // Every pad visit vertical, through a hover point
    // `pad_height` above the pad. Each drone's pad state per slot (every
    // drone starts on its launch pad).
    const double pad_height = std::max(config.solver.landing_approach_height_m, 0.0);
    const bool pad_rule =
        pad_height > 0.0 && slots_keep_distance(P, config.solver.continuous_gatekeeper.min_allowable_distance_m);
    const double enforced_min_distance = result.metadata.min_distance_enforced_m;
    std::vector<PadState> pad_state(n);
    if (pad_rule) {
        for (int i = 0; i < n; ++i) pad_state[i] = {PadKind::kOnPad, P.row(i).transpose()};
    }
    // The slot assignment, with a prohibitive cost for a parked drone moving
    // to another pad (a hop along the ground). With the rule
    // on, transition k+1's assignment is made while setting up k (to know
    // which parked drones leave next) and reused.
    constexpr double kPadChangeCost = 1e4;
    const auto assign_slots = [&](const Eigen::MatrixXd& from, const Eigen::MatrixXd& to, const Eigen::MatrixXd& v_xy,
                                  const std::vector<std::optional<Eigen::Vector3d>>& parked_pad) {
        assignment::AssignmentInput ain;
        ain.P = from;
        ain.Q = to;
        ain.v_in_xy = v_xy;
        ain.w_distance = config.weights.w_distance;
        ain.w_vertical_climb = config.weights.w_vertical_climb;
        ain.w_heading_change = config.weights.w_heading_change;
        Eigen::MatrixXd cost = assignment::build_cost_matrix(ain);
        for (int i = 0; i < static_cast<int>(parked_pad.size()); ++i) {
            if (!parked_pad[i]) continue;
            for (int j = 0; j < to.rows(); ++j) {
                const Eigen::Vector3d q = to.row(j).transpose();
                if (in_holding_region(q) && (q - *parked_pad[i]).norm() > 1e-6) cost(i, j) += kPadChangeCost;
            }
        }
        return assignment::solve_auction(cost);
    };
    std::optional<assignment::AssignmentResult> next_assignment;

    // One entry per transition: holding area -> keyframes[0] -> ... ->
    // keyframes[last], then, when the Phase 1 file has `legs` (schema 1.6.0),
    // the return leg keyframes[last] -> holding
    // area. Without `legs` this is exactly the pre-1.6.0 behavior.
    struct TransitionSpec {
        Eigen::MatrixXd targets;        // row = target slot
        Eigen::MatrixXi target_colors;  // LED color on arrival
        std::string from_name;
        std::string to_name;
        double keyframe_time_sec = 0.0;  // nominal arrival time (keyframe transitions)
        bool is_takeoff = false;         // departs from the holding area: staggered takeoff applies
        bool is_leg = false;             // timed by a leg target instead of the keyframe timeline
        std::optional<double> target_duration_sec;  // leg target; nullopt = Auto (T_min)
        bool ends_at_rest = false;       // Formation Hold / landing (v = 0) vs fly-through
        bool lands = false;              // return leg: `targets` are the slots
    };
    // Every fixed point of the show must already be on or above the floor:
    // the solver can bend paths, not move formation points or launch slots.
    // (The Blender add-on refuses to export such a design; this catches
    // hand-edited or older files.)
    if (config.safety.altitude_floor_m) {
        const double floor_z = *config.safety.altitude_floor_m;
        auto check_below = [&](const Eigen::MatrixXd& points, const std::string& where) {
            for (int i = 0; i < points.rows(); ++i) {
                if (points(i, 2) < floor_z - 1e-9) {
                    throw std::runtime_error(where + " has a point at z = " + std::to_string(points(i, 2)) +
                                             " m, below the ground at z = " + std::to_string(floor_z) + " m");
                }
            }
        };
        check_below(P, "a holding area");
        check_below(waiting_slots, "a waiting area");
        for (const Keyframe& kf : project.keyframes) check_below(points_by_index(kf, n), "keyframe '" + kf.shape_name + "'");
    }
    // Formation points must already keep the safe distance (the add-on
    // refuses to export otherwise): a pinned
    // target inside the zone can't be planned around. Points inside the
    // region itself are parked drones and don't count.
    for (const KeepOutZone& zone : config.safety.keep_out) {
        for (const Keyframe& kf : project.keyframes) {
            const Eigen::MatrixXd pts = points_by_index(kf, n);
            for (int i = 0; i < pts.rows(); ++i) {
                const Eigen::Vector3d p = pts.row(i).transpose();
                if (in_holding_region(p)) continue;
                const double d = optimizer::distance_to_keep_out_region(p, zone);
                if (d < zone.clearance_m - 1e-9) {
                    throw std::runtime_error("keyframe '" + kf.shape_name + "' has a point " + std::to_string(d) +
                                             " m from " + holding.name(zone.area) +
                                             ", closer than the safe distance of " + std::to_string(zone.clearance_m) +
                                             " m");
                }
            }
        }
    }

    const ShowLegs& legs = project.metadata.legs;
    std::vector<TransitionSpec> specs;
    specs.reserve(project.keyframes.size() + 1);
    for (size_t k = 0; k < project.keyframes.size(); ++k) {
        const Keyframe& kf = project.keyframes[k];
        TransitionSpec spec;
        spec.targets = points_by_index(kf, n);
        spec.target_colors = colors_by_index(kf, n);
        spec.from_name = k == 0 ? "holding_area" : project.keyframes[k - 1].shape_name;
        spec.to_name = kf.shape_name;
        spec.keyframe_time_sec = kf.time_sec;
        spec.is_takeoff = (k == 0);
        spec.is_leg = legs.present && k == 0;
        spec.target_duration_sec = spec.is_leg ? legs.takeoff_duration_sec : std::nullopt;
        // The last formation is always held (drones stop), with or without a
        // return leg after it, so the return starts from rest.
        spec.ends_at_rest = (k + 1 == project.keyframes.size());
        specs.push_back(std::move(spec));
    }
    if (legs.present) {
        // Return leg: land on any free slot of any holding area (the auction
        // picks the nearest), LEDs fading to off as they are while parked.
        TransitionSpec ret;
        ret.targets = compute_all_holding_positions(project.metadata.holding_areas);
        ret.target_colors = Eigen::MatrixXi::Zero(n, 3);
        ret.from_name = project.keyframes.back().shape_name;
        ret.to_name = "holding_area";
        ret.is_leg = true;
        ret.target_duration_sec = legs.return_duration_sec;
        ret.ends_at_rest = true;
        ret.lands = true;
        specs.push_back(std::move(ret));
    }

    // The assignment for one transition. A keyframe whose spare drones wait
    // in waiting areas has `spare` targets inside waiting
    // regions (Phase 1's padding, the first waiting slots). Which drones are
    // spare, and where each one goes, is chosen here:
    //  * a drone that has flown takes any free waiting slot (all areas), so
    //    it goes to a nearby one and a drone already waiting keeps its slot;
    //  * a drone still on its pad (it hasn't taken off yet: Phase 1 pads the
    //    first formation with holding slots) may stay on its own pad until a
    //    formation needs it; no other drone may take that pad.
    // The candidates are the formation targets, every waiting slot and the
    // pad of every drone still on one; dummy rows (one per candidate left
    // empty) can take waiting slots and pads, never a formation target. The
    // chosen slots and pads then replace the keyframe's waiting targets in
    // `spec`, LEDs off.
    // A pad column of another drone costs this: never chosen.
    constexpr double kForbiddenCost = 1e6;
    const auto assign_spec = [&](const Eigen::MatrixXd& from, TransitionSpec& spec, const Eigen::MatrixXd& v_xy,
                                 const std::vector<std::optional<Eigen::Vector3d>>& parked_pad) {
        std::vector<int> formation_rows, waiting_rows;
        for (int j = 0; j < spec.targets.rows(); ++j) {
            (in_waiting_region(spec.targets.row(j).transpose()) ? waiting_rows : formation_rows).push_back(j);
        }
        // Drones still on a pad, and that pad.
        std::vector<int> pad_owner;
        std::vector<Eigen::Vector3d> pads;
        for (int i = 0; i < n; ++i) {
            const Eigen::Vector3d at = from.row(i).transpose();
            if (parked_pad[i]) {
                pad_owner.push_back(i);
                pads.push_back(*parked_pad[i]);
            } else if (in_holding_region(at)) {
                pad_owner.push_back(i);
                pads.push_back(at);
            }
        }
        const int slots = static_cast<int>(waiting_slots.rows());
        const int n_pads = static_cast<int>(pads.size());
        if (waiting_rows.empty() || slots + n_pads < static_cast<int>(waiting_rows.size())) {
            return assign_slots(from, spec.targets, v_xy, parked_pad);
        }
        const int n_form = static_cast<int>(formation_rows.size());
        const int pad0 = n_form + slots;  // first pad column
        const int m = pad0 + n_pads;
        Eigen::MatrixXd candidates(m, 3);
        for (int c = 0; c < n_form; ++c) candidates.row(c) = spec.targets.row(formation_rows[c]);
        candidates.middleRows(n_form, slots) = waiting_slots;
        for (int k = 0; k < n_pads; ++k) candidates.row(pad0 + k) = pads[k].transpose();

        assignment::AssignmentInput ain;
        ain.P = Eigen::MatrixXd::Zero(m, 3);
        ain.P.topRows(n) = from;
        ain.Q = candidates;
        ain.v_in_xy = Eigen::MatrixXd::Zero(m, 2);
        ain.v_in_xy.topRows(n) = v_xy;
        ain.w_distance = config.weights.w_distance;
        ain.w_vertical_climb = config.weights.w_vertical_climb;
        ain.w_heading_change = config.weights.w_heading_change;
        Eigen::MatrixXd cost = assignment::build_cost_matrix(ain);
        // A pad column is its own drone's only: staying costs 0.
        for (int i = 0; i < n; ++i) cost.block(i, pad0, 1, n_pads).setConstant(kForbiddenCost);
        for (int k = 0; k < n_pads; ++k) cost(pad_owner[k], pad0 + k) = 0.0;
        for (int i = n; i < m; ++i) {
            cost.row(i).setZero();
            cost.block(i, 0, 1, n_form).setConstant(kForbiddenCost);
        }
        const assignment::AssignmentResult full = assignment::solve_auction(cost);

        assignment::AssignmentResult result;
        result.assignment.assign(n, -1);
        size_t next_waiting = 0;
        for (int i = 0; i < n; ++i) {
            const int c = full.assignment[i];
            if (cost(i, c) >= kForbiddenCost) {
                throw std::logic_error("a drone was assigned another drone's pad");
            }
            if (c < n_form) {
                result.assignment[i] = formation_rows[c];
            } else {
                const int row = waiting_rows.at(next_waiting++);
                spec.targets.row(row) = candidates.row(c);  // a waiting slot, or its own pad
                spec.target_colors.row(row).setZero();      // LEDs off while waiting or parked
                result.assignment[i] = row;
            }
            result.total_cost += cost(i, c);
        }
        if (next_waiting != waiting_rows.size()) {
            throw std::logic_error("a formation target was left without a drone");
        }
        return result;
    };

    for (size_t kf_index = 0; kf_index < specs.size(); ++kf_index) {
        TransitionSpec& spec = specs[kf_index];
        const bool is_final_keyframe = spec.ends_at_rest;
        const std::string& from_keyframe = spec.from_name;

        // Stamps this transition onto every event,
        // including the solver's. Empty (no cost) when there is no callback.
        ProgressCallback transition_progress;
        if (progress) {
            transition_progress = [&, kf_index](const ProgressEvent& solver_event) {
                ProgressEvent e = solver_event;
                e.transition_index = static_cast<int>(kf_index);
                e.transition_count = static_cast<int>(specs.size());
                e.from_keyframe = from_keyframe;
                e.to_keyframe = spec.to_name;
                progress(e);
            };
            ProgressEvent e;
            e.kind = ProgressEvent::Kind::TransitionStart;
            e.show_time_sec = t_cursor;
            transition_progress(e);
        }

        const Eigen::MatrixXd& Q = spec.targets;
        const Eigen::MatrixXi& Q_colors = spec.target_colors;
        const bool last_transition = (kf_index + 1 == specs.size());

        // A target in the holding region is a pad (a drone
        // parking there, or staying parked).
        std::vector<char> target_is_pad(n, 0);
        for (int j = 0; j < n; ++j) target_is_pad[j] = in_holding_region(Q.row(j).transpose()) ? 1 : 0;

        std::vector<std::optional<Eigen::Vector3d>> parked_pad(n);
        if (pad_rule) {
            for (int i = 0; i < n; ++i) {
                if (pad_state[i].kind != PadKind::kAir) parked_pad[i] = pad_state[i].pad;
            }
        }
        const assignment::AssignmentResult assign_result =
            next_assignment ? *next_assignment : assign_spec(P, spec, v_in_xy, parked_pad);
        next_assignment.reset();

        // The transition and its leg start at t_cursor; the solved part after
        // the first takeoff's climb, set below.
        const double t_leg_start = t_cursor;
        // Staggering only delays departure from the holding area (kf_index
        // 0); every drone's own solved maneuver still takes exactly
        // `duration` — the wave adds a per-row wait before/after it, folded
        // into this transition's total span so later keyframes' timing is
        // unaffected in relative terms (same time_stretch_offset mechanism
        // the T_min auto-scaling already relies on). The actual span
        // (and the time_stretch_offset update it feeds) is only known once
        // build_slot_outcomes() below has run and, if the staggered
        // configuration turns out unsafe, fallen back to the unstaggered
        // span — see that block.
        const bool apply_stagger = stagger_takeoff && spec.is_takeoff;

        // Formation Hold (final keyframe): drones stop, matching how the
        // show ends. Fly-Through Waypoint (every other keyframe): drones
        // keep moving through at a fraction of cruising speed instead of
        // braking to a full stop, which is what forcing v=0 at *every*
        // transition boundary was doing before — that left near-zero kinematic
        // slack for
        // collision-avoidance bending, the root cause of the
        // PRIMAL_INFEASIBLE cases earlier versions chased.
        constexpr double kFlyThroughSpeedFraction = 0.5;

        // Shared formation velocity: every
        // drone flying through this keyframe's formation passes its point with
        // the *same* velocity: the mean of the flying drones' unit travel
        // directions times the fly-through speed. Its size shrinks towards 0
        // when the directions disagree. A path's first and last 3 control
        // points are pinned by its boundary state, so near a keyframe the
        // solver can't bend it. With a velocity of its own direction per drone,
        // neighbours arriving from different directions crossed in that pinned
        // part (2026-09-30, 150_cone: 0.979 m in the last 0.7 s, unchanged by
        // longer retries). With one shared velocity the pinned parts move as
        // a rigid copy of the formation, keeping its own spacing.
        // A staggered takeoff arrives at rest: every row but the last hovers
        // at its formation point until the last row arrives (the post-gap
        // hold below, v = 0). With a fly-through end velocity those rows
        // jumped from flying speed to 0 instantly (found 2026-09-30), and the
        // next transition started with some drones hovering and some flying.
        Eigen::Vector3d formation_velocity = Eigen::Vector3d::Zero();
        if (!is_final_keyframe && !apply_stagger) {
            Eigen::Vector3d direction_sum = Eigen::Vector3d::Zero();
            int flying = 0;
            for (int slot = 0; slot < n; ++slot) {
                const Eigen::Vector3d target = Q.row(assign_result.assignment[slot]).transpose();
                // Parking or waiting: arrives at rest (below).
                if (in_holding_region(target) || in_waiting_region(target)) continue;
                const Eigen::Vector3d travel = target - P.row(slot).transpose();
                if (travel.norm() > 1e-6) {
                    direction_sum += travel.normalized();
                    ++flying;
                }
            }
            if (flying > 0) {
                formation_velocity = direction_sum / flying * (kFlyThroughSpeedFraction * config.kinematics.v_max_mps);
            }
            // The rule above only looks at the incoming
            // leg. A formation that arrives moving one way and leaves another
            // way was passed at full fly-through speed along the incoming
            // direction, and all its drones had to brake and turn at once at
            // the start of the next transition (200_cube, Shape_818: 3 m/s
            // north-east, next formation straight up, 0.546 m). With a
            // previous and a next formation, use the velocity of the
            // formation's centre between them instead (Catmull-Rom):
            // (centre_next - centre_prev) / (t_next - t_prev), over airborne
            // points and design keyframe times, capped at the fly-through speed.
            if (config.solver.centered_formation_velocity && kf_index >= 1 && kf_index + 1 < specs.size() &&
                !specs[kf_index + 1].is_leg) {
                const auto airborne_centre = [&](const Eigen::MatrixXd& points, Eigen::Vector3d* centre) {
                    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
                    int count = 0;
                    for (int r = 0; r < points.rows(); ++r) {
                        const Eigen::Vector3d p = points.row(r).transpose();
                        if (in_holding_region(p) || in_waiting_region(p)) continue;
                        sum += p;
                        ++count;
                    }
                    if (count > 0) *centre = sum / count;
                    return count > 0;
                };
                Eigen::Vector3d centre_prev, centre_next;
                const double dt = specs[kf_index + 1].keyframe_time_sec - specs[kf_index - 1].keyframe_time_sec;
                if (dt > 1e-6 && airborne_centre(P, &centre_prev) &&
                    airborne_centre(specs[kf_index + 1].targets, &centre_next)) {
                    formation_velocity = (centre_next - centre_prev) / dt;
                    const double cap = kFlyThroughSpeedFraction * config.kinematics.v_max_mps;
                    if (formation_velocity.norm() > cap) formation_velocity *= cap / formation_velocity.norm();
                }
            }
            // Near the floor, pass through level so neither a path's end nor
            // the next one's start can dip below it. Levelled
            // for the whole formation, so it stays one shared velocity.
            for (int slot = 0; slot < n; ++slot) {
                const Eigen::Vector3d target = Q.row(assign_result.assignment[slot]).transpose();
                if (in_holding_region(target) || in_waiting_region(target)) continue;
                if (optimizer::floor_safe_velocity(target, formation_velocity, config) != formation_velocity) {
                    formation_velocity.z() = 0.0;
                    break;
                }
            }
        }

        // Look one transition ahead (which parked drones leave
        // next), then this transition's pad moves.
        std::vector<int> next_leaves(n, 0);
        if (pad_rule && !last_transition) {
            TransitionSpec& next_spec = specs[kf_index + 1];
            Eigen::MatrixXd next_v_xy = Eigen::MatrixXd::Zero(n, 2);
            std::vector<std::optional<Eigen::Vector3d>> next_parked(n);
            for (int slot = 0; slot < n; ++slot) {
                const int j = assign_result.assignment[slot];
                if (target_is_pad[j]) {
                    next_parked[j] = Q.row(j).transpose();
                } else if (!is_final_keyframe) {
                    next_v_xy.row(j) = formation_velocity.head<2>().transpose();
                }
            }
            next_assignment = assign_spec(Q, next_spec, next_v_xy, next_parked);
            for (int j = 0; j < n; ++j) {
                if (!target_is_pad[j]) continue;
                const int jn = next_assignment->assignment[j];
                next_leaves[j] = in_holding_region(next_spec.targets.row(jn).transpose()) ? 0 : 1;
            }
        }
        std::vector<PadPlan> plans;
        if (pad_rule) {
            plans = plan_pad_moves(pad_state, P, Q, assign_result.assignment, target_is_pad, next_leaves, kf_index == 0,
                                   last_transition, pad_height, enforced_min_distance);
        }
        const auto start_of = [&](int slot) -> Eigen::Vector3d {
            return pad_rule ? plans[slot].start : Eigen::Vector3d(P.row(slot).transpose());
        };
        const auto end_of = [&](int slot) -> Eigen::Vector3d {
            return pad_rule ? plans[slot].end : Eigen::Vector3d(Q.row(assign_result.assignment[slot]).transpose());
        };

        double d_max = 0.0;
        for (int slot = 0; slot < n; ++slot) d_max = std::max(d_max, (end_of(slot) - start_of(slot)).norm());

        const auto any_plan = [&](auto pred) {
            return pad_rule && std::any_of(plans.begin(), plans.end(), pred);
        };
        const double vertical_s = pad_rule ? vertical_move_duration(pad_height, config) : 0.0;
        const double prelude_s =
            any_plan([](const PadPlan& p) { return p.leave == Leave::kPrelude; }) ? vertical_s : 0.0;
        double final_s = any_plan([](const PadPlan& p) { return p.final_descent; }) ? vertical_s : 0.0;
        const double t_start = t_cursor + prelude_s;
        // A keyframe transition ends at its keyframe's time (shifted by any
        // earlier stretch); a leg lasts its target, or just T_min when Auto.
        // Legs always get the T_min floor: "Auto" means the minimum, and a
        // target is flown as max(target, T_min).
        // A leg's target covers its climb before and descent after the solved
        // part too.
        const double nominal_t_end =
            spec.is_leg ? t_start + std::max(spec.target_duration_sec.value_or(0.0) - prelude_s - final_s, 0.0)
                        : spec.keyframe_time_sec + time_stretch_offset;
        const double nominal_duration = std::max(nominal_t_end - t_start, 1e-6);

        double duration = nominal_duration;
        if (config.solver.auto_scale_transition_time || spec.is_leg) {
            const double t_min = trajectory::compute_min_transition_time(
                d_max, v_limit_axis, a_limit_axis, j_limit_axis, config.solver.kinematic_slack_fraction);
            duration = std::max(nominal_duration, t_min);
        }

        std::vector<optimizer::DroneTransitionProblem> problems(n);
        const auto build_problem = [&](int slot) {
            optimizer::DroneTransitionProblem problem;
            problem.drone_id = drone_id_by_slot[slot];
            problem.start.position = start_of(slot);
            problem.start.velocity = actual_velocity.row(slot).transpose();
            problem.start.acceleration = Eigen::Vector3d::Zero();
            problem.end.position = end_of(slot);
            // A holding-area target mid-show is a drone parking (or staying
            // parked): it lands and stops like at the show's end. With the
            // fly-through speed it reached its pad at ~3 m/s sideways and,
            // starting the next transition at that speed, skidded ~2 m into
            // the neighbouring pad (2026-09-29, 150_cone: 0.159 m on every
            // attempt). At rest, a drone that stays parked is also held
            // fixed by the solver. Section
            // 1.28: a drone reaching or staying at a pad or its hover point.
            // A drone reaching or staying in a waiting slot too.
            const bool at_rest_end =
                (pad_rule ? (plans[slot].arrive != Arrive::kNone || plans[slot].move != PadMove::kNone)
                          : in_holding_region(problem.end.position)) ||
                in_waiting_region(problem.end.position);
            problem.end.velocity = is_final_keyframe || at_rest_end ? Eigen::Vector3d::Zero() : formation_velocity;
            problem.end.acceleration = Eigen::Vector3d::Zero();
            // Every drone keeps out of every holding area's zone except the
            // area it takes off from, lands in or is parked in, and a
            // pad-related drone's pad area (its hover point may be above the
            // region).
            problem.keep_out = !config.safety.keep_out.empty();
            problem.keep_out_exempt_area =
                exempt_area(holding, problem.start, problem.end,
                            pad_rule && plans[slot].pad_related() ? std::optional<Eigen::Vector3d>(plans[slot].pad)
                                                                  : std::nullopt);
            if (pad_rule) {
                const PadPlan& plan = plans[slot];
                if (plan.leave == Leave::kFromHover || plan.leave == Leave::kPrelude ||
                    plan.arrive == Arrive::kToHover) {
                    // Its own floor: half the hover height above
                    // its pad, never above its own start or end.
                    problem.floor_m = std::min({plan.pad.z() + 0.5 * pad_height, problem.start.position.z(),
                                                problem.end.position.z()});
                }
                if (plan.prescribed()) {
                    const Eigen::Vector3d pad = plan.pad;
                    const bool down = plan.move == PadMove::kDescend || plan.move == PadMove::kDescendClimb;
                    const bool up = plan.move == PadMove::kClimb || plan.move == PadMove::kDescendClimb;
                    const double height = pad_height;
                    const CoreConfig& cfg = config;
                    problem.prescribed = [pad, down, up, height, &cfg](double t0, double t1, double total, int count) {
                        Eigen::MatrixXd cp = pad_window_control_points(pad, height, down, up, t0, t1, total, count, cfg);
                        if (cp.rows() == 0) throw std::logic_error("a prescribed pad move doesn't fit its window");
                        return cp;
                    };
                }
            }
            problems[slot] = problem;
        };
        for (int slot = 0; slot < n; ++slot) build_problem(slot);

        // A prescribed move must fit every window the solver may
        // use, for the planned duration and every retry's longer one; a move
        // that doesn't is given up (the drone stays at its hover point or on
        // its pad), then the column check runs again.
        if (pad_rule) {
            settle_pad_plans(
                plans, problems, [&] { for (int slot = 0; slot < n; ++slot) build_problem(slot); }, pad_height, duration,
                enforced_min_distance, last_transition, config);
            if (final_s == 0.0 && any_plan([](const PadPlan& p) { return p.final_descent; })) final_s = vertical_s;
        }

        std::vector<optimizer::DroneTrajectorySolution> solutions;
        optimizer::SolveStats solve_stats;
        try {
            solutions = optimizer::solve(problems, duration, config, transition_progress, &solve_stats);
        } catch (const optimizer::SafetyViolationError& e) {
            // Put the rejection in show context
            // (which transition, show time, the rejected splines next to the
            // transitions that did pass) so a viewer can replay it.
            const optimizer::SafetyViolationReport& report = e.report();
            TransitionSafetyFailure failure;
            failure.solver = report;
            failure.transition_index = static_cast<int>(kf_index);
            failure.from_keyframe = from_keyframe;
            failure.to_keyframe = spec.to_name;
            failure.transition_start_time_sec = t_leg_start;
            failure.transition_duration_sec =
                (report.attempts.empty() ? duration : report.attempts.back().duration_sec) + prelude_s;
            failure.metadata = result.metadata;
            failure.metadata.total_duration_sec = t_leg_start + failure.transition_duration_sec;

            failure.completed_trajectories.reserve(trajectories_by_drone.size());
            for (const auto& [drone_id, traj] : trajectories_by_drone) failure.completed_trajectories.push_back(traj);

            std::map<int, DroneTrajectory> rejected_by_drone;
            for (int slot = 0; slot < n && slot < static_cast<int>(report.rejected_solutions.size()); ++slot) {
                const int target_slot = assign_result.assignment[slot];
                const int drone_id = drone_id_by_slot[slot];
                DroneTrajectory& traj = rejected_by_drone[drone_id];
                traj.drone_id = drone_id;
                int segment_index = static_cast<int>(trajectories_by_drone[drone_id].segments.size());
                if (prelude_s > 0.0) {  // the first takeoff's climb comes first
                    TrajectorySegment first =
                        plans[slot].leave == Leave::kPrelude
                            ? vertical_segment(plans[slot].pad, pad_height, t_leg_start, prelude_s,
                                               config.solver.num_control_points_min, actual_color.row(slot))
                            : still_segment(P.row(slot).transpose(), t_leg_start, prelude_s,
                                            config.solver.num_control_points_min, actual_color.row(slot));
                    first.segment_index = segment_index++;
                    traj.segments.push_back(std::move(first));
                }
                double stage_t_start = t_start;
                for (const auto& stage : report.rejected_solutions[slot].stages) {
                    const double stage_t_end = stage_t_start + stage.duration;
                    const double frac_start = (stage_t_start - t_start) / failure.transition_duration_sec;
                    const double frac_end = (stage_t_end - t_start) / failure.transition_duration_sec;
                    TrajectorySegment segment;
                    segment.segment_index = segment_index++;
                    segment.start_time_sec = stage_t_start;
                    segment.end_time_sec = stage_t_end;
                    segment.control_points = stage.control_points;
                    segment.knot_vector = trajectory::clamped_knot_vector(
                        static_cast<int>(stage.control_points.rows()), trajectory::kDegree, stage.duration);
                    segment.color_keyframes = {
                        ColorKeyframe{stage_t_start,
                                      lerp_color(actual_color.row(slot), Q_colors.row(target_slot), frac_start)},
                        ColorKeyframe{stage_t_end, lerp_color(actual_color.row(slot), Q_colors.row(target_slot), frac_end)},
                    };
                    traj.segments.push_back(std::move(segment));
                    stage_t_start = stage_t_end;
                }
            }
            failure.rejected_trajectories.reserve(rejected_by_drone.size());
            for (auto& [drone_id, traj] : rejected_by_drone) failure.rejected_trajectories.push_back(std::move(traj));
            // Now carried as rejected_trajectories; don't keep two copies.
            failure.solver.rejected_solutions.clear();

            throw PipelineSafetyError(e.what(), std::move(failure));
        } catch (const optimizer::KeepOutViolationError& e) {
            const optimizer::KeepOutViolation& v = e.violation();
            throw std::runtime_error(
                "Holding-area clearance violated in transition " + std::to_string(kf_index) + " (" + from_keyframe +
                " -> " + spec.to_name + "): drone " + std::to_string(v.drone_id) + " comes within " +
                std::to_string(v.distance_m) + " m of " + holding.name(v.area) + " at show time " +
                std::to_string(t_start + v.time_sec) + " s at (" + std::to_string(v.position.x()) + ", " + std::to_string(v.position.y()) + ", " + std::to_string(v.position.z()) + ") (required " + std::to_string(v.clearance_m) +
                " m), after " + std::to_string(e.attempts()) +
                " attempt(s)");
        }

        // `solutions` is the attempt that passed. After a
        // gatekeeper retry it is longer than `duration`, and everything below
        // (end time, next transition's start, LED fade, staggered-launch holds
        // and their re-check, leg times) must follow what is actually flown.
        const double flown_duration = solve_stats.flown_duration_sec;

        // Per-slot output of build_slot_outcomes() below, held before
        // committing to trajectories_by_drone so the Staggered Wave Takeoff
        // race check (see build_slot_outcomes' doc comment) can build,
        // verify, and if necessary discard a candidate build without any
        // pipeline-level state mutation.
        struct SlotOutcome {
            std::vector<TrajectorySegment> segments;  // this transition's new segments, in emission order
            Eigen::Vector3d next_velocity = Eigen::Vector3d::Zero();
            Eigen::Vector3i next_color = Eigen::Vector3i::Zero();
        };

        // Builds every slot's segment list for this transition under a given
        // staggering choice, without mutating any pipeline-level state.
        //
        // Staggered Wave Takeoff cross-row race (previously KNOWN BUG, found
        // 2026-09-17, fixed 2026-09-18): `solutions` above comes from ONE
        // joint optimizer::solve() call that assumes every drone executes
        // its own maneuver starting at the same local time 0. Wrapping a
        // row's solved maneuver in a pre-delay/post-gap hold (use_stagger =
        // true) re-times it into a different absolute-time window without
        // solve() ever having verified pairwise separation across that
        // shift. The fix is below, where this lambda's staggered output is
        // independently re-checked and, if unsafe, discarded in favor of
        // this same lambda's use_stagger=false output — which needs no
        // separate re-check because it is exactly the synchronized,
        // local-time-0 configuration optimizer::solve()'s own Decoupled
        // Continuous Gatekeeper already verified before returning
        // `solutions`, i.e. it is safe by construction, not just observed
        // safe on one ablation.
        auto build_slot_outcomes = [&](bool use_stagger, double* out_t_end) {
            std::vector<SlotOutcome> outcomes(n);
            const double this_stagger_span_s = use_stagger ? max_launch_row * config.solver.staggered_wave_delay_s : 0.0;
            const double this_t_end = t_start + flown_duration + this_stagger_span_s;
            if (out_t_end) *out_t_end = this_t_end;

            for (int slot = 0; slot < n; ++slot) {
                const int target_slot = assign_result.assignment[slot];
                SlotOutcome outcome;
                double stage_t_start = t_start;

                // rows farther from the front of the
                // launch grid wait `row_index * staggered_wave_delay_s`
                // before starting their real maneuver.
                if (use_stagger) {
                    const int row = launch_row_indices[slot];
                    const double pre_delay_s = row * config.solver.staggered_wave_delay_s;
                    if (pre_delay_s > 1e-9) {
                        const trajectory::BoundaryConditions& start = problems[slot].start;
                        TrajectorySegment hold;
                        hold.start_time_sec = stage_t_start;
                        hold.end_time_sec = stage_t_start + pre_delay_s;
                        hold.control_points = build_hold_segment_control_points(start, pre_delay_s,
                                                                                 config.solver.num_control_points_min);
                        hold.knot_vector = trajectory::clamped_knot_vector(
                            static_cast<int>(hold.control_points.rows()), trajectory::kDegree, pre_delay_s);
                        hold.color_keyframes = {ColorKeyframe{hold.start_time_sec, actual_color.row(slot)},
                                                 ColorKeyframe{hold.end_time_sec, actual_color.row(slot)}};
                        outcome.segments.push_back(std::move(hold));
                        stage_t_start += pre_delay_s;
                    }
                }

                // a transition longer than the
                // mega-cluster threshold comes back as multiple chained
                // sub-stages — emit one TrajectorySegment per sub-stage,
                // chained start-to-end. `maneuver_t_start` (rather than
                // t_start) anchors the color-lerp fraction so a staggered
                // pre-delay hold doesn't shift the real maneuver's color
                // interpolation.
                const double maneuver_t_start = stage_t_start;
                const auto& stages = solutions[slot].stages;
                for (size_t stage_idx = 0; stage_idx < stages.size(); ++stage_idx) {
                    const auto& stage = stages[stage_idx];
                    const double stage_t_end = stage_t_start + stage.duration;
                    const int num_control_points = static_cast<int>(stage.control_points.rows());
                    const Eigen::VectorXd knot_vector =
                        trajectory::clamped_knot_vector(num_control_points, trajectory::kDegree, stage.duration);

                    const double frac_start = (stage_t_start - maneuver_t_start) / flown_duration;
                    const double frac_end = (stage_t_end - maneuver_t_start) / flown_duration;

                    TrajectorySegment segment;
                    segment.start_time_sec = stage_t_start;
                    segment.end_time_sec = stage_t_end;
                    segment.knot_vector = knot_vector;
                    segment.control_points = stage.control_points;
                    segment.color_keyframes = {
                        ColorKeyframe{stage_t_start,
                                      lerp_color(actual_color.row(slot), Q_colors.row(target_slot), frac_start)},
                        ColorKeyframe{stage_t_end, lerp_color(actual_color.row(slot), Q_colors.row(target_slot), frac_end)},
                    };
                    outcome.segments.push_back(std::move(segment));

                    if (stage_idx + 1 == stages.size()) {
                        const trajectory::QuinticBSpline spline(stage.control_points, stage.duration);
                        outcome.next_velocity = spline.velocity(stage.duration);
                    }
                    stage_t_start = stage_t_end;
                }

                // Rows that departed earlier finish their own maneuver
                // before this_t_end; they hover at their formation slot
                // (v=0) until the slowest (highest-row) wave catches up, so
                // every slot is simultaneously at rest at Q when the next
                // transition starts.
                if (use_stagger) {
                    const double post_gap_s = this_t_end - stage_t_start;
                    if (post_gap_s > 1e-9) {
                        trajectory::BoundaryConditions rest;
                        rest.position = problems[slot].end.position;  // its pad's hover point when parking
                        rest.velocity = Eigen::Vector3d::Zero();
                        rest.acceleration = Eigen::Vector3d::Zero();
                        TrajectorySegment hold;
                        hold.start_time_sec = stage_t_start;
                        hold.end_time_sec = this_t_end;
                        hold.control_points = build_hold_segment_control_points(rest, post_gap_s,
                                                                                 config.solver.num_control_points_min);
                        hold.knot_vector = trajectory::clamped_knot_vector(
                            static_cast<int>(hold.control_points.rows()), trajectory::kDegree, post_gap_s);
                        hold.color_keyframes = {ColorKeyframe{hold.start_time_sec, Q_colors.row(target_slot)},
                                                 ColorKeyframe{hold.end_time_sec, Q_colors.row(target_slot)}};
                        outcome.segments.push_back(std::move(hold));
                        outcome.next_velocity = Eigen::Vector3d::Zero();
                    }
                }

                outcome.next_color = Q_colors.row(target_slot);
                outcomes[slot] = std::move(outcome);
            }
            return outcomes;
        };

        double t_end = 0.0;
        std::vector<SlotOutcome> outcomes = build_slot_outcomes(apply_stagger, &t_end);

        if (apply_stagger) {
            std::vector<std::vector<optimizer::DroneTrajectorySolution::Stage>> per_drone_stages(n);
            for (int slot = 0; slot < n; ++slot) {
                per_drone_stages[slot].reserve(outcomes[slot].segments.size());
                for (const auto& seg : outcomes[slot].segments) {
                    per_drone_stages[slot].push_back(optimizer::DroneTrajectorySolution::Stage{
                        seg.control_points, seg.end_time_sec - seg.start_time_sec});
                }
            }
            const double enforced_min_distance =
                config.safety.min_distance_m * (1.0 + config.solver.collision_margin_fraction);
            const double worst_staggered_separation = optimizer::evaluate_worst_case_separation_over_stages(
                per_drone_stages, drone_id_by_slot, t_end - t_start, enforced_min_distance,
                config.solver.continuous_gatekeeper.verification_frequency_hz);
            if (worst_staggered_separation < config.solver.continuous_gatekeeper.min_allowable_distance_m) {
                // Staggering desynced this transition into an unsafe
                // configuration — fall back to the synchronized build, safe
                // by construction (see build_slot_outcomes' comment).
                outcomes = build_slot_outcomes(false, &t_end);
            }
        }
        // After the last transition, every drone that ended it
        // at a hover point descends onto its pad, all together (the others
        // stay where they are). Every drone is at rest at t_end (the last
        // transition is held and never staggered).
        if (final_s > 0.0) {
            for (int slot = 0; slot < n; ++slot) {
                const Eigen::Vector3i color = outcomes[slot].next_color;
                outcomes[slot].segments.push_back(
                    plans[slot].final_descent
                        ? landing_descent_segment(plans[slot].pad, pad_height, t_end, final_s,
                                                  config.solver.num_control_points_min)
                        : still_segment(problems[slot].end.position, t_end, final_s,
                                        config.solver.num_control_points_min, color));
                outcomes[slot].next_velocity = Eigen::Vector3d::Zero();
            }
            t_end += final_s;
        }
        // Before the first transition's solved part (and any
        // staggered wait, now at the hover points), the drones leaving their
        // pads climb off them, all together; the others wait. Rigid, so safe
        // where the column check passed; outside the stagger re-check on
        // purpose.
        if (prelude_s > 0.0) {
            for (int slot = 0; slot < n; ++slot) {
                auto& segments = outcomes[slot].segments;
                segments.insert(segments.begin(),
                                plans[slot].leave == Leave::kPrelude
                                    ? vertical_segment(plans[slot].pad, pad_height, t_leg_start, prelude_s,
                                                       config.solver.num_control_points_min, actual_color.row(slot))
                                    : still_segment(P.row(slot).transpose(), t_leg_start, prelude_s,
                                                    config.solver.num_control_points_min, actual_color.row(slot)));
            }
        }
        if (spec.is_leg && spec.is_takeoff) {
            // The show's own timeline starts when the takeoff reaches
            // keyframes[0]; later keyframes keep their spacing from it.
            time_stretch_offset = t_end - spec.keyframe_time_sec;
        } else if (!spec.is_leg) {
            time_stretch_offset += (t_end - nominal_t_end);
        }
        TransitionTiming transition_timing;
        transition_timing.index = static_cast<int>(kf_index);
        transition_timing.from_keyframe = spec.from_name;
        transition_timing.to_keyframe = spec.to_name;
        transition_timing.start_time_sec = t_leg_start;
        transition_timing.end_time_sec = t_end;
        transition_timing.planned_duration_sec = duration + prelude_s + final_s;
        transition_timing.flown_duration_sec = flown_duration + prelude_s + final_s;
        transition_timing.attempts = solve_stats.attempts;
        if (pad_rule) transition_timing.pad_moves = count_pad_moves(plans);
        result.metadata.transitions.push_back(std::move(transition_timing));

        if (spec.is_leg) {
            LegTiming timing;
            timing.start_time_sec = t_leg_start;
            timing.end_time_sec = t_end;
            timing.target_duration_sec = spec.target_duration_sec;
            (spec.is_takeoff ? result.metadata.takeoff_leg : result.metadata.return_leg) = timing;
        }

        Eigen::MatrixXd next_actual_velocity = Eigen::MatrixXd::Zero(n, 3);
        Eigen::MatrixXi next_actual_color = Eigen::MatrixXi::Zero(n, 3);
        std::vector<int> next_drone_id_by_slot(n, -1);

        for (int slot = 0; slot < n; ++slot) {
            const int target_slot = assign_result.assignment[slot];
            const int drone_id = drone_id_by_slot[slot];
            DroneTrajectory& traj = trajectories_by_drone[drone_id];

            for (auto& segment : outcomes[slot].segments) {
                segment.segment_index = static_cast<int>(traj.segments.size());
                traj.segments.push_back(std::move(segment));
            }
            next_actual_velocity.row(target_slot) = outcomes[slot].next_velocity.transpose();
            next_actual_color.row(target_slot) = outcomes[slot].next_color.transpose();
            next_drone_id_by_slot[target_slot] = drone_id;
        }

        // Where every drone really is now (at a hover point, or
        // on its pad after a descent), and its pad state.
        if (pad_rule) {
            std::vector<PadState> next_state(n);
            Eigen::MatrixXd next_P(n, 3);
            for (int slot = 0; slot < n; ++slot) {
                const int j = assign_result.assignment[slot];
                next_state[j] = plans[slot].next;
                next_P.row(j) = (plans[slot].final_descent ? plans[slot].pad : problems[slot].end.position).transpose();
            }
            pad_state = std::move(next_state);
            P = next_P;
        } else {
            P = spec.targets;
        }
        v_in_xy = next_actual_velocity.leftCols(2);
        actual_velocity = next_actual_velocity;
        actual_color = next_actual_color;
        drone_id_by_slot = std::move(next_drone_id_by_slot);
        t_cursor = t_end;

        if (transition_progress) {
            ProgressEvent e;
            e.kind = ProgressEvent::Kind::TransitionEnd;
            e.show_time_sec = t_end;
            transition_progress(e);
        }
    }

    // The actual, possibly auto-scaled total show duration, not the nominal
    // sum of Phase 1's keyframe timestamps.
    result.metadata.total_duration_sec = t_cursor;

    result.trajectories.reserve(trajectories_by_drone.size());
    for (auto& [drone_id, traj] : trajectories_by_drone) {
        result.trajectories.push_back(std::move(traj));
    }
    return result;
}

ReturnPathResult plan_return_path(const ProjectData& project, const CoreConfig& base_config,
                                  const std::vector<DroneTrajectory>& show,
                                  const std::vector<TransitionTiming>& transitions, int keyframe_index,
                                  std::optional<double> target_duration_sec, const ProgressCallback& progress,
                                  std::optional<double> abort_time_sec, bool estimate_only) {
    const int n = project.metadata.fleet_size;
    const int keyframe_count = static_cast<int>(project.keyframes.size());
    if (keyframe_index < 0 || keyframe_index >= keyframe_count) {
        throw std::runtime_error("formation index " + std::to_string(keyframe_index) + " is outside [0, " +
                                 std::to_string(keyframe_count) + ")");
    }
    const std::string& from_name = project.keyframes[keyframe_index].shape_name;
    // Transition k is the one into keyframes[k] (0 = takeoff), in the show
    // result as in run_pipeline().
    if (keyframe_index >= static_cast<int>(transitions.size()) ||
        transitions[keyframe_index].to_keyframe != from_name) {
        throw std::runtime_error("the show result's transitions don't match the Phase 1 file (no transition " +
                                 std::to_string(keyframe_index) + " into '" + from_name + "')");
    }
    const TransitionTiming& into = transitions[keyframe_index];
    const bool inside = abort_time_sec.has_value();
    if (inside && !(into.start_time_sec < *abort_time_sec && *abort_time_sec < into.end_time_sec)) {
        throw std::runtime_error("abort time " + std::to_string(*abort_time_sec) +
                                 " s is not inside the transition into '" + from_name + "' (" +
                                 std::to_string(into.start_time_sec) + " to " + std::to_string(into.end_time_sec) +
                                 " s)");
    }
    const double abort_time = inside ? *abort_time_sec : into.end_time_sec;
    if (static_cast<int>(show.size()) != n) {
        throw std::runtime_error("the show result has " + std::to_string(show.size()) + " drones, the Phase 1 file " +
                                 std::to_string(n));
    }

    const HoldingAreas holding(project);
    const CoreConfig config = show_config(project, base_config, holding);

    // Each drone's state and LED color at the abort time, from its own
    // spline: the segment that ends there (every transition ends at a
    // segment boundary), else the one that contains it. At a formation the
    // fleet is at rest; inside a transition it keeps its acceleration too.
    Eigen::MatrixXd P(n, 3);
    Eigen::MatrixXd start_velocity(n, 3);
    Eigen::MatrixXd start_acceleration = Eigen::MatrixXd::Zero(n, 3);
    Eigen::MatrixXi start_color(n, 3);
    std::vector<int> drone_id_by_slot(n);
    constexpr double kTimeTolerance = 1e-6;
    for (int slot = 0; slot < n; ++slot) {
        const DroneTrajectory& traj = show[slot];
        drone_id_by_slot[slot] = traj.drone_id;
        const TrajectorySegment* found = nullptr;
        for (const auto& seg : traj.segments) {
            if (std::abs(seg.end_time_sec - abort_time) <= kTimeTolerance) {
                found = &seg;
                break;
            }
            if (!found && seg.start_time_sec <= abort_time && abort_time <= seg.end_time_sec) found = &seg;
        }
        if (!found) {
            throw std::runtime_error("drone " + std::to_string(traj.drone_id) + " has no segment at show time " +
                                     std::to_string(abort_time) + " s");
        }
        const double seg_duration = found->end_time_sec - found->start_time_sec;
        const double local_t = std::clamp(abort_time - found->start_time_sec, 0.0, seg_duration);
        const trajectory::QuinticBSpline spline(found->control_points, seg_duration);
        P.row(slot) = spline.position(local_t).transpose();
        start_velocity.row(slot) = spline.velocity(local_t).transpose();
        if (inside) start_acceleration.row(slot) = spline.acceleration(local_t).transpose();

        Eigen::Vector3i color = Eigen::Vector3i::Zero();
        const auto& keys = found->color_keyframes;
        if (!keys.empty()) {
            color = keys.back().color_rgb;
            for (size_t i = 0; i + 1 < keys.size(); ++i) {
                if (abort_time <= keys[i + 1].time_sec) {
                    const double span = keys[i + 1].time_sec - keys[i].time_sec;
                    const double frac = span > 1e-12 ? std::clamp((abort_time - keys[i].time_sec) / span, 0.0, 1.0) : 1.0;
                    color = lerp_color(keys[i].color_rgb, keys[i + 1].color_rgb, frac);
                    break;
                }
            }
        }
        start_color.row(slot) = color.transpose();
    }

    // As the show's return leg: any free slot of any holding area (the
    // auction picks), landing at rest, LEDs fading to off; through hover
    // points above the slots. A drone already parked (on its pad, or at its
    // hover point) at the abort time keeps its pad.
    const Eigen::MatrixXd slots = compute_all_holding_positions(project.metadata.holding_areas);
    const double pad_height = std::max(config.solver.landing_approach_height_m, 0.0);
    const bool pad_rule =
        pad_height > 0.0 && slots_keep_distance(slots, config.solver.continuous_gatekeeper.min_allowable_distance_m);
    const double enforced_min_distance = meta_min_distance(config);
    std::vector<PadState> state(n);
    std::vector<std::optional<Eigen::Vector3d>> parked(n);
    if (pad_rule) {
        const Eigen::Vector3d up(0.0, 0.0, pad_height);
        for (int slot = 0; slot < n; ++slot) {
            const Eigen::Vector3d at = P.row(slot).transpose();
            for (int k = 0; k < slots.rows(); ++k) {
                const Eigen::Vector3d pad = slots.row(k).transpose();
                if ((at - pad).norm() < 1e-6) state[slot] = {PadKind::kOnPad, pad};
                if ((at - pad - up).norm() < 1e-6) state[slot] = {PadKind::kHover, pad};
            }
            if (state[slot].kind != PadKind::kAir) parked[slot] = state[slot].pad;
        }
    }
    assignment::AssignmentInput ain;
    ain.P = P;
    ain.Q = slots;
    ain.v_in_xy = start_velocity.leftCols(2);
    ain.w_distance = config.weights.w_distance;
    ain.w_vertical_climb = config.weights.w_vertical_climb;
    ain.w_heading_change = config.weights.w_heading_change;
    Eigen::MatrixXd cost = assignment::build_cost_matrix(ain);
    for (int i = 0; i < n; ++i) {
        if (!parked[i]) continue;
        for (int j = 0; j < n; ++j) {
            if ((slots.row(j).transpose() - *parked[i]).norm() > 1e-6) cost(i, j) += 1e4;  // no pad change
        }
    }
    const assignment::AssignmentResult assign_result = assignment::solve_auction(cost);

    std::vector<PadPlan> plans;
    if (pad_rule) {
        plans = plan_pad_moves(state, P, slots, assign_result.assignment, std::vector<char>(n, 1), std::vector<int>(n, 0),
                               false, true, pad_height, enforced_min_distance);
    }
    const auto start_of = [&](int slot) -> Eigen::Vector3d {
        return pad_rule ? plans[slot].start : Eigen::Vector3d(P.row(slot).transpose());
    };
    const auto end_of = [&](int slot) -> Eigen::Vector3d {
        return pad_rule ? plans[slot].end : Eigen::Vector3d(slots.row(assign_result.assignment[slot]).transpose());
    };
    double d_max = 0.0;
    int farthest = 0;
    for (int slot = 0; slot < n; ++slot) {
        const double d = (end_of(slot) - start_of(slot)).norm();
        if (d > d_max) d_max = d, farthest = slot;
    }
    const auto any_final = [&] {
        return pad_rule && std::any_of(plans.begin(), plans.end(), [](const PadPlan& p) { return p.final_descent; });
    };
    const double vertical_s = pad_rule ? vertical_move_duration(pad_height, config) : 0.0;
    double final_s = any_final() ? vertical_s : 0.0;
    // Timed like a leg: Auto = T_min, a target is flown as max(target, T_min);
    // a target covers the descent after the solved part too.
    const double t_min = trajectory::compute_min_transition_time(
        d_max, trajectory::inscribed_axis_limit(config.kinematics.v_max_mps),
        trajectory::inscribed_axis_limit(config.kinematics.a_max_mps2),
        trajectory::inscribed_axis_limit(config.kinematics.j_max_mps3), config.solver.kinematic_slack_fraction);
    const double duration = std::max({target_duration_sec.value_or(0.0) - final_s, t_min, 1e-6});

    std::vector<optimizer::DroneTransitionProblem> problems(n);
    const auto build_problem = [&](int slot) {
        optimizer::DroneTransitionProblem problem;
        problem.drone_id = drone_id_by_slot[slot];
        problem.start.position = start_of(slot);
        problem.start.velocity = start_velocity.row(slot).transpose();
        problem.start.acceleration = start_acceleration.row(slot).transpose();
        problem.end.position = end_of(slot);
        problem.end.velocity = Eigen::Vector3d::Zero();
        problem.end.acceleration = Eigen::Vector3d::Zero();
        // Every drone ends in a holding area, so it is exempt from that
        // area's zone (a hover point above the region lands too) and keeps
        // out of the others, exactly as on the show's return leg.
        problem.keep_out = !config.safety.keep_out.empty();
        problem.keep_out_exempt_area =
            exempt_area(holding, problem.start, problem.end,
                        pad_rule ? std::optional<Eigen::Vector3d>(plans[slot].pad) : std::nullopt);
        if (pad_rule) {
            const PadPlan& plan = plans[slot];
            if (plan.arrive == Arrive::kToHover) {
                problem.floor_m = std::min({plan.pad.z() + 0.5 * pad_height, problem.start.position.z(),
                                            problem.end.position.z()});
            }
            if (plan.prescribed()) {
                const Eigen::Vector3d pad = plan.pad;
                const bool down = plan.move == PadMove::kDescend || plan.move == PadMove::kDescendClimb;
                const bool up = plan.move == PadMove::kClimb || plan.move == PadMove::kDescendClimb;
                const CoreConfig& cfg = config;
                problem.prescribed = [pad, down, up, pad_height, &cfg](double t0, double t1, double total, int count) {
                    Eigen::MatrixXd cp = pad_window_control_points(pad, pad_height, down, up, t0, t1, total, count, cfg);
                    if (cp.rows() == 0) throw std::logic_error("a prescribed pad move doesn't fit its window");
                    return cp;
                };
            }
        }
        problems[slot] = problem;
    };
    for (int slot = 0; slot < n; ++slot) build_problem(slot);
    if (pad_rule) {
        settle_pad_plans(
            plans, problems, [&] { for (int slot = 0; slot < n; ++slot) build_problem(slot); }, pad_height, duration,
            enforced_min_distance, true, config);
        if (final_s == 0.0 && any_final()) final_s = vertical_s;
    }

    ReturnPathResult result;
    result.keyframe_index = keyframe_index;
    result.from_keyframe = from_name;
    result.abort_time_sec = abort_time;
    result.min_duration_sec = std::max(t_min, 1e-6) + final_s;
    result.farthest_drone_id = drone_id_by_slot[farthest];
    result.farthest_area = holding.area_of(slots.row(assign_result.assignment[farthest]).transpose());
    result.farthest_distance_m = d_max;
    if (estimate_only) return result;

    const std::string to_name = "holding_area";
    ProgressCallback transition_progress;
    if (progress) {
        transition_progress = [&](const ProgressEvent& solver_event) {
            ProgressEvent e = solver_event;
            e.transition_index = 0;
            e.transition_count = 1;
            e.from_keyframe = from_name;
            e.to_keyframe = to_name;
            progress(e);
        };
        ProgressEvent e;
        e.kind = ProgressEvent::Kind::TransitionStart;
        e.show_time_sec = 0.0;
        transition_progress(e);
    }

    ShowMetadata& meta = result.metadata;
    meta.fleet_size = n;
    meta.spline_degree = trajectory::kDegree;
    meta.min_distance_enforced_m = config.safety.min_distance_m * (1.0 + config.solver.collision_margin_fraction);
    meta.altitude_floor_m = config.safety.altitude_floor_m;
    meta.holding_clearance_m = largest_clearance(config);
    meta.holding_areas = project.metadata.holding_areas;
    meta.takeoff_area = holding.takeoff;

    std::vector<optimizer::DroneTrajectorySolution> solutions;
    optimizer::SolveStats solve_stats;
    try {
        solutions = optimizer::solve(problems, duration, config, transition_progress, &solve_stats);
    } catch (const optimizer::SafetyViolationError& e) {
        const optimizer::SafetyViolationReport& report = e.report();
        TransitionSafetyFailure failure;
        failure.solver = report;
        failure.transition_index = 0;
        failure.from_keyframe = from_name;
        failure.to_keyframe = to_name;
        failure.transition_start_time_sec = 0.0;
        failure.transition_duration_sec = report.attempts.empty() ? duration : report.attempts.back().duration_sec;
        failure.metadata = meta;
        failure.metadata.total_duration_sec = failure.transition_duration_sec;
        for (int slot = 0; slot < n && slot < static_cast<int>(report.rejected_solutions.size()); ++slot) {
            DroneTrajectory traj;
            traj.drone_id = drone_id_by_slot[slot];
            double stage_t_start = 0.0;
            for (const auto& stage : report.rejected_solutions[slot].stages) {
                TrajectorySegment segment;
                segment.segment_index = static_cast<int>(traj.segments.size());
                segment.start_time_sec = stage_t_start;
                segment.end_time_sec = stage_t_start + stage.duration;
                segment.control_points = stage.control_points;
                segment.knot_vector = trajectory::clamped_knot_vector(static_cast<int>(stage.control_points.rows()),
                                                                      trajectory::kDegree, stage.duration);
                segment.color_keyframes = {ColorKeyframe{segment.start_time_sec, start_color.row(slot)},
                                           ColorKeyframe{segment.end_time_sec, start_color.row(slot)}};
                traj.segments.push_back(std::move(segment));
                stage_t_start += stage.duration;
            }
            failure.rejected_trajectories.push_back(std::move(traj));
        }
        std::sort(failure.rejected_trajectories.begin(), failure.rejected_trajectories.end(),
                  [](const DroneTrajectory& a, const DroneTrajectory& b) { return a.drone_id < b.drone_id; });
        failure.solver.rejected_solutions.clear();
        throw PipelineSafetyError("Return path from '" + from_name + "' rejected: " + e.what(), std::move(failure));
    } catch (const optimizer::KeepOutViolationError& e) {
        // Not expected (every drone is exempt), kept so it can't escape untyped.
        throw std::runtime_error("Return path from '" + from_name + "': " + e.what());
    }

    const double flown_duration = solve_stats.flown_duration_sec;
    const Eigen::Vector3i off = Eigen::Vector3i::Zero();
    std::map<int, DroneTrajectory> by_drone;
    for (int slot = 0; slot < n; ++slot) {
        DroneTrajectory& traj = by_drone[drone_id_by_slot[slot]];
        traj.drone_id = drone_id_by_slot[slot];
        double stage_t_start = 0.0;
        for (const auto& stage : solutions[slot].stages) {
            const double stage_t_end = stage_t_start + stage.duration;
            TrajectorySegment segment;
            segment.segment_index = static_cast<int>(traj.segments.size());
            segment.start_time_sec = stage_t_start;
            segment.end_time_sec = stage_t_end;
            segment.control_points = stage.control_points;
            segment.knot_vector = trajectory::clamped_knot_vector(static_cast<int>(stage.control_points.rows()),
                                                                  trajectory::kDegree, stage.duration);
            segment.color_keyframes = {
                ColorKeyframe{stage_t_start, lerp_color(start_color.row(slot), off, stage_t_start / flown_duration)},
                ColorKeyframe{stage_t_end, lerp_color(start_color.row(slot), off, stage_t_end / flown_duration)},
            };
            traj.segments.push_back(std::move(segment));
            stage_t_start = stage_t_end;
        }
        if (final_s > 0.0) {  // the drones at their hover points land, together
            TrajectorySegment last =
                plans[slot].final_descent
                    ? landing_descent_segment(plans[slot].pad, pad_height, flown_duration, final_s,
                                              config.solver.num_control_points_min)
                    : still_segment(problems[slot].end.position, flown_duration, final_s,
                                    config.solver.num_control_points_min, off);
            last.segment_index = static_cast<int>(traj.segments.size());
            traj.segments.push_back(std::move(last));
        }
    }
    const double total_duration = flown_duration + final_s;
    for (auto& [drone_id, traj] : by_drone) result.trajectories.push_back(std::move(traj));

    TransitionTiming timing;
    timing.index = 0;
    timing.from_keyframe = from_name;
    timing.to_keyframe = to_name;
    timing.start_time_sec = 0.0;
    timing.end_time_sec = total_duration;
    timing.planned_duration_sec = duration + final_s;
    if (pad_rule) timing.pad_moves = count_pad_moves(plans);
    timing.flown_duration_sec = total_duration;
    timing.attempts = solve_stats.attempts;
    meta.transitions.push_back(timing);
    meta.return_leg = LegTiming{0.0, total_duration, target_duration_sec};
    meta.total_duration_sec = total_duration;
    result.worst_separation_m = solve_stats.worst_separation_m;

    if (transition_progress) {
        ProgressEvent e;
        e.kind = ProgressEvent::Kind::TransitionEnd;
        e.show_time_sec = total_duration;
        transition_progress(e);
    }
    return result;
}

}  // namespace drone_core::io
