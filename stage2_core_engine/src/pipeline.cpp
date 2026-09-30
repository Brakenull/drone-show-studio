#include "io/pipeline.hpp"

#include <algorithm>
#include <cmath>
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

// Rev 2.6 section 1.7's Staggered Wave Takeoff prepends/appends a
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

}  // namespace

PipelineResult run_pipeline(const ProjectData& project, const CoreConfig& base_config,
                            const ProgressCallback& progress) {
    const int n = project.metadata.fleet_size;
    // Altitude floor (docs/2-phase_2.md section 1.13, bug-report P2-02): the
    // design's ground, when the file declares one.
    CoreConfig config = base_config;
    config.safety.altitude_floor_m = project.metadata.ground_z_m;
    // Holding-area keep-out zone (docs/2-phase_2.md section 1.14): the
    // designer's safe distance around the holding region, when declared.
    const HoldingRegion holding_region = compute_holding_region(n, project.metadata.holding_area,
                                                                project.metadata.holding_area.grid_spacing_m);
    if (project.metadata.holding_area.show_clearance_m) {
        KeepOutZone zone;
        for (int axis = 0; axis < 3; ++axis) {
            zone.lo[axis] = holding_region.lo(axis);
            zone.hi[axis] = holding_region.hi(axis);
        }
        zone.clearance_m = *project.metadata.holding_area.show_clearance_m;
        config.safety.keep_out = zone;
    }
    // A point inside the holding region is a launch slot or a parked drone:
    // drones starting or ending a transition there are exempt from the zone.
    auto in_holding_region = [&](const Eigen::Vector3d& p) {
        return (p.array() >= holding_region.lo.array() - 1e-6).all() &&
               (p.array() <= holding_region.hi.array() + 1e-6).all();
    };
    PipelineResult result;
    result.metadata.altitude_floor_m = config.safety.altitude_floor_m;
    if (config.safety.keep_out) result.metadata.holding_clearance_m = config.safety.keep_out->clearance_m;
    result.metadata.fleet_size = n;
    result.metadata.spline_degree = trajectory::kDegree;
    result.metadata.min_distance_enforced_m =
        config.safety.min_distance_m * (1.0 + config.solver.collision_margin_fraction);

    Eigen::MatrixXd P =
        compute_holding_positions(n, project.metadata.holding_area, project.metadata.holding_area.grid_spacing_m);
    std::vector<int> drone_id_by_slot(n);
    for (int i = 0; i < n; ++i) drone_id_by_slot[i] = i;

    // Rev 2.6 section 1.7's Staggered Wave Takeoff applies only to the very
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
    const std::vector<int> launch_row_indices =
        stagger_takeoff ? compute_holding_row_indices(n, project.metadata.holding_area,
                                                        project.metadata.holding_area.grid_spacing_m)
                         : std::vector<int>();
    const int max_launch_row =
        launch_row_indices.empty() ? 0 : *std::max_element(launch_row_indices.begin(), launch_row_indices.end());

    // Real physical velocity at the start of the next transition; zero for
    // every drone at the launch pad (rest state).
    Eigen::MatrixXd actual_velocity = Eigen::MatrixXd::Zero(n, 3);

    // LEDs are off while parked in the holding area (not defined by the
    // Phase 1 schema, which has no color for holding_area) — an assumption
    // worth confirming once Phase 1/3 color handling at launch is settled.
    Eigen::MatrixXi actual_color = Eigen::MatrixXi::Zero(n, 3);

    // Cost-function heading vector: the spec initializes this to a unit
    // vector derived from heading_offset_deg only for the very first
    // transition (no prior motion to measure an angle from); every
    // subsequent transition uses the real outgoing velocity computed above.
    const double heading_rad = project.metadata.heading_offset_deg * kPi / 180.0;
    const Eigen::Vector2d initial_heading = assignment::initial_heading_velocity(heading_rad);
    Eigen::MatrixXd v_in_xy(n, 2);
    for (int i = 0; i < n; ++i) v_in_xy.row(i) = initial_heading.transpose();

    double t_cursor = 0.0;
    // Rev 2.5 section 1.6's T_min auto-scaling can push a transition's end
    // time later than the animator's nominal keyframe time_sec; this
    // accumulates that stretch so every later keyframe's nominal timestamp
    // shifts by the same amount, preserving their relative spacing.
    double time_stretch_offset = 0.0;

    const double v_limit_axis = trajectory::inscribed_axis_limit(config.kinematics.v_max_mps);
    const double a_limit_axis = trajectory::inscribed_axis_limit(config.kinematics.a_max_mps2);
    const double j_limit_axis = trajectory::inscribed_axis_limit(config.kinematics.j_max_mps3);

    std::map<int, DroneTrajectory> trajectories_by_drone;
    for (int i = 0; i < n; ++i) trajectories_by_drone[i].drone_id = i;

    // One entry per transition: holding area -> keyframes[0] -> ... ->
    // keyframes[last], then, when the Phase 1 file has `legs` (schema 1.6.0,
    // 1-phase_1.md section 3.8), the return leg keyframes[last] -> holding
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
        check_below(P, "the holding area");
        for (const Keyframe& kf : project.keyframes) check_below(points_by_index(kf, n), "keyframe '" + kf.shape_name + "'");
    }
    // Formation points must already keep the safe distance (the add-on
    // refuses to export otherwise, 1-phase_1.md section 3.2.2): a pinned
    // target inside the zone can't be planned around. Points inside the
    // region itself are parked drones and don't count.
    if (config.safety.keep_out) {
        const KeepOutZone& zone = *config.safety.keep_out;
        for (const Keyframe& kf : project.keyframes) {
            const Eigen::MatrixXd pts = points_by_index(kf, n);
            for (int i = 0; i < pts.rows(); ++i) {
                const Eigen::Vector3d p = pts.row(i).transpose();
                if (in_holding_region(p)) continue;
                const double d = optimizer::distance_to_keep_out_region(p, zone);
                if (d < zone.clearance_m - 1e-9) {
                    throw std::runtime_error("keyframe '" + kf.shape_name + "' has a point " + std::to_string(d) +
                                             " m from the holding area, closer than the safe distance of " +
                                             std::to_string(zone.clearance_m) + " m");
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
        // Return leg: land on the holding-area slots, any free slot (the
        // auction picks), LEDs fading to off as they are while parked.
        TransitionSpec ret;
        ret.targets = compute_holding_positions(n, project.metadata.holding_area,
                                                project.metadata.holding_area.grid_spacing_m);
        ret.target_colors = Eigen::MatrixXi::Zero(n, 3);
        ret.from_name = project.keyframes.back().shape_name;
        ret.to_name = "holding_area";
        ret.is_leg = true;
        ret.target_duration_sec = legs.return_duration_sec;
        ret.ends_at_rest = true;
        specs.push_back(std::move(ret));
    }

    for (size_t kf_index = 0; kf_index < specs.size(); ++kf_index) {
        const TransitionSpec& spec = specs[kf_index];
        const bool is_final_keyframe = spec.ends_at_rest;
        const std::string& from_keyframe = spec.from_name;

        // docs/5-studio_gui.md B2: stamps this transition onto every event,
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

        assignment::AssignmentInput ain;
        ain.P = P;
        ain.Q = Q;
        ain.v_in_xy = v_in_xy;
        ain.w_distance = config.weights.w_distance;
        ain.w_vertical_climb = config.weights.w_vertical_climb;
        ain.w_heading_change = config.weights.w_heading_change;
        const assignment::AssignmentResult assign_result = assignment::solve_auction(assignment::build_cost_matrix(ain));

        double d_max = 0.0;
        for (int slot = 0; slot < n; ++slot) {
            const int target_slot = assign_result.assignment[slot];
            d_max = std::max(d_max, (Q.row(target_slot) - P.row(slot)).norm());
        }

        const double t_start = t_cursor;
        // A keyframe transition ends at its keyframe's time (shifted by any
        // earlier stretch); a leg lasts its target, or just T_min when Auto.
        // Legs always get the T_min floor: "Auto" means the minimum, and a
        // target is flown as max(target, T_min) (1-phase_1.md section 3.8).
        const double nominal_t_end = spec.is_leg ? t_start + spec.target_duration_sec.value_or(0.0)
                                                 : spec.keyframe_time_sec + time_stretch_offset;
        const double nominal_duration = std::max(nominal_t_end - t_start, 1e-6);

        double duration = nominal_duration;
        if (config.solver.auto_scale_transition_time || spec.is_leg) {
            const double t_min = trajectory::compute_min_transition_time(
                d_max, v_limit_axis, a_limit_axis, j_limit_axis, config.solver.kinematic_slack_fraction);
            duration = std::max(nominal_duration, t_min);
        }
        // Staggering only delays departure from the holding area (kf_index
        // 0); every drone's own solved maneuver still takes exactly
        // `duration` — the wave adds a per-row wait before/after it, folded
        // into this transition's total span so later keyframes' timing is
        // unaffected in relative terms (same time_stretch_offset mechanism
        // Rev 2.5's T_min auto-scaling already relies on). The actual span
        // (and the time_stretch_offset update it feeds) is only known once
        // build_slot_outcomes() below has run and, if the staggered
        // configuration turns out unsafe, fallen back to the unstaggered
        // span — see that block.
        const bool apply_stagger = stagger_takeoff && spec.is_takeoff;

        // Formation Hold (final keyframe): drones stop, matching how the
        // show ends. Fly-Through Waypoint (every other keyframe): drones
        // keep moving through at a fraction of cruising speed instead of
        // braking to a full stop, which is what forcing v=0 at *every*
        // transition boundary was doing before (docs/2-phase_2.md Rev 2.5
        // section 1.6) — that left near-zero kinematic slack for
        // collision-avoidance bending, the root cause of the
        // PRIMAL_INFEASIBLE cases the last two revisions chased.
        constexpr double kFlyThroughSpeedFraction = 0.5;

        // Shared formation velocity (docs/2-phase_2.md section 1.18): every
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
                if (in_holding_region(target)) continue;  // parking: arrives at rest (below)
                const Eigen::Vector3d travel = target - P.row(slot).transpose();
                if (travel.norm() > 1e-6) {
                    direction_sum += travel.normalized();
                    ++flying;
                }
            }
            if (flying > 0) {
                formation_velocity = direction_sum / flying * (kFlyThroughSpeedFraction * config.kinematics.v_max_mps);
            }
            // Section 1.23 (P2-14): the rule above only looks at the incoming
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
                        if (in_holding_region(p)) continue;
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
            // the next one's start can dip below it (section 1.13). Levelled
            // for the whole formation, so it stays one shared velocity.
            for (int slot = 0; slot < n; ++slot) {
                const Eigen::Vector3d target = Q.row(assign_result.assignment[slot]).transpose();
                if (in_holding_region(target)) continue;
                if (optimizer::floor_safe_velocity(target, formation_velocity, config) != formation_velocity) {
                    formation_velocity.z() = 0.0;
                    break;
                }
            }
        }

        std::vector<optimizer::DroneTransitionProblem> problems(n);
        for (int slot = 0; slot < n; ++slot) {
            const int target_slot = assign_result.assignment[slot];
            optimizer::DroneTransitionProblem problem;
            problem.drone_id = drone_id_by_slot[slot];
            problem.start.position = P.row(slot).transpose();
            problem.start.velocity = actual_velocity.row(slot).transpose();
            problem.start.acceleration = Eigen::Vector3d::Zero();
            problem.end.position = Q.row(target_slot).transpose();
            // A holding-area target mid-show is a drone parking (or staying
            // parked): it lands and stops like at the show's end. With the
            // fly-through speed it reached its pad at ~3 m/s sideways and,
            // starting the next transition at that speed, skidded ~2 m into
            // the neighbouring pad (2026-09-29, 150_cone: 0.159 m on every
            // attempt). At rest, a drone that stays parked is also held
            // fixed by the solver (docs/2-phase_2.md section 1.15).
            problem.end.velocity = is_final_keyframe || in_holding_region(problem.end.position)
                                       ? Eigen::Vector3d::Zero()
                                       : formation_velocity;
            problem.end.acceleration = Eigen::Vector3d::Zero();
            // Section 1.14: only drones flying the show keep out of the zone;
            // taking off, landing or parked (start or end in the holding
            // region) are exempt.
            problem.keep_out = config.safety.keep_out.has_value() && !in_holding_region(problem.start.position) &&
                               !in_holding_region(problem.end.position);
            problems[slot] = problem;
        }

        auto lerp_color = [](const Eigen::Vector3i& c0, const Eigen::Vector3i& c1, double frac) -> Eigen::Vector3i {
            const Eigen::Vector3d result = c0.cast<double>() + frac * (c1.cast<double>() - c0.cast<double>());
            return result.array().round().matrix().cast<int>();
        };

        std::vector<optimizer::DroneTrajectorySolution> solutions;
        optimizer::SolveStats solve_stats;
        try {
            solutions = optimizer::solve(problems, duration, config, transition_progress, &solve_stats);
        } catch (const optimizer::SafetyViolationError& e) {
            // docs/5-studio_gui.md B1: put the rejection in show context
            // (which transition, show time, the rejected splines next to the
            // transitions that did pass) so a viewer can replay it.
            const optimizer::SafetyViolationReport& report = e.report();
            TransitionSafetyFailure failure;
            failure.solver = report;
            failure.transition_index = static_cast<int>(kf_index);
            failure.from_keyframe = from_keyframe;
            failure.to_keyframe = spec.to_name;
            failure.transition_start_time_sec = t_start;
            failure.transition_duration_sec = report.attempts.empty() ? duration : report.attempts.back().duration_sec;
            failure.metadata = result.metadata;
            failure.metadata.total_duration_sec = t_start + failure.transition_duration_sec;

            failure.completed_trajectories.reserve(trajectories_by_drone.size());
            for (const auto& [drone_id, traj] : trajectories_by_drone) failure.completed_trajectories.push_back(traj);

            std::map<int, DroneTrajectory> rejected_by_drone;
            for (int slot = 0; slot < n && slot < static_cast<int>(report.rejected_solutions.size()); ++slot) {
                const int target_slot = assign_result.assignment[slot];
                const int drone_id = drone_id_by_slot[slot];
                DroneTrajectory& traj = rejected_by_drone[drone_id];
                traj.drone_id = drone_id;
                int segment_index = static_cast<int>(trajectories_by_drone[drone_id].segments.size());
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
                std::to_string(v.distance_m) + " m of the holding area at show time " +
                std::to_string(t_start + v.time_sec) + " s (required " +
                std::to_string(config.safety.keep_out->clearance_m) + " m), after " + std::to_string(e.attempts()) +
                " attempt(s)");
        }

        // bug-report P2-03: `solutions` is the attempt that passed. After a
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

                // Rev 2.6 section 1.7: rows farther from the front of the
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

                // Rev 2.6 section 1.7: a transition longer than the
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
                        rest.position = Q.row(target_slot).transpose();
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
        transition_timing.start_time_sec = t_start;
        transition_timing.end_time_sec = t_end;
        transition_timing.planned_duration_sec = duration;
        transition_timing.flown_duration_sec = flown_duration;
        transition_timing.attempts = solve_stats.attempts;
        result.metadata.transitions.push_back(std::move(transition_timing));

        if (spec.is_leg) {
            LegTiming timing;
            timing.start_time_sec = t_start;
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

        P = Q;
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

}  // namespace drone_core::io
