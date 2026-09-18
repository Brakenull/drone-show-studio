#include "io/pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <map>
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

PipelineResult run_pipeline(const ProjectData& project, const CoreConfig& config) {
    const int n = project.metadata.fleet_size;
    PipelineResult result;
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
    // KNOWN BUG (found 2026-09-17, pre-existing since Rev 2.6/2.7, not fixed
    // by Rev 2.9, not yet fixed here): every row's maneuver control points
    // come from ONE joint optimizer::solve() call below that assumes every
    // drone executes its own maneuver starting at the SAME local time 0. The
    // per-row delay applied further down (`stage_t_start = t_start +
    // pre_delay_s`) then re-times each row's *already solved* maneuver to
    // start later in absolute wall-clock time, without re-verifying (or
    // re-solving) pairwise separation across that shift — a delayed row's
    // real position at absolute time T is that maneuver's solved position at
    // local time (T - delay), which the joint solve never checked against
    // an undelayed row's position at local time T. Confirmed via ablation on
    // a synthetic N=10 ring-swap holding-area departure: enabling staggering
    // measured 1.001 m worst-case separation; disabling it (no other change)
    // raised that to 1.773 m. See stage2_nway_conflict_limitation memory.
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

    for (size_t kf_index = 0; kf_index < project.keyframes.size(); ++kf_index) {
        const Keyframe& kf = project.keyframes[kf_index];
        const bool is_final_keyframe = (kf_index + 1 == project.keyframes.size());

        const Eigen::MatrixXd Q = points_by_index(kf, n);
        const Eigen::MatrixXi Q_colors = colors_by_index(kf, n);

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
        const double nominal_t_end = kf.time_sec + time_stretch_offset;
        const double nominal_duration = std::max(nominal_t_end - t_start, 1e-6);

        double duration = nominal_duration;
        if (config.solver.auto_scale_transition_time) {
            const double t_min = trajectory::compute_min_transition_time(
                d_max, v_limit_axis, a_limit_axis, j_limit_axis, config.solver.kinematic_slack_fraction);
            duration = std::max(nominal_duration, t_min);
        }
        // Staggering only delays departure from the holding area (kf_index
        // 0); every drone's own solved maneuver still takes exactly
        // `duration` — the wave adds a per-row wait before/after it, folded
        // into this transition's total span so later keyframes' timing is
        // unaffected in relative terms (same time_stretch_offset mechanism
        // Rev 2.5's T_min auto-scaling already relies on).
        const bool apply_stagger = stagger_takeoff && kf_index == 0;
        const double stagger_span_s = apply_stagger ? max_launch_row * config.solver.staggered_wave_delay_s : 0.0;
        const double t_end = t_start + duration + stagger_span_s;
        time_stretch_offset += (t_end - nominal_t_end);

        // Formation Hold (final keyframe): drones stop, matching how the
        // show ends. Fly-Through Waypoint (every other keyframe): drones
        // keep moving through at a fraction of cruising speed instead of
        // braking to a full stop, which is what forcing v=0 at *every*
        // transition boundary was doing before (docs/2-phase_2.md Rev 2.5
        // section 1.6) — that left near-zero kinematic slack for
        // collision-avoidance bending, the root cause of the
        // PRIMAL_INFEASIBLE cases the last two revisions chased.
        constexpr double kFlyThroughSpeedFraction = 0.5;

        std::vector<optimizer::DroneTransitionProblem> problems(n);
        for (int slot = 0; slot < n; ++slot) {
            const int target_slot = assign_result.assignment[slot];
            optimizer::DroneTransitionProblem problem;
            problem.drone_id = drone_id_by_slot[slot];
            problem.start.position = P.row(slot).transpose();
            problem.start.velocity = actual_velocity.row(slot).transpose();
            problem.start.acceleration = Eigen::Vector3d::Zero();
            problem.end.position = Q.row(target_slot).transpose();
            if (is_final_keyframe) {
                problem.end.velocity = Eigen::Vector3d::Zero();
            } else {
                const Eigen::Vector3d travel = problem.end.position - problem.start.position;
                const double travel_norm = travel.norm();
                problem.end.velocity =
                    travel_norm > 1e-6
                        ? Eigen::Vector3d(travel / travel_norm * (kFlyThroughSpeedFraction * config.kinematics.v_max_mps))
                        : Eigen::Vector3d::Zero();
            }
            problem.end.acceleration = Eigen::Vector3d::Zero();
            problems[slot] = problem;
        }

        const std::vector<optimizer::DroneTrajectorySolution> solutions = optimizer::solve(problems, duration, config);

        Eigen::MatrixXd next_actual_velocity = Eigen::MatrixXd::Zero(n, 3);
        Eigen::MatrixXi next_actual_color = Eigen::MatrixXi::Zero(n, 3);
        std::vector<int> next_drone_id_by_slot(n, -1);

        auto lerp_color = [](const Eigen::Vector3i& c0, const Eigen::Vector3i& c1, double frac) -> Eigen::Vector3i {
            const Eigen::Vector3d result = c0.cast<double>() + frac * (c1.cast<double>() - c0.cast<double>());
            return result.array().round().matrix().cast<int>();
        };

        for (int slot = 0; slot < n; ++slot) {
            const int target_slot = assign_result.assignment[slot];
            const int drone_id = drone_id_by_slot[slot];
            DroneTrajectory& traj = trajectories_by_drone[drone_id];

            double stage_t_start = t_start;

            // Rev 2.6 section 1.7: rows farther from the front of the launch
            // grid wait `row_index * staggered_wave_delay_s` before starting
            // their real maneuver, and (to keep every drone's segment list
            // spanning the same [t_start, t_end] transition window) the rows
            // that took off first idle at their solved end state for the
            // remaining wave time after their maneuver finishes.
            if (apply_stagger) {
                const int row = launch_row_indices[slot];
                const double pre_delay_s = row * config.solver.staggered_wave_delay_s;
                if (pre_delay_s > 1e-9) {
                    const trajectory::BoundaryConditions& start = problems[slot].start;
                    TrajectorySegment hold;
                    hold.segment_index = static_cast<int>(traj.segments.size());
                    hold.start_time_sec = stage_t_start;
                    hold.end_time_sec = stage_t_start + pre_delay_s;
                    hold.control_points =
                        build_hold_segment_control_points(start, pre_delay_s, config.solver.num_control_points_min);
                    hold.knot_vector = trajectory::clamped_knot_vector(
                        static_cast<int>(hold.control_points.rows()), trajectory::kDegree, pre_delay_s);
                    hold.color_keyframes = {ColorKeyframe{hold.start_time_sec, actual_color.row(slot)},
                                             ColorKeyframe{hold.end_time_sec, actual_color.row(slot)}};
                    traj.segments.push_back(std::move(hold));
                    stage_t_start += pre_delay_s;
                }
            }

            // Rev 2.6 section 1.7: a transition longer than the mega-cluster
            // threshold comes back as multiple chained sub-stages instead of
            // one piece — emit one TrajectorySegment per sub-stage, chained
            // start-to-end across the transition's [t_start, t_end] span.
            // `maneuver_t_start` (rather than t_start) anchors the color-lerp
            // fraction so a staggered pre-delay hold doesn't shift the real
            // maneuver's color interpolation.
            const double maneuver_t_start = stage_t_start;
            const auto& stages = solutions[slot].stages;
            for (size_t stage_idx = 0; stage_idx < stages.size(); ++stage_idx) {
                const auto& stage = stages[stage_idx];
                const double stage_t_end = stage_t_start + stage.duration;
                const int num_control_points = static_cast<int>(stage.control_points.rows());
                const Eigen::VectorXd knot_vector =
                    trajectory::clamped_knot_vector(num_control_points, trajectory::kDegree, stage.duration);

                const double frac_start = (stage_t_start - maneuver_t_start) / duration;
                const double frac_end = (stage_t_end - maneuver_t_start) / duration;

                TrajectorySegment segment;
                segment.segment_index = static_cast<int>(traj.segments.size());
                segment.start_time_sec = stage_t_start;
                segment.end_time_sec = stage_t_end;
                segment.knot_vector = knot_vector;
                segment.control_points = stage.control_points;
                segment.color_keyframes = {
                    ColorKeyframe{stage_t_start, lerp_color(actual_color.row(slot), Q_colors.row(target_slot), frac_start)},
                    ColorKeyframe{stage_t_end, lerp_color(actual_color.row(slot), Q_colors.row(target_slot), frac_end)},
                };
                traj.segments.push_back(std::move(segment));

                if (stage_idx + 1 == stages.size()) {
                    const trajectory::QuinticBSpline spline(stage.control_points, stage.duration);
                    next_actual_velocity.row(target_slot) = spline.velocity(stage.duration).transpose();
                }
                stage_t_start = stage_t_end;
            }

            // Rows that departed earlier finish their own maneuver before
            // t_end; they hover at their formation slot (v=0) until the
            // slowest (highest-row) wave catches up, so every slot is
            // simultaneously at rest at Q when the next transition starts —
            // matching the P = Q assumption below for every slot, not just
            // the ones with no post-gap.
            if (apply_stagger) {
                const double post_gap_s = t_end - stage_t_start;
                if (post_gap_s > 1e-9) {
                    trajectory::BoundaryConditions rest;
                    rest.position = Q.row(target_slot).transpose();
                    rest.velocity = Eigen::Vector3d::Zero();
                    rest.acceleration = Eigen::Vector3d::Zero();
                    TrajectorySegment hold;
                    hold.segment_index = static_cast<int>(traj.segments.size());
                    hold.start_time_sec = stage_t_start;
                    hold.end_time_sec = t_end;
                    hold.control_points =
                        build_hold_segment_control_points(rest, post_gap_s, config.solver.num_control_points_min);
                    hold.knot_vector = trajectory::clamped_knot_vector(
                        static_cast<int>(hold.control_points.rows()), trajectory::kDegree, post_gap_s);
                    hold.color_keyframes = {ColorKeyframe{hold.start_time_sec, Q_colors.row(target_slot)},
                                             ColorKeyframe{hold.end_time_sec, Q_colors.row(target_slot)}};
                    traj.segments.push_back(std::move(hold));
                    next_actual_velocity.row(target_slot) = Eigen::Vector3d::Zero();
                }
            }

            next_actual_color.row(target_slot) = Q_colors.row(target_slot);
            next_drone_id_by_slot[target_slot] = drone_id;
        }

        P = Q;
        v_in_xy = next_actual_velocity.leftCols(2);
        actual_velocity = next_actual_velocity;
        actual_color = next_actual_color;
        drone_id_by_slot = std::move(next_drone_id_by_slot);
        t_cursor = t_end;
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
