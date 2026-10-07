#pragma once

// Config resolution:
//   A. Kinematics — 4-tier: optional_config_overrides > Phase1
//      kinematic_constraints > core_config.json > compile-time defaults.
//   B. Solver hyperparameters & weights — 3-tier: optional_config_overrides
//      > core_config.json > compile-time defaults (never read from Phase1).
// Kept header-only per the file tree (no src/config.cpp).

#include <array>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>

#include <nlohmann/json.hpp>

namespace drone_core {

struct KinematicConfig {
    double v_max_mps = 6.0;
    double a_max_mps2 = 3.0;
    double j_max_mps3 = 5.0;
};

struct SolverWeights {
    double w_distance = 1.0;
    double w_vertical_climb = 2.5;
    // Lowered from 0.5 so the Auction assignment doesn't create
    // unnecessary topological crossings just to preserve heading.
    double w_heading_change = 0.2;
    double w_smoothness_snap = 1.0;
    double w_smoothness_jerk = 0.1;
};

// Holding-area keep-out zone: drones flying
// the show keep at least `clearance_m` from one holding area's region box
// [lo, hi] (ENU). A drone taking off from, landing in or parked in that area
// in a transition is exempt from that zone only
// (DroneTransitionProblem::keep_out_exempt_area).
struct KeepOutZone {
    std::array<double, 3> lo{};
    std::array<double, 3> hi{};
    double clearance_m = 0.0;
    int area = 0;  // the holding area's index
};

struct SafetyConfig {
    double safety_radius_m = 0.75;
    double min_distance_m = 1.5;
    // Altitude floor: no
    // point of any planned path may go below this ENU height. Set per run by
    // io::run_pipeline() from the Phase 1 file's project_metadata.ground_z_m;
    // unset (no floor) for files that don't declare a ground. Not a JSON key.
    std::optional<double> altitude_floor_m;
    // Set per run by io::run_pipeline(): one zone per holding area that
    // declares a show_clearance_m; empty (no zone) otherwise. Not a JSON key.
    std::vector<KeepOutZone> keep_out;
};

// APF Warm-Start Seeding:
// replaces the earlier drone_id-parity bow heuristic, which only broke
// symmetry for an isolated pairwise 2-drone crossing and, on a real dense
// show, forced dozens of drones converging on the same region into the same
// narrow bow surface — see trajectory/apf_seeder.hpp's incident writeup.
//
// This retires the continuous H*sin(theta) Z-stratification
// (it cancels to ~0 for due-East/West headings, theta ~= 0/pi, leaving
// exactly those flows with no altitude separation) in favor of a discrete
// 4-sector assignment: z_stratification_mode selects the mode
// ("4_sector_discrete" is the only supported non-disabled mode; any other
// value, e.g. "disabled", turns Z-stratification off), and z_layer_step_m
// is the unit step multiplied by {1,3}: the {+-0.75m, +-2.25m} sector
// bands.
struct ApfSeedingConfig {
    bool enabled = true;
    double k_repulsion = 50.0;
    double detection_radius_m = 3.0;
    int num_euler_steps = 10;
    double euler_dt = 0.05;
    std::string z_stratification_mode = "4_sector_discrete";
    double z_layer_step_m = 0.75;
};

// Adaptive Cutting-Plane Collocation: at the end of each SCP iteration, densely samples every conflict
// pair's real (non-linearized) separation at detection_frequency_hz and
// inserts up to max_dynamic_collocations_per_pair local-minima timestamps
// as extra hard collocation rows for the next iteration's QP, so OSQP
// constrains the true worst point of a near-miss instead of only the
// coarse per-window midpoint sample collect_collision_rows() already uses.
struct CuttingPlaneConfig {
    bool enabled = true;
    // <= 0: every dip under the planning distance gets a row.
    int max_dynamic_collocations_per_pair = 0;
    double detection_frequency_hz = 100.0;
};

// Decoupled Continuous Gatekeeper:
// replaces the earlier Slack Rejection Gatekeeper. A converged soft-slack
// solution is only ever OSQP-feasible, never automatically flight-safe —
// see scp_solver.cpp's evaluate_continuous_clearance() for the independent,
// slack-variable-free post-solve verification and the retry/expansion loop
// this configures.
struct ContinuousGatekeeperConfig {
    double verification_frequency_hz = 100.0;
    double min_allowable_distance_m = 1.45;
    bool auto_retry_with_expansion = true;
    double expansion_factor = 1.25;
    int max_retry_count = 2;
    // Fail fast (2026-09-29): an attempt whose worst separation is below
    // this is not retried. Stretching the duration by expansion_factor fixes
    // near misses at best; a miss this deep is structural (e.g. a pinned
    // boundary state), and each retry is a bigger solve than the last.
    // 0 retries every separation failure, as before.
    double min_retry_separation_m = 1.0;
};

// SCP/OSQP hyperparameters.
struct SolverOptions {
    // Floor for the adaptive control-point count,
    // not a fixed count anymore — see trajectory::adaptive_num_control_points.
    int num_control_points_min = 10;
    bool adaptive_control_points = true;
    int max_scp_iterations = 25;
    double convergence_tol = 1e-3;
    // Stall stop (2026-09-29): end a sub-stage's SCP loop after this many
    // iterations in a row that improve its best iterate by less than
    // scp_stall_tol_m (0 = off). The loop practically never meets
    // convergence_tol, and the returned best iterate was measured to gain at
    // most ~2 mm after ~10 iterations, so the rest of max_scp_iterations was
    // wasted time.
    int scp_stall_iterations = 6;
    double scp_stall_tol_m = 5e-4;
    // Move each drone's starting path to the closest flyable
    // one (kinematic box + floor) before the SCP's first step.
    bool repair_seed = true;
    // Every moving drone passes a sub-stage boundary with the
    // same velocity (the mean of their "straight to the end point" ones).
    bool shared_substage_velocity = true;
    // A mid-show formation's shared fly-through velocity is the
    // velocity of the formation's centre between the previous and the next
    // formation (false: the mean incoming direction x the fly-through speed).
    bool centered_formation_velocity = true;
    double trust_region_delta_m = 1.0;
    // 2026-10-02: each SCP step gets collision rows (and
    // coloring edges) only for the pairs its trust region can bring within
    // the planning distance, and only pairs that can be under it get the
    // dense scan. false = the old broad phase (pad 0.75 m + planning distance
    // per box, one voxel of slack, every pair scanned). The curve margin pads
    // each window's sampled box for the spline bulging between samples.
    bool tight_broad_phase = true;
    double broad_phase_curve_margin_m = 0.25;
    double collision_margin_fraction = 0.05;
    // The old seed_bow_magnitude_m was retired
    // along with the parity-seeding mechanism it configured — replaced by
    // apf_seeding below. jitter_magnitude_m is unrelated: it still backs the
    // 2-tier infeasibility self-healing loop's tier-2 fallback, so it's kept even though the
    // core_config.json example omits it (a compile-time default is enough
    // since nothing about tier 2 changed this revision).
    double jitter_magnitude_m = 0.3;
    // T_min auto-scaling: fraction of kinematic
    // headroom reserved for collision-avoidance bending.
    double kinematic_slack_fraction = 0.25;
    bool auto_scale_transition_time = true;
    // adaptive spatio-temporal hash time-bucket
    // (keeps window count near target_time_windows instead of a fixed 0.5s
    // bucket exploding once auto-scaling stretches T to 40-50s+), and
    // mega-cluster decomposition of long transitions into Gauss-Seidel
    // sub-stages so one transition never becomes a single, thousands-deep
    // sequential solve.
    bool adaptive_time_bucketing = true;
    int target_time_windows = 16;
    double max_substage_duration_s = 12.0;
    bool enable_staggered_takeoff = true;
    double staggered_wave_delay_s = 1.2;
    // 2026-10-02: landing drones first fly
    // to a hover point this far straight above their slot, then all descend
    // vertically together. 0 = fly straight onto the slot (before).
    double landing_approach_height_m = 2.0;
    // soft-slack collision formulation (a quadratic
    // penalty w_slack_collision on a nonnegative slack variable per collision
    // row, instead of a hard separation bound) so OSQP always has a
    // mathematically feasible solution regardless of how contested the local
    // geometry is, plus graph-coloring parallelization of the intra-cluster
    // solve order.
    double w_slack_collision = 100000.0;
    bool enable_graph_coloring = true;
    // These configure conflict-graph edge filtering
    // (cluster_distance_threshold_m) and weak-edge cluster-size capping
    // (max_cluster_size). Both are parsed from config for doc/schema
    // fidelity, but scp_solver.cpp currently does NOT apply either to its
    // conflict graph: doing so would let two drones that still share a
    // soft-slack collision QP row land in different clusters/colors and
    // solve concurrently on different OpenMP threads, racing on each
    // other's spline/control-point state (observed on a real 300-drone show
    // as two drones ending up 0.029 m apart, well under the 1.5 m minimum).
    // See scp_solver.cpp's build_conflict_edges() for the full incident
    // writeup and the safety invariant any future use of these two fields
    // must preserve.
    double cluster_distance_threshold_m = 2.0;
    int max_cluster_size = 12;

    ApfSeedingConfig apf_seeding{};
    CuttingPlaneConfig cutting_plane{};
    ContinuousGatekeeperConfig continuous_gatekeeper{};
};

struct CoreConfig {
    KinematicConfig kinematics{};
    SolverWeights weights{};
    SafetyConfig safety{};
    SolverOptions solver{};
};

namespace config_detail {

template <typename T>
inline void read_key(const nlohmann::json& section, const char* key, T& target) {
    if (section.contains(key) && !section.at(key).is_null()) {
        target = section.at(key).get<T>();
    }
}

}  // namespace config_detail

// Overlays a core_config.json-shaped object ({kinematics_default,
// solver_weights, safety, solver}) onto `config`. Missing sections/keys
// leave the corresponding field untouched. Shared by load_core_config_file()
// and by bindings/py_bindings.cpp's `optional_config_overrides` parameter,
// which uses this same shape as its highest-priority tier.
//
// The weights section accepts either "solver_weights" (core_config.json's
// own key) or "weights" (the override-level key:
// "optional_config_overrides.solver / weights"); both name the same thing,
// so both are accepted here.
inline void apply_json_overrides(CoreConfig& config, const nlohmann::json& root) {
    if (root.contains("kinematics_default")) {
        const auto& k = root.at("kinematics_default");
        config_detail::read_key(k, "v_max_mps", config.kinematics.v_max_mps);
        config_detail::read_key(k, "a_max_mps2", config.kinematics.a_max_mps2);
        config_detail::read_key(k, "j_max_mps3", config.kinematics.j_max_mps3);
    }
    for (const char* weights_key : {"solver_weights", "weights"}) {
        if (root.contains(weights_key)) {
            const auto& w = root.at(weights_key);
            config_detail::read_key(w, "w_distance", config.weights.w_distance);
            config_detail::read_key(w, "w_vertical_climb", config.weights.w_vertical_climb);
            config_detail::read_key(w, "w_heading_change", config.weights.w_heading_change);
            config_detail::read_key(w, "w_smoothness_snap", config.weights.w_smoothness_snap);
            config_detail::read_key(w, "w_smoothness_jerk", config.weights.w_smoothness_jerk);
        }
    }
    if (root.contains("safety")) {
        const auto& s = root.at("safety");
        config_detail::read_key(s, "safety_radius_m", config.safety.safety_radius_m);
        config_detail::read_key(s, "min_distance_m", config.safety.min_distance_m);
    }
    if (root.contains("solver")) {
        const auto& s = root.at("solver");
        config_detail::read_key(s, "num_control_points_min", config.solver.num_control_points_min);
        config_detail::read_key(s, "adaptive_control_points", config.solver.adaptive_control_points);
        config_detail::read_key(s, "max_scp_iterations", config.solver.max_scp_iterations);
        config_detail::read_key(s, "convergence_tol", config.solver.convergence_tol);
        config_detail::read_key(s, "scp_stall_iterations", config.solver.scp_stall_iterations);
        config_detail::read_key(s, "scp_stall_tol_m", config.solver.scp_stall_tol_m);
        config_detail::read_key(s, "repair_seed", config.solver.repair_seed);
        config_detail::read_key(s, "shared_substage_velocity", config.solver.shared_substage_velocity);
        config_detail::read_key(s, "centered_formation_velocity", config.solver.centered_formation_velocity);
        config_detail::read_key(s, "trust_region_delta_m", config.solver.trust_region_delta_m);
        config_detail::read_key(s, "tight_broad_phase", config.solver.tight_broad_phase);
        config_detail::read_key(s, "broad_phase_curve_margin_m", config.solver.broad_phase_curve_margin_m);        config_detail::read_key(s, "collision_margin_fraction", config.solver.collision_margin_fraction);
        config_detail::read_key(s, "jitter_magnitude_m", config.solver.jitter_magnitude_m);
        config_detail::read_key(s, "kinematic_slack_fraction", config.solver.kinematic_slack_fraction);
        config_detail::read_key(s, "auto_scale_transition_time", config.solver.auto_scale_transition_time);
        config_detail::read_key(s, "adaptive_time_bucketing", config.solver.adaptive_time_bucketing);
        config_detail::read_key(s, "target_time_windows", config.solver.target_time_windows);
        config_detail::read_key(s, "max_substage_duration_s", config.solver.max_substage_duration_s);
        config_detail::read_key(s, "enable_staggered_takeoff", config.solver.enable_staggered_takeoff);
        config_detail::read_key(s, "staggered_wave_delay_s", config.solver.staggered_wave_delay_s);
        config_detail::read_key(s, "landing_approach_height_m", config.solver.landing_approach_height_m);
        config_detail::read_key(s, "w_slack_collision", config.solver.w_slack_collision);
        config_detail::read_key(s, "enable_graph_coloring", config.solver.enable_graph_coloring);
        config_detail::read_key(s, "cluster_distance_threshold_m", config.solver.cluster_distance_threshold_m);
        config_detail::read_key(s, "max_cluster_size", config.solver.max_cluster_size);

        if (s.contains("apf_seeding")) {
            const auto& a = s.at("apf_seeding");
            config_detail::read_key(a, "enabled", config.solver.apf_seeding.enabled);
            config_detail::read_key(a, "k_repulsion", config.solver.apf_seeding.k_repulsion);
            config_detail::read_key(a, "detection_radius_m", config.solver.apf_seeding.detection_radius_m);
            config_detail::read_key(a, "num_euler_steps", config.solver.apf_seeding.num_euler_steps);
            config_detail::read_key(a, "euler_dt", config.solver.apf_seeding.euler_dt);
            config_detail::read_key(a, "z_stratification_mode", config.solver.apf_seeding.z_stratification_mode);
            config_detail::read_key(a, "z_layer_step_m", config.solver.apf_seeding.z_layer_step_m);
        }
        if (s.contains("cutting_plane")) {
            const auto& c = s.at("cutting_plane");
            config_detail::read_key(c, "enabled", config.solver.cutting_plane.enabled);
            config_detail::read_key(c, "max_dynamic_collocations_per_pair",
                                     config.solver.cutting_plane.max_dynamic_collocations_per_pair);
            config_detail::read_key(c, "detection_frequency_hz", config.solver.cutting_plane.detection_frequency_hz);
        }
        if (s.contains("continuous_gatekeeper")) {
            const auto& g = s.at("continuous_gatekeeper");
            config_detail::read_key(g, "verification_frequency_hz",
                                     config.solver.continuous_gatekeeper.verification_frequency_hz);
            config_detail::read_key(g, "min_allowable_distance_m",
                                     config.solver.continuous_gatekeeper.min_allowable_distance_m);
            config_detail::read_key(g, "auto_retry_with_expansion",
                                     config.solver.continuous_gatekeeper.auto_retry_with_expansion);
            config_detail::read_key(g, "expansion_factor", config.solver.continuous_gatekeeper.expansion_factor);
            config_detail::read_key(g, "max_retry_count", config.solver.continuous_gatekeeper.max_retry_count);
            config_detail::read_key(g, "min_retry_separation_m",
                                     config.solver.continuous_gatekeeper.min_retry_separation_m);
        }
    }
}

// Overlays stage2_core_engine/config/core_config.json onto `config`. A
// missing file leaves `config` untouched.
inline void load_core_config_file(const std::filesystem::path& config_path, CoreConfig& config) {
    if (!std::filesystem::exists(config_path)) {
        return;
    }
    std::ifstream file(config_path);
    if (!file.is_open()) {
        return;
    }
    nlohmann::json root;
    file >> root;
    apply_json_overrides(config, root);
}

// Kinematics-only overlay from project_metadata.kinematic_constraints in the
// Phase 1 JSON: overrides v_max/a_max/j_max only when present and non-null.
// Solver weights/hyperparameters are never sourced from Phase 1 data.
inline void apply_phase1_kinematic_overrides(CoreConfig& config, const nlohmann::json& project_metadata) {
    if (!project_metadata.contains("kinematic_constraints")) {
        return;
    }
    const auto& kc = project_metadata.at("kinematic_constraints");
    if (kc.is_null()) {
        return;
    }
    config_detail::read_key(kc, "v_max_mps", config.kinematics.v_max_mps);
    config_detail::read_key(kc, "a_max_mps2", config.kinematics.a_max_mps2);
    config_detail::read_key(kc, "j_max_mps3", config.kinematics.j_max_mps3);
}

// Resolves the full chain in the order lowest-to-highest priority:
//   Tier4/3 compile-time defaults -> Tier3/2 core_config.json ->
//   Tier2 Phase1 kinematic_constraints (kinematics only) ->
//   Tier1 optional_config_overrides (everything, highest priority).
inline CoreConfig resolve_core_config(const std::optional<nlohmann::json>& project_metadata,
                                       const nlohmann::json& overrides, const std::filesystem::path& config_path) {
    CoreConfig config{};
    load_core_config_file(config_path, config);
    if (project_metadata.has_value()) {
        apply_phase1_kinematic_overrides(config, *project_metadata);
    }
    apply_json_overrides(config, overrides);
    return config;
}

}  // namespace drone_core
