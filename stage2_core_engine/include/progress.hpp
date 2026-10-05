#pragma once

#include <functional>
#include <limits>
#include <string>

// Progress reporting for long Stage 2 solves. An empty ProgressCallback is the default everywhere and costs
// one branch per call site: no event is built unless a callback is set.
//
// Every callback runs on the calling (main) thread, never inside an OpenMP
// region, so a callback may do anything the caller's thread may do (the
// Python binding re-acquires the GIL in it). An exception thrown by the
// callback aborts the solve and propagates to the caller unchanged.

namespace drone_core {

struct ProgressEvent {
    enum class Kind {
        TransitionStart,  // before assignment for transition `transition_index`
        AttemptStart,     // before gatekeeper attempt `attempt` of the current transition
        ScpIteration,     // after SCP iteration `iteration` of sub-stage `substage`
        AttemptEnd,       // after the continuous gatekeeper checked attempt `attempt`
        TransitionEnd,    // after the transition's segments were committed
    };

    Kind kind = Kind::TransitionStart;

    // Set by the pipeline on every event (the solver alone doesn't know them).
    int transition_index = -1;  // 0 = holding area -> keyframes[0]
    int transition_count = 0;
    std::string from_keyframe;
    std::string to_keyframe;

    // TransitionStart: show time the transition starts. TransitionEnd: show
    // time it ends (after any stagger / auto-scaled stretch).
    double show_time_sec = 0.0;

    // Attempt* and ScpIteration. 1-based; max_attempts includes the first try.
    int attempt = 0;
    int max_attempts = 0;
    double duration_sec = 0.0;  // this attempt's (possibly expanded) transition duration

    // ScpIteration. 1-based.
    int substage = 0;
    int substage_count = 0;
    int iteration = 0;
    int max_iterations = 0;
    int conflict_pairs = 0;  // distinct drone pairs in this iteration's conflict graph
    double max_delta_m = 0.0;  // largest control-point move this iteration (convergence measure)
    // The solver's own broad-phase estimate, not the gatekeeper's verdict;
    // +infinity when no pair was a candidate.
    double min_separation_m = std::numeric_limits<double>::infinity();
    bool converged = false;
    // This step's candidate was accepted (else the sub-stage fell
    // back to its best iterate); the step's trust-region radius; the best
    // iterate's separation (same scan as min_separation_m).
    bool step_accepted = false;
    double trust_region_m = 0.0;
    double best_min_separation_m = std::numeric_limits<double>::infinity();
    // This step's drone QPs solved by tier 0 (with trust region), tier 1
    // (without), tier 2 (jittered), or none.
    int qp_tier_counts[4] = {0, 0, 0, 0};
    // Seed repair, per sub-stage (same on each of its steps): drones whose
    // starting path was already flyable, was repaired, or couldn't be.
    int seed_repair_counts[3] = {0, 0, 0};
    // Where the step's time went (2026-10-02, measurement only). Wall times:
    // the whole step, its sweep of drone QPs, and the candidate's evaluation
    // split into broad phase (hash + conflict edges) and dense pair scans.
    // The *_cpu_sec fields are summed over the sweep's drone solves (all
    // threads): building collision rows vs. the QP tiers themselves (incl.
    // any row re-collection a fallback tier does).
    // setup_sec is the sub-stage's seeding, repair and first evaluation
    // (same on each of its steps).
    double step_sec = 0.0;
    double sweep_sec = 0.0;
    double broad_phase_sec = 0.0;
    double scan_sec = 0.0;
    double rows_cpu_sec = 0.0;
    double qp_cpu_sec = 0.0;
    double setup_sec = 0.0;
    int candidate_pairs = 0;  // broad-phase (drone pair, time window) entries the step's QPs read
    int collision_rows = 0;   // collision + keep-out rows over the sweep's drone QPs (first build)
    int color_count = 0;      // sequential batches of the sweep (clusters when coloring is off)

    // AttemptEnd.
    double worst_separation_m = std::numeric_limits<double>::infinity();
    double required_separation_m = 0.0;
    bool passed = false;
    // Which gatekeeper checks this attempt failed (passed == all three ok).
    // The floor and zone checks are trivially ok when the run has no floor
    // or no keep-out zone.
    bool separation_ok = false;
    bool floor_ok = true;
    bool zone_ok = true;
    // Lowest control point of the attempt (bounds its whole path from below);
    // +infinity when the run has no altitude floor.
    double lowest_z_m = std::numeric_limits<double>::infinity();
};

using ProgressCallback = std::function<void(const ProgressEvent&)>;

}  // namespace drone_core
