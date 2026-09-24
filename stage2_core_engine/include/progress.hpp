#pragma once

#include <functional>
#include <limits>
#include <string>

// Progress reporting for long Stage 2 solves (docs/5-studio_gui.md section
// 5.2, B2). An empty ProgressCallback is the default everywhere and costs
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

    // AttemptEnd.
    double worst_separation_m = std::numeric_limits<double>::infinity();
    double required_separation_m = 0.0;
    bool passed = false;
};

using ProgressCallback = std::function<void(const ProgressEvent&)>;

}  // namespace drone_core
