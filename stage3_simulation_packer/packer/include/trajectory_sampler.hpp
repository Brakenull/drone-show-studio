#pragma once

// Loads the Phase 2 -> Phase 3 contract (trajectory_splines.json)
// and samples each drone's B-spline timeline on the
// fixed 20 Hz flight-file grid.
//
// This is an independent C++ implementation (de Boor on the B-spline and
// its derivative curve) of the same timeline rules as
// warp_sim/loaders/spline_evaluator.py, which evaluates the power-basis form
// instead -- the tests cross-check the two:
//   * before a drone's first segment: first segment's start, v = 0
//   * inside a segment: exact spline, v = first derivative
//   * in a gap / after the last segment: hold the last reached end point, v = 0
//   * LED color: per-channel lerp between keyframes, floor(x + 0.5) to uint8,
//     same-instant keyframes step (later wins), clamp-to-edge.

#include <array>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include "quantizer.hpp"

namespace flight_packer {

class ContractError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

struct BSplineCurve {
    int degree = 0;
    std::vector<double> knots;                       // segment-local time
    std::vector<std::array<double, 3>> control_points;

    std::array<double, 3> evaluate(double u) const;  // clamps u to the knot range
    BSplineCurve derivative() const;
};

struct SplineSegment {
    double start_time_sec = 0.0;
    double end_time_sec = 0.0;
    BSplineCurve position;
    BSplineCurve velocity;  // position.derivative()
};

struct ColorKeyframe {
    double time_sec = 0.0;
    std::array<double, 3> rgb{};
};

struct DroneTimeline {
    int drone_id = 0;
    std::vector<SplineSegment> segments;   // time-ordered, non-overlapping
    std::vector<ColorKeyframe> colors;     // time-ordered (stable), merged across segments

    void position_velocity(double t, std::array<double, 3>& pos, std::array<double, 3>& vel) const;
    std::array<std::uint8_t, 3> color(double t) const;
    double end_time_sec() const;
};

struct ShowData {
    int fleet_size = 0;
    int spline_degree = 0;
    double total_duration_sec = 0.0;
    std::vector<DroneTimeline> drones;     // indexed by drone_id, ids exactly 0..N-1

    // Latest segment end across the fleet (>= metadata total_duration_sec when consistent).
    double end_time_sec() const;
};

ShowData load_trajectory_json(const std::filesystem::path& path);
ShowData parse_trajectory_json_text(const std::string& text);

// K = ceil(T_ms / dt_ms) + 1 samples at t_k = k * dt_ms, so the final record
// sits at (or just past) the show end and carries the landed/final state.
std::uint32_t record_count_for_duration(double duration_sec, std::uint16_t dt_ms);

std::vector<Waypoint> sample_drone(const DroneTimeline& drone, std::uint32_t record_count, std::uint16_t dt_ms);

}  // namespace flight_packer
