#include "trajectory_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

#include <nlohmann/json.hpp>

namespace flight_packer {

namespace {

constexpr double kTimeTol = 1e-6;

int find_span(const BSplineCurve& c, double u) {
    const int n = static_cast<int>(c.control_points.size());
    if (u >= c.knots[n]) return n - 1;
    if (u <= c.knots[c.degree]) return c.degree;
    // Last index k in [degree, n-1] with knots[k] <= u.
    const auto it = std::upper_bound(c.knots.begin() + c.degree, c.knots.begin() + n + 1, u);
    return static_cast<int>(it - c.knots.begin()) - 1;
}

std::string where(int drone_id, std::size_t seg) {
    return "drone " + std::to_string(drone_id) + " segment " + std::to_string(seg) + ": ";
}

}  // namespace

std::array<double, 3> BSplineCurve::evaluate(double u) const {
    const int n = static_cast<int>(control_points.size());
    const int p = degree;
    u = std::clamp(u, knots[p], knots[n]);
    const int k = find_span(*this, u);
    // de Boor's algorithm.
    std::vector<std::array<double, 3>> d(control_points.begin() + (k - p), control_points.begin() + (k + 1));
    for (int r = 1; r <= p; ++r) {
        for (int j = p; j >= r; --j) {
            const double left = knots[j + k - p];
            const double denom = knots[j + 1 + k - r] - left;
            const double alpha = denom > 0.0 ? (u - left) / denom : 0.0;
            for (int axis = 0; axis < 3; ++axis) {
                d[j][axis] = (1.0 - alpha) * d[j - 1][axis] + alpha * d[j][axis];
            }
        }
    }
    return d[p];
}

BSplineCurve BSplineCurve::derivative() const {
    BSplineCurve out;
    const int n = static_cast<int>(control_points.size());
    out.degree = degree - 1;
    out.knots.assign(knots.begin() + 1, knots.end() - 1);
    out.control_points.resize(n - 1);
    for (int i = 0; i < n - 1; ++i) {
        const double denom = knots[i + degree + 1] - knots[i + 1];
        const double scale = denom > 0.0 ? degree / denom : 0.0;
        for (int axis = 0; axis < 3; ++axis) {
            out.control_points[i][axis] = scale * (control_points[i + 1][axis] - control_points[i][axis]);
        }
    }
    if (out.degree < 0 || out.control_points.empty()) {
        out.degree = 0;
        out.knots = {0.0, knots.back()};
        out.control_points = {{0.0, 0.0, 0.0}};
    }
    return out;
}

void DroneTimeline::position_velocity(double t, std::array<double, 3>& pos, std::array<double, 3>& vel) const {
    vel = {0.0, 0.0, 0.0};
    const SplineSegment& first = segments.front();
    if (t < first.start_time_sec) {
        pos = first.position.evaluate(0.0);
        return;
    }
    // Last segment that has started by t.
    const auto it = std::upper_bound(segments.begin(), segments.end(), t,
                                     [](double time, const SplineSegment& s) { return time < s.start_time_sec; });
    const SplineSegment& seg = *(it - 1);
    const double duration = seg.end_time_sec - seg.start_time_sec;
    const double u = t - seg.start_time_sec;
    if (u > duration) {
        pos = seg.position.evaluate(duration);  // gap or after the show: hold
        return;
    }
    pos = seg.position.evaluate(u);
    vel = seg.velocity.evaluate(u);
}

std::array<std::uint8_t, 3> DroneTimeline::color(double t) const {
    const auto it = std::upper_bound(colors.begin(), colors.end(), t,
                                     [](double time, const ColorKeyframe& k) { return time < k.time_sec; });
    std::array<double, 3> rgb;
    if (it == colors.begin()) {
        rgb = colors.front().rgb;
    } else if (it == colors.end()) {
        rgb = colors.back().rgb;
    } else {
        const ColorKeyframe& a = *(it - 1);
        const ColorKeyframe& b = *it;
        const double frac = (t - a.time_sec) / (b.time_sec - a.time_sec);
        for (int c = 0; c < 3; ++c) rgb[c] = a.rgb[c] + (b.rgb[c] - a.rgb[c]) * frac;
    }
    std::array<std::uint8_t, 3> out;
    for (int c = 0; c < 3; ++c) out[c] = static_cast<std::uint8_t>(std::clamp(std::floor(rgb[c] + 0.5), 0.0, 255.0));
    return out;
}

double DroneTimeline::end_time_sec() const { return segments.back().end_time_sec; }

double ShowData::end_time_sec() const {
    double end = 0.0;
    for (const auto& d : drones) end = std::max(end, d.end_time_sec());
    return end;
}

ShowData parse_trajectory_json_text(const std::string& text) {
    nlohmann::json root;
    try {
        root = nlohmann::json::parse(text);
    } catch (const nlohmann::json::parse_error& e) {
        throw ContractError(std::string("invalid JSON: ") + e.what());
    }
    try {
        const auto& meta = root.at("metadata");
        ShowData show;
        show.fleet_size = meta.at("fleet_size").get<int>();
        show.spline_degree = meta.at("spline_degree").get<int>();
        show.total_duration_sec = meta.at("total_duration_sec").get<double>();
        if (meta.value("coordinate_system", std::string("ENU")) != "ENU") {
            throw ContractError("metadata.coordinate_system must be ENU");
        }
        const auto& trajectories = root.at("trajectories");
        if (static_cast<int>(trajectories.size()) != show.fleet_size) {
            throw ContractError("metadata.fleet_size does not match the number of trajectories");
        }
        show.drones.resize(show.fleet_size);
        std::vector<bool> seen(show.fleet_size, false);

        for (const auto& traj : trajectories) {
            const int id = traj.at("drone_id").get<int>();
            if (id < 0 || id >= show.fleet_size || seen[id]) {
                throw ContractError("drone_id values must be unique and in 0..N-1 (got " + std::to_string(id) + ")");
            }
            seen[id] = true;
            DroneTimeline& drone = show.drones[id];
            drone.drone_id = id;

            for (const auto& s : traj.at("segments")) {
                SplineSegment seg;
                seg.start_time_sec = s.at("start_time_sec").get<double>();
                seg.end_time_sec = s.at("end_time_sec").get<double>();
                const std::size_t seg_no = drone.segments.size();
                if (!(seg.end_time_sec > seg.start_time_sec)) {
                    throw ContractError(where(id, seg_no) + "end_time_sec must exceed start_time_sec");
                }
                auto& curve = seg.position;
                curve.knots = s.at("knot_vector").get<std::vector<double>>();
                for (const auto& cp : s.at("control_points")) {
                    if (cp.size() != 3) throw ContractError(where(id, seg_no) + "control points must be [x, y, z]");
                    curve.control_points.push_back({cp[0].get<double>(), cp[1].get<double>(), cp[2].get<double>()});
                }
                curve.degree = static_cast<int>(curve.knots.size()) - static_cast<int>(curve.control_points.size()) - 1;
                if (curve.degree != show.spline_degree) {
                    throw ContractError(where(id, seg_no) + "knot/control-point count does not match spline_degree");
                }
                if (!std::is_sorted(curve.knots.begin(), curve.knots.end())) {
                    throw ContractError(where(id, seg_no) + "knot_vector must be non-decreasing");
                }
                // Accept local [0, duration] or absolute [start, end] knots; store local.
                const double duration = seg.end_time_sec - seg.start_time_sec;
                const double k0 = curve.knots.front();
                const double k1 = curve.knots.back();
                if (std::abs(k0 - seg.start_time_sec) <= kTimeTol && std::abs(k1 - seg.end_time_sec) <= kTimeTol &&
                    !(std::abs(k0) <= kTimeTol && std::abs(k1 - duration) <= kTimeTol)) {
                    for (double& k : curve.knots) k -= seg.start_time_sec;
                } else if (!(std::abs(k0) <= kTimeTol && std::abs(k1 - duration) <= kTimeTol)) {
                    throw ContractError(where(id, seg_no) + "knot_vector range matches neither local nor absolute time");
                }
                seg.velocity = curve.derivative();

                for (const auto& kf : s.at("color_keyframes")) {
                    const auto& c = kf.at("color_rgb");
                    ColorKeyframe key;
                    key.time_sec = kf.at("time_sec").get<double>();
                    for (int ch = 0; ch < 3; ++ch) {
                        const double v = c.at(ch).get<double>();
                        if (v < 0.0 || v > 255.0) throw ContractError(where(id, seg_no) + "color channel out of range");
                        key.rgb[ch] = v;
                    }
                    drone.colors.push_back(key);
                }
                drone.segments.push_back(std::move(seg));
            }
            if (drone.segments.empty()) throw ContractError("drone " + std::to_string(id) + " has no segments");
            if (drone.colors.empty()) throw ContractError("drone " + std::to_string(id) + " has no color keyframes");

            std::stable_sort(drone.segments.begin(), drone.segments.end(),
                             [](const SplineSegment& a, const SplineSegment& b) { return a.start_time_sec < b.start_time_sec; });
            for (std::size_t i = 1; i < drone.segments.size(); ++i) {
                if (drone.segments[i].start_time_sec < drone.segments[i - 1].end_time_sec - kTimeTol) {
                    throw ContractError(where(id, i) + "overlaps the previous segment");
                }
            }
            std::stable_sort(drone.colors.begin(), drone.colors.end(),
                             [](const ColorKeyframe& a, const ColorKeyframe& b) { return a.time_sec < b.time_sec; });
        }
        return show;
    } catch (const nlohmann::json::exception& e) {
        throw ContractError(std::string("trajectory contract: ") + e.what());
    }
}

ShowData load_trajectory_json(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw ContractError("cannot open " + path.string());
    std::ostringstream buffer;
    buffer << in.rdbuf();
    return parse_trajectory_json_text(buffer.str());
}

std::uint32_t record_count_for_duration(double duration_sec, std::uint16_t dt_ms) {
    const double duration_ms = std::max(0.0, duration_sec) * 1000.0;
    return static_cast<std::uint32_t>(std::ceil(duration_ms / dt_ms - 1e-9)) + 1u;
}

std::vector<Waypoint> sample_drone(const DroneTimeline& drone, std::uint32_t record_count, std::uint16_t dt_ms) {
    std::vector<Waypoint> out(record_count);
    for (std::uint32_t k = 0; k < record_count; ++k) {
        Waypoint& wp = out[k];
        wp.time_sec = static_cast<double>(k) * dt_ms / 1000.0;
        drone.position_velocity(wp.time_sec, wp.position_m, wp.velocity_mps);
        wp.color_rgb = drone.color(wp.time_sec);
    }
    return out;
}

}  // namespace flight_packer
