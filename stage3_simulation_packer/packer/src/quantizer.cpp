#include "quantizer.hpp"

#include <cmath>
#include <limits>
#include <sstream>

namespace flight_packer {

namespace {

std::int16_t to_int16(double scaled, double original, const char* what, const char* unit) {
    if (!std::isfinite(scaled)) {
        throw QuantizationError(std::string(what) + " is not finite");
    }
    const double rounded = std::round(scaled);  // halves away from zero
    if (rounded < std::numeric_limits<std::int16_t>::min() || rounded > std::numeric_limits<std::int16_t>::max()) {
        std::ostringstream msg;
        msg << what << " " << original << " " << unit << " is outside the int16 flight-file range";
        throw QuantizationError(msg.str());
    }
    return static_cast<std::int16_t>(rounded);
}

}  // namespace

std::int16_t quantize_position_cm(double metres, const char* axis) {
    return to_int16(metres * FLIGHT_POSITION_UNITS_PER_M, metres, axis, "m");
}

std::int16_t quantize_velocity_mms(double metres_per_second, const char* axis) {
    return to_int16(metres_per_second * FLIGHT_VELOCITY_UNITS_PER_MPS, metres_per_second, axis, "m/s");
}

std::uint32_t quantize_time_ms(double seconds) {
    const double ms = std::round(seconds * 1000.0);
    if (!std::isfinite(ms) || ms < 0.0 || ms > static_cast<double>(std::numeric_limits<std::uint32_t>::max())) {
        throw QuantizationError("time " + std::to_string(seconds) + " s is outside the uint32 ms range");
    }
    return static_cast<std::uint32_t>(ms);
}

TrajectoryRecord quantize_waypoint(const Waypoint& wp, int drone_id) {
    try {
        TrajectoryRecord r{};
        r.time_ms = quantize_time_ms(wp.time_sec);
        r.pos_x_cm = quantize_position_cm(wp.position_m[0], "position x");
        r.pos_y_cm = quantize_position_cm(wp.position_m[1], "position y");
        r.pos_z_cm = quantize_position_cm(wp.position_m[2], "position z");
        r.vel_x_mms = quantize_velocity_mms(wp.velocity_mps[0], "velocity x");
        r.vel_y_mms = quantize_velocity_mms(wp.velocity_mps[1], "velocity y");
        r.vel_z_mms = quantize_velocity_mms(wp.velocity_mps[2], "velocity z");
        r.color_r = wp.color_rgb[0];
        r.color_g = wp.color_rgb[1];
        r.color_b = wp.color_rgb[2];
        return r;
    } catch (const QuantizationError& e) {
        std::ostringstream msg;
        msg << "drone " << drone_id << " at t=" << wp.time_sec << " s: " << e.what();
        throw QuantizationError(msg.str());
    }
}

Waypoint dequantize_record(const TrajectoryRecord& r) {
    Waypoint wp;
    wp.time_sec = r.time_ms / 1000.0;
    wp.position_m = {r.pos_x_cm / FLIGHT_POSITION_UNITS_PER_M, r.pos_y_cm / FLIGHT_POSITION_UNITS_PER_M,
                     r.pos_z_cm / FLIGHT_POSITION_UNITS_PER_M};
    wp.velocity_mps = {r.vel_x_mms / FLIGHT_VELOCITY_UNITS_PER_MPS, r.vel_y_mms / FLIGHT_VELOCITY_UNITS_PER_MPS,
                       r.vel_z_mms / FLIGHT_VELOCITY_UNITS_PER_MPS};
    wp.color_rgb = {r.color_r, r.color_g, r.color_b};
    return wp;
}

}  // namespace flight_packer
