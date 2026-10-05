#pragma once

// Waypoint quantization:
//   position  -> int16 cm     [-327.68 m, +327.67 m]
//   velocity  -> int16 mm/s   [-32.768 m/s, +32.767 m/s]
//   color     -> uint8 x 3
//   time      -> uint32 ms since show start
//
// Rounding is to nearest, halves away from zero. A value outside the int16
// range is a hard error (QuantizationError), never a silent clamp: a
// saturated coordinate would send the drone somewhere it was never planned
// to go.

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "flight_binary_spec.h"

namespace flight_packer {

class QuantizationError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

struct Waypoint {
    double time_sec = 0.0;
    std::array<double, 3> position_m{};
    std::array<double, 3> velocity_mps{};
    std::array<std::uint8_t, 3> color_rgb{};
};

std::int16_t quantize_position_cm(double metres, const char* axis = "position");
std::int16_t quantize_velocity_mms(double metres_per_second, const char* axis = "velocity");
std::uint32_t quantize_time_ms(double seconds);

// Quantizes one waypoint; QuantizationError messages name the drone and time.
TrajectoryRecord quantize_waypoint(const Waypoint& waypoint, int drone_id);

// Inverse mapping, used by verification and tests.
Waypoint dequantize_record(const TrajectoryRecord& record);

}  // namespace flight_packer
