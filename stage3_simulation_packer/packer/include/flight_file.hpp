#pragma once

// drone_<id>.bin encode / write / verify (docs/3-phase-3.md §4.2, §5).

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "flight_binary_spec.h"
#include "quantizer.hpp"

namespace flight_packer {

// Full file image: header + records + CRC-32 footer (little-endian).
std::vector<std::uint8_t> encode_flight_file(std::uint16_t drone_id, const std::vector<TrajectoryRecord>& records,
                                             std::uint16_t sampling_dt_ms = FLIGHT_FILE_SAMPLING_DT_MS);

// Writes via a temporary file + rename so a crash never leaves a truncated .bin.
void write_file_atomically(const std::filesystem::path& path, const std::vector<std::uint8_t>& bytes);

std::filesystem::path flight_file_name(const std::filesystem::path& dir, int drone_id);

struct VerifyResult {
    bool ok = false;
    std::string error;                 // empty when ok
    FlightFileHeader header{};
    std::uint32_t stored_crc = 0;
    std::uint32_t computed_crc = 0;
    std::uint64_t file_size = 0;
};

// Structural + integrity check of a file image: size == 20 + 19K, magic,
// version, dt, reserved == 0, CRC-32 over header + payload matches footer,
// and record timestamps strictly increase by sampling_dt_ms.
VerifyResult verify_flight_image(const std::vector<std::uint8_t>& bytes);
VerifyResult verify_flight_file(const std::filesystem::path& path);

std::vector<TrajectoryRecord> decode_records(const std::vector<std::uint8_t>& bytes);

}  // namespace flight_packer
