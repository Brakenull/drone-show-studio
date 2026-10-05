#pragma once

// drone_<id>.bin encode / write / verify (docs/3-phase-3.md §4.2, §5): version 2, the show and its return
// paths as tracks plus the return table; version 1 files are still read and verified.

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "flight_binary_spec.h"
#include "quantizer.hpp"

namespace flight_packer {

struct FlightTrack {
    std::uint16_t kind = FLIGHT_TRACK_SHOW;
    std::uint16_t formation = FLIGHT_NO_FORMATION;
    std::uint32_t start_ms = 0;               // show time the track starts at
    std::vector<TrajectoryRecord> records;    // timed from the track's start
};

struct FlightImage {
    std::uint16_t drone_id = 0;
    std::uint16_t sampling_dt_ms = FLIGHT_FILE_SAMPLING_DT_MS;
    std::uint32_t pack_id = 0;
    std::vector<FlightTrack> tracks;          // tracks[0] is the show
    std::vector<FlightReturnEntry> return_table;
};

// Full file image: header, track directory, return table, records, CRC-32 footer (little-endian).
std::vector<std::uint8_t> encode_flight_file(const FlightImage& image);

// A show-only file (one track, no return table, pack_id 0).
std::vector<std::uint8_t> encode_flight_file(std::uint16_t drone_id, const std::vector<TrajectoryRecord>& records,
                                             std::uint16_t sampling_dt_ms = FLIGHT_FILE_SAMPLING_DT_MS);

// Writes via a temporary file + rename so a crash never leaves a truncated .bin.
void write_file_atomically(const std::filesystem::path& path, const std::vector<std::uint8_t>& bytes);

std::filesystem::path flight_file_name(const std::filesystem::path& dir, int drone_id);

struct VerifyResult {
    bool ok = false;
    std::string error;                        // empty when ok
    std::uint16_t version = 0;
    FlightFileHeader header{};                // a version 1 file reads as one show track and no table
    std::vector<FlightTrackEntry> tracks;
    std::vector<FlightReturnEntry> return_table;
    std::uint32_t stored_crc = 0;
    std::uint32_t computed_crc = 0;
    std::uint64_t file_size = 0;
};

// Structural + integrity check of a file image: the size formula, magic, version, dt, reserved == 0,
// CRC-32 over everything before the footer; the tracks start with the show, follow each other in the
// records and add up to total_records, and each track's record times are k * sampling_dt_ms; the return
// table is sorted, contiguous from 0 to FLIGHT_TIME_END and names existing tracks.
VerifyResult verify_flight_image(const std::vector<std::uint8_t>& bytes);
VerifyResult verify_flight_file(const std::filesystem::path& path);

FlightImage decode_flight_image(const std::vector<std::uint8_t>& bytes);
// The show track's records.
std::vector<TrajectoryRecord> decode_records(const std::vector<std::uint8_t>& bytes);

}  // namespace flight_packer
