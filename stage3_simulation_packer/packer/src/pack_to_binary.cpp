// pack_to_binary: Phase 2 trajectory contract -> drone_<id>.bin flight files
// (docs/3-phase-3.md §4).
//
//   pack_to_binary <trajectory_splines.json> <out_dir> [--dt-ms 50]
//   pack_to_binary --verify <file.bin | dir> [...]
//
// Every written file is read back from disk and re-verified (size formula,
// header, CRC-32) before the run counts as successful. A manifest.json with
// per-file size and CRC is written next to the .bin files for the Phase 4
// flasher.
//
// Exit codes: 0 ok, 1 verification failed, 2 bad input / quantization error.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "drone_constants.hpp"
#include "flight_file.hpp"
#include "quantizer.hpp"
#include "trajectory_sampler.hpp"

namespace fs = std::filesystem;
using namespace flight_packer;

namespace {

void print_usage() {
    std::cerr << "usage:\n"
                 "  pack_to_binary <trajectory_splines.json> <out_dir> [--dt-ms N]\n"
                 "  pack_to_binary --verify <file.bin | dir> [...]\n";
}

int verify_paths(const std::vector<std::string>& targets) {
    std::vector<fs::path> files;
    for (const auto& t : targets) {
        const fs::path p(t);
        if (fs::is_directory(p)) {
            for (const auto& entry : fs::directory_iterator(p)) {
                if (entry.is_regular_file() && entry.path().extension() == ".bin") files.push_back(entry.path());
            }
        } else {
            files.push_back(p);
        }
    }
    if (files.empty()) {
        std::cerr << "no .bin files found\n";
        return 2;
    }
    int failures = 0;
    for (const auto& f : files) {
        const VerifyResult r = verify_flight_file(f);
        if (!r.ok) {
            ++failures;
            std::cerr << "FAIL " << f.string() << ": " << r.error << "\n";
        }
    }
    std::cout << (files.size() - failures) << "/" << files.size() << " file(s) passed CRC-32 + layout verification\n";
    return failures == 0 ? 0 : 1;
}

char hex_digit(unsigned v) { return "0123456789abcdef"[v & 0xFu]; }

std::string crc_hex(std::uint32_t crc) {
    std::string s(8, '0');
    for (int i = 7; i >= 0; --i, crc >>= 4) s[i] = hex_digit(crc);
    return s;
}

}  // namespace

int main(int argc, char** argv) {
    std::vector<std::string> args(argv + 1, argv + argc);
    if (args.empty()) {
        print_usage();
        return 2;
    }
    if (args[0] == "--verify") {
        if (args.size() < 2) {
            print_usage();
            return 2;
        }
        return verify_paths(std::vector<std::string>(args.begin() + 1, args.end()));
    }

    std::vector<std::string> positional;
    std::uint16_t dt_ms = drone_constants::kSamplingDtMs;
    for (std::size_t i = 0; i < args.size(); ++i) {
        if (args[i] == "--dt-ms" && i + 1 < args.size()) {
            const int v = std::atoi(args[++i].c_str());
            if (v <= 0 || v > 65535) {
                std::cerr << "--dt-ms must be in 1..65535\n";
                return 2;
            }
            dt_ms = static_cast<std::uint16_t>(v);
        } else {
            positional.push_back(args[i]);
        }
    }
    if (positional.size() != 2) {
        print_usage();
        return 2;
    }
    const fs::path input(positional[0]);
    const fs::path out_dir(positional[1]);

    ShowData show;
    try {
        show = load_trajectory_json(input);
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 2;
    }
    if (show.fleet_size > 65536) {
        std::cerr << "error: fleet_size " << show.fleet_size << " exceeds the uint16 drone_id range\n";
        return 2;
    }

    const double duration = std::max(show.end_time_sec(), show.total_duration_sec);
    const std::uint32_t record_count = record_count_for_duration(duration, dt_ms);
    fs::create_directories(out_dir);

    nlohmann::json manifest;
    manifest["source"] = input.string();
    manifest["fleet_size"] = show.fleet_size;
    manifest["sampling_dt_ms"] = dt_ms;
    manifest["records_per_file"] = record_count;
    manifest["file_size_bytes"] = FLIGHT_FILE_SIZE(record_count);
    manifest["files"] = nlohmann::json::array();

    int failures = 0;
    std::uint64_t total_bytes = 0;
    for (const auto& drone : show.drones) {
        std::vector<TrajectoryRecord> records;
        records.reserve(record_count);
        try {
            for (const auto& wp : sample_drone(drone, record_count, dt_ms)) {
                records.push_back(quantize_waypoint(wp, drone.drone_id));
            }
        } catch (const QuantizationError& e) {
            std::cerr << "error: " << e.what() << "\n";
            return 2;
        }

        const auto image = encode_flight_file(static_cast<std::uint16_t>(drone.drone_id), records, dt_ms);
        const fs::path path = flight_file_name(out_dir, drone.drone_id);
        write_file_atomically(path, image);

        const VerifyResult check = verify_flight_file(path);  // read back from disk
        const bool ok = check.ok && check.header.drone_id == drone.drone_id &&
                        check.header.total_records == record_count && check.file_size == FLIGHT_FILE_SIZE(record_count);
        if (!ok) {
            ++failures;
            std::cerr << "FAIL " << path.string() << ": " << (check.ok ? "header mismatch" : check.error) << "\n";
        }
        total_bytes += image.size();
        manifest["files"].push_back({{"drone_id", drone.drone_id},
                                     {"file", path.filename().string()},
                                     {"size_bytes", image.size()},
                                     {"crc32", crc_hex(check.stored_crc)},
                                     {"verified", ok}});
    }

    std::ofstream(out_dir / "manifest.json") << manifest.dump(2) << "\n";
    std::cout << "Packed " << show.fleet_size << " drone(s): " << record_count << " records x " << dt_ms
              << " ms (" << duration << " s), " << FLIGHT_FILE_SIZE(record_count) << " bytes/file, " << total_bytes
              << " bytes total -> " << out_dir.string() << "\n";
    std::cout << (show.fleet_size - failures) << "/" << show.fleet_size << " file(s) passed read-back CRC-32 verification\n";
    return failures == 0 ? 0 : 1;
}
