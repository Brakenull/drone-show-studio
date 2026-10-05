// pack_to_binary: Phase 2 trajectory contract -> drone_<id>.bin flight files
// (version 2 with return tracks).
//
//   pack_to_binary <trajectory_splines.json> <out_dir> [--dt-ms 50]
//   pack_to_binary --plan <pack_plan.json> <out_dir> [--dt-ms 50]
//   pack_to_binary --verify <file.bin | dir> [...]
//
// A pack plan names the show contract, the return-path contracts to pack as
// tracks after it, and the return table:
//
//   {"show": "stage2/trajectory_splines.json",
//    "tracks": [{"id": "1", "file": "stage2/returns/return_1.json", "kind": "return",
//                "formation": 1, "start_ms": 84549}, ...],          // kind: "return" | "abort_point"
//    "return_table": [{"from_ms": 0, "to_ms": 36637, "track": 1},  // track: 0 = the show, i = tracks[i - 1],
//                     ..., {"from_ms": 188000, "to_ms": null, "track": 0}]}   // null = no way home
//
// Relative paths are taken from the plan's folder. Without a plan the files hold the show alone.
//
// Every written file is read back from disk and re-verified (size formula,
// header, tracks, return table, CRC-32) before the run counts as successful.
// A manifest.json with per-file size and CRC, the tracks and the pack id is
// written next to the .bin files for the Phase 4 flasher.
//
// Exit codes: 0 ok, 1 verification failed, 2 bad input / quantization error.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <span>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "crc32.hpp"
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
                 "  pack_to_binary --plan <pack_plan.json> <out_dir> [--dt-ms N]\n"
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

std::vector<std::uint8_t> read_bytes(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot read " + path.string());
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

// One track to pack: its contract and where it sits in the show.
struct TrackSource {
    std::string id;
    ShowData data;
    std::uint16_t kind = FLIGHT_TRACK_SHOW;
    std::uint16_t formation = FLIGHT_NO_FORMATION;
    std::uint32_t start_ms = 0;
    std::uint32_t record_count = 0;
};

struct PackPlan {
    fs::path show_path;
    std::vector<TrackSource> tracks;           // tracks[0] is the show
    std::vector<FlightReturnEntry> return_table;
    std::uint32_t pack_id = 0;
};

// The pack id: CRC-32 over every input contract and the plan itself, so files from different inputs differ.
PackPlan load_plan(const fs::path& plan_path, const fs::path& show_path) {
    PackPlan plan;
    std::uint32_t id_state = crc32_init();
    auto feed = [&id_state](const std::vector<std::uint8_t>& bytes) {
        id_state = crc32_update(id_state, std::span<const std::uint8_t>(bytes.data(), bytes.size()));
    };

    nlohmann::json j = nlohmann::json::object();
    fs::path base;
    if (!plan_path.empty()) {
        const auto bytes = read_bytes(plan_path);
        feed(bytes);
        j = nlohmann::json::parse(bytes.begin(), bytes.end());
        base = plan_path.parent_path();
    }
    auto resolve = [&base](const std::string& p) {
        const fs::path path(p);
        return path.is_absolute() || base.empty() ? path : base / path;
    };
    plan.show_path = plan_path.empty() ? show_path : resolve(j.at("show").get<std::string>());

    TrackSource show;
    show.id = "show";
    show.data = load_trajectory_json(plan.show_path);
    feed(read_bytes(plan.show_path));
    plan.tracks.push_back(std::move(show));

    for (const auto& t : j.value("tracks", nlohmann::json::array())) {
        TrackSource track;
        track.id = t.at("id").get<std::string>();
        const fs::path file = resolve(t.at("file").get<std::string>());
        track.data = load_trajectory_json(file);
        feed(read_bytes(file));
        const std::string kind = t.at("kind").get<std::string>();
        if (kind == "return") {
            track.kind = FLIGHT_TRACK_RETURN;
        } else if (kind == "abort_point") {
            track.kind = FLIGHT_TRACK_ABORT_POINT;
        } else {
            throw std::runtime_error("track " + track.id + ": unknown kind '" + kind + "'");
        }
        track.formation = t.at("formation").get<std::uint16_t>();
        track.start_ms = t.at("start_ms").get<std::uint32_t>();
        if (track.data.fleet_size != plan.tracks[0].data.fleet_size) {
            throw std::runtime_error("track " + track.id + " has " + std::to_string(track.data.fleet_size) +
                                     " drones, the show " + std::to_string(plan.tracks[0].data.fleet_size));
        }
        plan.tracks.push_back(std::move(track));
    }
    if (plan.tracks.size() > FLIGHT_TRACK_NONE) throw std::runtime_error("too many tracks");

    for (const auto& e : j.value("return_table", nlohmann::json::array())) {
        FlightReturnEntry entry{};
        entry.from_ms = e.at("from_ms").get<std::uint32_t>();
        entry.to_ms = e.at("to_ms").is_null() ? FLIGHT_TIME_END : e.at("to_ms").get<std::uint32_t>();
        if (e.at("track").is_null()) {
            entry.track = FLIGHT_TRACK_NONE;
        } else {
            const int track = e.at("track").get<int>();
            if (track < 0 || track >= static_cast<int>(plan.tracks.size())) {
                throw std::runtime_error("return table names track " + std::to_string(track) + ", which isn't packed");
            }
            entry.track = static_cast<std::uint16_t>(track);
        }
        plan.return_table.push_back(entry);
    }
    plan.pack_id = crc32_final(id_state);
    return plan;
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
    fs::path plan_path;
    for (std::size_t i = 0; i < args.size(); ++i) {
        if (args[i] == "--dt-ms" && i + 1 < args.size()) {
            const int v = std::atoi(args[++i].c_str());
            if (v <= 0 || v > 65535) {
                std::cerr << "--dt-ms must be in 1..65535\n";
                return 2;
            }
            dt_ms = static_cast<std::uint16_t>(v);
        } else if (args[i] == "--plan" && i + 1 < args.size()) {
            plan_path = args[++i];
        } else {
            positional.push_back(args[i]);
        }
    }
    if (positional.size() != (plan_path.empty() ? 2u : 1u)) {
        print_usage();
        return 2;
    }
    const fs::path out_dir(positional.back());

    PackPlan plan;
    try {
        plan = load_plan(plan_path, plan_path.empty() ? fs::path(positional[0]) : fs::path());
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 2;
    }
    const ShowData& show = plan.tracks[0].data;
    if (show.fleet_size > 65536) {
        std::cerr << "error: fleet_size " << show.fleet_size << " exceeds the uint16 drone_id range\n";
        return 2;
    }

    std::uint32_t total_records = 0;
    for (auto& t : plan.tracks) {
        t.record_count = record_count_for_duration(std::max(t.data.end_time_sec(), t.data.total_duration_sec), dt_ms);
        total_records += t.record_count;
    }
    const std::uint32_t file_size =
        FLIGHT_FILE_SIZE(plan.tracks.size(), plan.return_table.size(), total_records);
    fs::create_directories(out_dir);

    nlohmann::json manifest;
    manifest["source"] = plan.show_path.string();
    if (!plan_path.empty()) manifest["plan"] = plan_path.string();
    manifest["file_version"] = FLIGHT_FILE_VERSION;
    manifest["pack_id"] = crc_hex(plan.pack_id);
    manifest["fleet_size"] = show.fleet_size;
    manifest["sampling_dt_ms"] = dt_ms;
    manifest["records_per_file"] = total_records;
    manifest["file_size_bytes"] = file_size;
    manifest["tracks"] = nlohmann::json::array();
    for (const auto& t : plan.tracks) {
        manifest["tracks"].push_back({{"id", t.id},
                                      {"kind", t.kind == FLIGHT_TRACK_SHOW     ? "show"
                                               : t.kind == FLIGHT_TRACK_RETURN ? "return"
                                                                               : "abort_point"},
                                      {"formation", t.kind == FLIGHT_TRACK_SHOW ? nlohmann::json() : nlohmann::json(t.formation)},
                                      {"start_ms", t.start_ms},
                                      {"records", t.record_count}});
    }
    manifest["return_entries"] = plan.return_table.size();
    manifest["files"] = nlohmann::json::array();

    int failures = 0;
    std::uint64_t total_bytes = 0;
    for (std::size_t d = 0; d < show.drones.size(); ++d) {
        const int drone_id = show.drones[d].drone_id;
        FlightImage image;
        image.drone_id = static_cast<std::uint16_t>(drone_id);
        image.sampling_dt_ms = dt_ms;
        image.pack_id = plan.pack_id;
        image.return_table = plan.return_table;
        try {
            for (const auto& t : plan.tracks) {
                // Drone ids are exactly 0..N-1 in every contract (load_trajectory_json checks it).
                const DroneTimeline& drone = t.data.drones[d];
                FlightTrack track{t.kind, t.formation, t.start_ms, {}};
                track.records.reserve(t.record_count);
                for (const auto& wp : sample_drone(drone, t.record_count, dt_ms)) {
                    track.records.push_back(quantize_waypoint(wp, drone_id));
                }
                image.tracks.push_back(std::move(track));
            }
        } catch (const QuantizationError& e) {
            std::cerr << "error: " << e.what() << "\n";
            return 2;
        }

        const auto bytes = encode_flight_file(image);
        const fs::path path = flight_file_name(out_dir, drone_id);
        write_file_atomically(path, bytes);

        const VerifyResult check = verify_flight_file(path);  // read back from disk
        const bool ok = check.ok && check.header.drone_id == drone_id && check.header.total_records == total_records &&
                        check.header.pack_id == plan.pack_id && check.tracks.size() == plan.tracks.size() &&
                        check.return_table.size() == plan.return_table.size() && check.file_size == file_size;
        if (!ok) {
            ++failures;
            std::cerr << "FAIL " << path.string() << ": " << (check.ok ? "header mismatch" : check.error) << "\n";
        }
        total_bytes += bytes.size();
        manifest["files"].push_back({{"drone_id", drone_id},
                                     {"file", path.filename().string()},
                                     {"size_bytes", bytes.size()},
                                     {"crc32", crc_hex(check.stored_crc)},
                                     {"verified", ok}});
    }

    std::ofstream(out_dir / "manifest.json") << manifest.dump(2) << "\n";
    std::cout << "Packed " << show.fleet_size << " drone(s): " << plan.tracks.size() << " track(s) (the show and "
              << plan.tracks.size() - 1 << " return(s)), " << plan.return_table.size() << " return entries, "
              << total_records << " records x " << dt_ms << " ms, " << file_size << " bytes/file, " << total_bytes
              << " bytes total, pack " << crc_hex(plan.pack_id) << " -> " << out_dir.string() << "\n";
    std::cout << (show.fleet_size - failures) << "/" << show.fleet_size << " file(s) passed read-back CRC-32 verification\n";
    return failures == 0 ? 0 : 1;
}
