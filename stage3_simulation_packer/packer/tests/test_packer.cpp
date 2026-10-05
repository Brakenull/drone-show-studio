// Packer unit tests (no external test framework, matching stage2_core_engine/tests).

#include <cmath>
#include <cstdio>
#include <cstring>
#include <span>
#include <string>
#include <vector>

#include "crc32.hpp"
#include "flight_binary_spec.h"
#include "flight_file.hpp"
#include "quantizer.hpp"
#include "trajectory_sampler.hpp"

using namespace flight_packer;

namespace {

int g_failures = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
            ++g_failures;                                                        \
        }                                                                        \
    } while (0)

template <typename Fn>
bool throws(Fn&& fn) {
    try {
        fn();
    } catch (const std::exception&) {
        return true;
    }
    return false;
}

std::vector<std::uint8_t> bytes_of(const char* s) { return {s, s + std::strlen(s)}; }

void test_crc32() {
    CHECK(crc32_ieee(bytes_of("123456789")) == 0xCBF43926u);
    CHECK(crc32_ieee(bytes_of("")) == 0x00000000u);
    // Incremental == one-shot.
    const auto all = bytes_of("The quick brown fox jumps over the lazy dog");
    std::uint32_t state = crc32_init();
    state = crc32_update(state, std::span<const std::uint8_t>(all.data(), 10));
    state = crc32_update(state, std::span<const std::uint8_t>(all.data() + 10, all.size() - 10));
    CHECK(crc32_final(state) == crc32_ieee(all));
    CHECK(crc32_ieee(all) == 0x414FA339u);
    static_assert(crc32_ieee(std::span<const std::uint8_t>()) == 0u, "constexpr CRC");
}

void test_quantizer() {
    CHECK(quantize_position_cm(1.234) == 123);
    CHECK(quantize_position_cm(1.235) == 124 || quantize_position_cm(1.235) == 123);  // fp representation
    CHECK(quantize_position_cm(-0.005) == -1);   // halves away from zero
    CHECK(quantize_position_cm(327.67) == 32767);
    CHECK(quantize_position_cm(-327.68) == -32768);
    CHECK(throws([] { quantize_position_cm(327.68); }));
    CHECK(throws([] { quantize_position_cm(-327.69); }));
    CHECK(throws([] { quantize_position_cm(std::nan("")); }));
    CHECK(quantize_velocity_mms(1.5) == 1500);
    CHECK(quantize_velocity_mms(-32.768) == -32768);
    CHECK(throws([] { quantize_velocity_mms(32.768); }));
    CHECK(quantize_time_ms(599.95) == 599950u);
    CHECK(throws([] { quantize_time_ms(-1.0); }));

    Waypoint wp;
    wp.time_sec = 1.25;
    wp.position_m = {1.0, -2.5, 30.004};
    wp.velocity_mps = {0.0015, -0.0015, 6.0};
    wp.color_rgb = {1, 2, 3};
    const TrajectoryRecord r = quantize_waypoint(wp, 7);
    CHECK(r.time_ms == 1250u && r.pos_x_cm == 100 && r.pos_y_cm == -250 && r.pos_z_cm == 3000);
    CHECK(r.vel_x_mms == 2 && r.vel_y_mms == -2 && r.vel_z_mms == 6000);
    CHECK(r.color_r == 1 && r.color_g == 2 && r.color_b == 3);
    const Waypoint back = dequantize_record(r);
    CHECK(std::abs(back.position_m[2] - 30.0) < 1e-12 && std::abs(back.velocity_mps[2] - 6.0) < 1e-12);

    wp.position_m[0] = 400.0;
    bool named_drone = false;
    try {
        quantize_waypoint(wp, 42);
    } catch (const QuantizationError& e) {
        named_drone = std::string(e.what()).find("drone 42") != std::string::npos;
    }
    CHECK(named_drone);
}

void test_layout_and_file_image() {
    CHECK(sizeof(FlightFileHeader) == 24);
    CHECK(sizeof(FlightTrackEntry) == 16 && sizeof(FlightReturnEntry) == 12);
    CHECK(sizeof(TrajectoryRecord) == 19);
    // Worked example: 10 min at 20 Hz; the packer adds the closing sample.
    CHECK(record_count_for_duration(600.0, 50) == 12001u);
    CHECK(FLIGHT_FILE_SIZE(1, 0, 12000) == 228044u);
    CHECK(record_count_for_duration(0.0, 50) == 1u);
    CHECK(record_count_for_duration(0.049, 50) == 2u);

    std::vector<TrajectoryRecord> recs(3);
    for (std::uint32_t k = 0; k < recs.size(); ++k) {
        recs[k] = {};
        recs[k].time_ms = k * 50;
        recs[k].pos_z_cm = static_cast<std::int16_t>(-100 * static_cast<int>(k));
        recs[k].color_b = 255;
    }
    auto image = encode_flight_file(513, recs);
    CHECK(image.size() == 24 + 16 + 19 * 3 + 4);
    // Little-endian magic: bytes 57 48 53 44 ("WHSD" in memory == 0x44534857).
    CHECK(image[0] == 0x57 && image[1] == 0x48 && image[2] == 0x53 && image[3] == 0x44);
    CHECK(image[6] == 0x01 && image[7] == 0x02);  // drone 513
    // A packed struct view matches the explicit serialization (the firmware reads it this way).
    FlightFileHeader h;
    std::memcpy(&h, image.data(), sizeof h);
    CHECK(h.magic_number == FLIGHT_FILE_MAGIC && h.file_version == FLIGHT_FILE_VERSION && h.drone_id == 513 &&
          h.total_records == 3 && h.sampling_dt_ms == 50 && h.track_count == 1 && h.return_entry_count == 0 &&
          h.reserved == 0);
    FlightTrackEntry show;
    std::memcpy(&show, image.data() + 24, sizeof show);
    CHECK(show.first_record == 0 && show.record_count == 3 && show.kind == FLIGHT_TRACK_SHOW &&
          show.formation == FLIGHT_NO_FORMATION);
    TrajectoryRecord second;
    std::memcpy(&second, image.data() + 24 + 16 + 19, sizeof second);
    CHECK(second.time_ms == 50 && second.pos_z_cm == -100 && second.color_b == 255);

    const VerifyResult ok = verify_flight_image(image);
    CHECK(ok.ok);
    CHECK(ok.stored_crc == crc32_ieee(std::span<const std::uint8_t>(image.data(), image.size() - 4)));
    const auto decoded = decode_records(image);
    CHECK(decoded.size() == 3 && decoded[2].pos_z_cm == -200);

    auto flipped = image;
    flipped[24 + 16 + 19 + 5] ^= 0x10;  // one bit in a payload coordinate
    CHECK(!verify_flight_image(flipped).ok);
    auto truncated = image;
    truncated.pop_back();
    CHECK(!verify_flight_image(truncated).ok);
}

// Version 2: the show and two return tracks, a return table; every structural rule of the verifier.
void test_tracks_and_return_table() {
    auto track = [](std::uint16_t kind, std::uint16_t formation, std::uint32_t start, int n, std::int16_t z) {
        FlightTrack t{kind, formation, start, {}};
        for (int k = 0; k < n; ++k) {
            TrajectoryRecord r{};
            r.time_ms = static_cast<std::uint32_t>(k) * 50;
            r.pos_z_cm = z;
            t.records.push_back(r);
        }
        return t;
    };
    FlightImage in;
    in.drone_id = 7;
    in.pack_id = 0xA1B2C3D4u;
    in.tracks = {track(FLIGHT_TRACK_SHOW, FLIGHT_NO_FORMATION, 0, 5, 100),
                 track(FLIGHT_TRACK_RETURN, 0, 1000, 3, 200),
                 track(FLIGHT_TRACK_ABORT_POINT, 1, 1500, 2, 300)};
    in.return_table = {{0, 1000, 1, 0}, {1000, 1300, 2, 0}, {1300, 2000, 0, 0}, {2000, FLIGHT_TIME_END, FLIGHT_TRACK_NONE, 0}};
    const auto image = encode_flight_file(in);
    CHECK(image.size() == FLIGHT_FILE_SIZE(3, 4, 10));
    const VerifyResult v = verify_flight_image(image);
    CHECK(v.ok);
    CHECK(v.header.pack_id == 0xA1B2C3D4u && v.tracks.size() == 3 && v.return_table.size() == 4);
    CHECK(v.tracks[1].first_record == 5 && v.tracks[2].first_record == 8 && v.tracks[2].start_ms == 1500);
    const FlightImage out = decode_flight_image(image);
    CHECK(out.tracks[2].kind == FLIGHT_TRACK_ABORT_POINT && out.tracks[2].formation == 1);
    CHECK(out.tracks[1].records.size() == 3 && out.tracks[1].records[2].pos_z_cm == 200);
    CHECK(out.return_table[3].track == FLIGHT_TRACK_NONE && out.return_table[3].to_ms == FLIGHT_TIME_END);
    CHECK(decode_records(image).size() == 5);

    auto broken = [&](auto change) {
        FlightImage copy = in;
        change(copy);
        return verify_flight_image(encode_flight_file(copy));
    };
    CHECK(!broken([](FlightImage& f) { f.return_table[1].from_ms = 1100; }).ok);              // a gap
    CHECK(!broken([](FlightImage& f) { f.return_table[3].to_ms = 5000; }).ok);                // doesn't run to the end
    CHECK(!broken([](FlightImage& f) { f.return_table[0].track = 9; }).ok);                   // no such track
    CHECK(!broken([](FlightImage& f) { f.tracks[1].records[1].time_ms = 60; }).ok);          // not k * dt
    CHECK(!broken([](FlightImage& f) { f.tracks[2].kind = FLIGHT_TRACK_SHOW; }).ok);          // a second show
}

// Version 1 files (16-byte header, the show only) are still read.
void test_version_1_files_still_read() {
    std::vector<std::uint8_t> v1;
    auto put = [&v1](std::uint64_t v, int bytes) {
        for (int i = 0; i < bytes; ++i) v1.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
    };
    put(FLIGHT_FILE_MAGIC, 4);
    put(FLIGHT_FILE_VERSION_1, 2);
    put(3, 2);
    put(2, 4);
    put(50, 2);
    put(0, 2);
    for (int k = 0; k < 2; ++k) {
        put(static_cast<std::uint64_t>(k) * 50, 4);
        for (int i = 0; i < 15; ++i) v1.push_back(static_cast<std::uint8_t>(k + 1));
    }
    put(crc32_ieee(std::span<const std::uint8_t>(v1.data(), v1.size())), 4);
    CHECK(v1.size() == FLIGHT_FILE_V1_SIZE(2));
    const VerifyResult v = verify_flight_image(v1);
    CHECK(v.ok && v.version == FLIGHT_FILE_VERSION_1 && v.tracks.size() == 1 && v.return_table.empty());
    const auto recs = decode_records(v1);
    CHECK(recs.size() == 2 && recs[1].time_ms == 50 && recs[1].color_b == 2);
}

const char* kShowJson = R"({
  "metadata": {"version": "1.1.0", "fleet_size": 2, "spline_degree": 5, "continuity": "C4",
               "total_duration_sec": 10.0, "coordinate_system": "ENU"},
  "trajectories": [
    {"drone_id": 1, "segments": [
      {"segment_index": 0, "start_time_sec": 2.0, "end_time_sec": 6.0,
       "knot_vector": [2,2,2,2,2,2,6,6,6,6,6,6],
       "control_points": [[0,0,0],[1,0,0],[2,0,0],[3,0,0],[4,0,0],[5,0,0]],
       "color_keyframes": [{"time_sec": 2.0, "color_rgb": [0,0,0]}, {"time_sec": 4.0, "color_rgb": [255,0,0]},
                           {"time_sec": 4.0, "color_rgb": [0,255,0]}, {"time_sec": 6.0, "color_rgb": [0,255,100]}]}
    ]},
    {"drone_id": 0, "segments": [
      {"segment_index": 0, "start_time_sec": 0.0, "end_time_sec": 10.0,
       "knot_vector": [0,0,0,0,0,0,10,10,10,10,10,10],
       "control_points": [[0,0,10],[0,0,10],[0,0,10],[0,0,10],[0,0,10],[0,0,10]],
       "color_keyframes": [{"time_sec": 0.0, "color_rgb": [9,9,9]}]}
    ]}
  ]})";

void test_sampler() {
    const ShowData show = parse_trajectory_json_text(kShowJson);
    CHECK(show.fleet_size == 2 && show.drones[1].drone_id == 1);
    const DroneTimeline& d = show.drones[1];  // absolute knots, normalized to local
    std::array<double, 3> p, v;

    // Equally spaced collinear control points -> uniform motion 5 m / 4 s.
    d.position_velocity(4.0, p, v);
    CHECK(std::abs(p[0] - 2.5) < 1e-9 && std::abs(v[0] - 1.25) < 1e-9);
    d.position_velocity(0.0, p, v);  // before the first segment: hold start, v = 0
    CHECK(std::abs(p[0]) < 1e-12 && v[0] == 0.0);
    d.position_velocity(9.0, p, v);  // after the last segment: hold end, v = 0
    CHECK(std::abs(p[0] - 5.0) < 1e-12 && v[0] == 0.0);

    CHECK((d.color(3.0) == std::array<std::uint8_t, 3>{128, 0, 0}));  // 127.5 -> floor(x + 0.5)
    CHECK((d.color(4.0) == std::array<std::uint8_t, 3>{0, 255, 0}));  // step: later keyframe wins
    CHECK((d.color(5.0) == std::array<std::uint8_t, 3>{0, 255, 50}));
    CHECK((d.color(1.0) == std::array<std::uint8_t, 3>{0, 0, 0}));    // clamp-to-edge
    CHECK((d.color(99.0) == std::array<std::uint8_t, 3>{0, 255, 100}));

    const auto samples = sample_drone(d, record_count_for_duration(show.total_duration_sec, 50), 50);
    CHECK(samples.size() == 201);
    CHECK(std::abs(samples[80].time_sec - 4.0) < 1e-12 && std::abs(samples[80].position_m[0] - 2.5) < 1e-9);

    // Rejections.
    CHECK(throws([] { parse_trajectory_json_text("{"); }));
    std::string wrong_degree = kShowJson;
    wrong_degree.replace(wrong_degree.find("\"spline_degree\": 5"), 18, "\"spline_degree\": 4");
    CHECK(throws([&] { parse_trajectory_json_text(wrong_degree); }));
    std::string dup_id = kShowJson;
    dup_id.replace(dup_id.find("\"drone_id\": 0"), 13, "\"drone_id\": 1");
    CHECK(throws([&] { parse_trajectory_json_text(dup_id); }));
}

void test_bspline_derivative_matches_finite_difference() {
    BSplineCurve c;
    c.degree = 5;
    c.knots = {0, 0, 0, 0, 0, 0, 1.5, 3, 3, 3, 3, 3, 3};
    c.control_points = {{0, 0, 0}, {1, 2, 0}, {3, -1, 2}, {4, 4, 1}, {6, 0, 5}, {7, 1, 3}, {9, 2, 2}};
    const BSplineCurve dc = c.derivative();
    for (double u : {0.0, 0.7, 1.5, 2.2, 3.0}) {
        const double h = 1e-6;
        const double lo = std::max(0.0, u - h), hi = std::min(3.0, u + h);
        const auto a = c.evaluate(lo);
        const auto b = c.evaluate(hi);
        const auto d = dc.evaluate(u);
        for (int axis = 0; axis < 3; ++axis) {
            CHECK(std::abs((b[axis] - a[axis]) / (hi - lo) - d[axis]) < 1e-4);
        }
    }
    // Clamped ends interpolate the end control points.
    CHECK(std::abs(c.evaluate(3.0)[0] - 9.0) < 1e-12 && std::abs(c.evaluate(0.0)[1]) < 1e-12);
}

}  // namespace

int main() {
    test_crc32();
    test_quantizer();
    test_layout_and_file_image();
    test_tracks_and_return_table();
    test_version_1_files_still_read();
    test_sampler();
    test_bspline_derivative_matches_finite_difference();
    if (g_failures == 0) std::printf("test_packer: all checks passed\n");
    return g_failures == 0 ? 0 : 1;
}
