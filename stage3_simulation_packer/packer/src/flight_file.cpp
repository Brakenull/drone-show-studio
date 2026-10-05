#include "flight_file.hpp"

#include <cstring>
#include <fstream>
#include <iterator>
#include <span>
#include <stdexcept>
#include <system_error>

#include "crc32.hpp"

namespace flight_packer {

namespace {

void put_u8(std::vector<std::uint8_t>& out, std::uint8_t v) { out.push_back(v); }

void put_u16(std::vector<std::uint8_t>& out, std::uint16_t v) {
    out.push_back(static_cast<std::uint8_t>(v & 0xFFu));
    out.push_back(static_cast<std::uint8_t>(v >> 8));
}

void put_i16(std::vector<std::uint8_t>& out, std::int16_t v) { put_u16(out, static_cast<std::uint16_t>(v)); }

void put_u32(std::vector<std::uint8_t>& out, std::uint32_t v) {
    for (int shift = 0; shift < 32; shift += 8) out.push_back(static_cast<std::uint8_t>((v >> shift) & 0xFFu));
}

std::uint16_t get_u16(const std::uint8_t* p) { return static_cast<std::uint16_t>(p[0] | (p[1] << 8)); }

std::uint32_t get_u32(const std::uint8_t* p) {
    return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}

TrajectoryRecord get_record(const std::uint8_t* p) {
    TrajectoryRecord r{};
    r.time_ms = get_u32(p);
    r.pos_x_cm = static_cast<std::int16_t>(get_u16(p + 4));
    r.pos_y_cm = static_cast<std::int16_t>(get_u16(p + 6));
    r.pos_z_cm = static_cast<std::int16_t>(get_u16(p + 8));
    r.vel_x_mms = static_cast<std::int16_t>(get_u16(p + 10));
    r.vel_y_mms = static_cast<std::int16_t>(get_u16(p + 12));
    r.vel_z_mms = static_cast<std::int16_t>(get_u16(p + 14));
    r.color_r = p[16];
    r.color_g = p[17];
    r.color_b = p[18];
    return r;
}

// Where the records start in a verified image.
std::size_t records_offset(const VerifyResult& v) {
    if (v.version == FLIGHT_FILE_VERSION_1) return FLIGHT_FILE_V1_HEADER_SIZE;
    return FLIGHT_FILE_HEADER_SIZE + FLIGHT_TRACK_ENTRY_SIZE * v.tracks.size() +
           FLIGHT_RETURN_ENTRY_SIZE * v.return_table.size();
}

}  // namespace

std::vector<std::uint8_t> encode_flight_file(const FlightImage& image) {
    if (image.tracks.empty() || image.tracks[0].kind != FLIGHT_TRACK_SHOW) {
        throw std::invalid_argument("encode_flight_file: track 0 must be the show");
    }
    if (image.tracks.size() > 0xFFFFu || image.return_table.size() > 0xFFFFu) {
        throw std::invalid_argument("encode_flight_file: too many tracks or return entries");
    }
    std::uint64_t total = 0;
    for (const auto& t : image.tracks) total += t.records.size();
    std::vector<std::uint8_t> out;
    out.reserve(FLIGHT_FILE_SIZE(image.tracks.size(), image.return_table.size(), total));
    // Serialized field by field (explicit little-endian), not memcpy'd, so the
    // image is identical whatever the host's endianness or struct ABI.
    put_u32(out, FLIGHT_FILE_MAGIC);
    put_u16(out, FLIGHT_FILE_VERSION);
    put_u16(out, image.drone_id);
    put_u32(out, static_cast<std::uint32_t>(total));
    put_u16(out, image.sampling_dt_ms);
    put_u16(out, static_cast<std::uint16_t>(image.tracks.size()));
    put_u16(out, static_cast<std::uint16_t>(image.return_table.size()));
    put_u16(out, 0);
    put_u32(out, image.pack_id);
    std::uint32_t first = 0;
    for (const auto& t : image.tracks) {
        put_u32(out, first);
        put_u32(out, static_cast<std::uint32_t>(t.records.size()));
        put_u32(out, t.start_ms);
        put_u16(out, t.kind);
        put_u16(out, t.formation);
        first += static_cast<std::uint32_t>(t.records.size());
    }
    for (const auto& e : image.return_table) {
        put_u32(out, e.from_ms);
        put_u32(out, e.to_ms);
        put_u16(out, e.track);
        put_u16(out, 0);
    }
    for (const auto& t : image.tracks) {
        for (const auto& r : t.records) {
            put_u32(out, r.time_ms);
            put_i16(out, r.pos_x_cm);
            put_i16(out, r.pos_y_cm);
            put_i16(out, r.pos_z_cm);
            put_i16(out, r.vel_x_mms);
            put_i16(out, r.vel_y_mms);
            put_i16(out, r.vel_z_mms);
            put_u8(out, r.color_r);
            put_u8(out, r.color_g);
            put_u8(out, r.color_b);
        }
    }
    put_u32(out, crc32_ieee(std::span<const std::uint8_t>(out.data(), out.size())));
    return out;
}

std::vector<std::uint8_t> encode_flight_file(std::uint16_t drone_id, const std::vector<TrajectoryRecord>& records,
                                             std::uint16_t sampling_dt_ms) {
    FlightImage image;
    image.drone_id = drone_id;
    image.sampling_dt_ms = sampling_dt_ms;
    image.tracks.push_back(FlightTrack{FLIGHT_TRACK_SHOW, FLIGHT_NO_FORMATION, 0, records});
    return encode_flight_file(image);
}

void write_file_atomically(const std::filesystem::path& path, const std::vector<std::uint8_t>& bytes) {
    std::filesystem::path tmp = path;
    tmp += ".tmp";
    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        if (!out) throw std::runtime_error("cannot write " + tmp.string());
        out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        if (!out) throw std::runtime_error("write failed for " + tmp.string());
    }
    std::error_code ec;
    std::filesystem::rename(tmp, path, ec);
    if (ec) {
        std::filesystem::remove(tmp);
        throw std::runtime_error("cannot move " + tmp.string() + " to " + path.string() + ": " + ec.message());
    }
}

std::filesystem::path flight_file_name(const std::filesystem::path& dir, int drone_id) {
    return dir / ("drone_" + std::to_string(drone_id) + ".bin");
}

VerifyResult verify_flight_image(const std::vector<std::uint8_t>& bytes) {
    VerifyResult r;
    r.file_size = bytes.size();
    auto fail = [&r](std::string msg) {
        r.ok = false;
        r.error = std::move(msg);
        return r;
    };
    if (bytes.size() < FLIGHT_FILE_V1_HEADER_SIZE + FLIGHT_FILE_FOOTER_SIZE) return fail("file too short");

    const std::uint8_t* p = bytes.data();
    r.header.magic_number = get_u32(p);
    r.header.file_version = get_u16(p + 4);
    r.header.drone_id = get_u16(p + 6);
    r.header.total_records = get_u32(p + 8);
    r.header.sampling_dt_ms = get_u16(p + 12);
    r.version = r.header.file_version;
    if (r.header.magic_number != FLIGHT_FILE_MAGIC) return fail("bad magic number");

    std::uint64_t expected = 0;
    if (r.version == FLIGHT_FILE_VERSION_1) {
        // Version 1: a 16-byte header and the show's records only.
        r.header.reserved = get_u16(p + 14);
        r.header.track_count = 1;
        r.header.return_entry_count = 0;
        r.tracks.push_back(FlightTrackEntry{0, r.header.total_records, 0, FLIGHT_TRACK_SHOW, FLIGHT_NO_FORMATION});
        expected = FLIGHT_FILE_V1_HEADER_SIZE + static_cast<std::uint64_t>(FLIGHT_FILE_RECORD_SIZE) * r.header.total_records +
                   FLIGHT_FILE_FOOTER_SIZE;
    } else if (r.version == FLIGHT_FILE_VERSION) {
        if (bytes.size() < FLIGHT_FILE_HEADER_SIZE + FLIGHT_FILE_FOOTER_SIZE) return fail("file too short");
        r.header.track_count = get_u16(p + 14);
        r.header.return_entry_count = get_u16(p + 16);
        r.header.reserved = get_u16(p + 18);
        r.header.pack_id = get_u32(p + 20);
        expected = FLIGHT_FILE_HEADER_SIZE + static_cast<std::uint64_t>(FLIGHT_TRACK_ENTRY_SIZE) * r.header.track_count +
                   static_cast<std::uint64_t>(FLIGHT_RETURN_ENTRY_SIZE) * r.header.return_entry_count +
                   static_cast<std::uint64_t>(FLIGHT_FILE_RECORD_SIZE) * r.header.total_records + FLIGHT_FILE_FOOTER_SIZE;
    } else {
        return fail("unsupported file_version");
    }
    if (r.header.sampling_dt_ms == 0) return fail("sampling_dt_ms is 0");
    if (r.header.reserved != 0) return fail("reserved field is not 0");
    if (bytes.size() != expected) {
        return fail("size " + std::to_string(bytes.size()) + " != " + std::to_string(expected) + " for " +
                    std::to_string(r.header.track_count) + " track(s), " + std::to_string(r.header.return_entry_count) +
                    " return entries and " + std::to_string(r.header.total_records) + " records");
    }

    const std::size_t body = bytes.size() - FLIGHT_FILE_FOOTER_SIZE;
    r.stored_crc = get_u32(p + body);
    r.computed_crc = crc32_ieee(std::span<const std::uint8_t>(p, body));
    if (r.stored_crc != r.computed_crc) return fail("CRC-32 mismatch");

    if (r.version == FLIGHT_FILE_VERSION) {
        if (r.header.track_count == 0) return fail("no tracks");
        const std::uint8_t* q = p + FLIGHT_FILE_HEADER_SIZE;
        std::uint32_t next = 0;
        for (std::uint16_t i = 0; i < r.header.track_count; ++i, q += FLIGHT_TRACK_ENTRY_SIZE) {
            FlightTrackEntry t{get_u32(q), get_u32(q + 4), get_u32(q + 8), get_u16(q + 12), get_u16(q + 14)};
            if (t.first_record != next) return fail("track " + std::to_string(i) + " doesn't follow the previous one");
            if (t.record_count == 0) return fail("track " + std::to_string(i) + " has no records");
            if ((i == 0) != (t.kind == FLIGHT_TRACK_SHOW)) return fail("track 0, and only track 0, must be the show");
            if (t.kind > FLIGHT_TRACK_ABORT_POINT) return fail("track " + std::to_string(i) + " has an unknown kind");
            next += t.record_count;
            r.tracks.push_back(t);
        }
        if (next != r.header.total_records) return fail("the tracks' records don't add up to total_records");
        std::uint32_t from = 0;
        for (std::uint16_t i = 0; i < r.header.return_entry_count; ++i, q += FLIGHT_RETURN_ENTRY_SIZE) {
            FlightReturnEntry e{get_u32(q), get_u32(q + 4), get_u16(q + 8), get_u16(q + 10)};
            const std::string where = "return entry " + std::to_string(i);
            if (e.from_ms != from) return fail(where + " doesn't start where the previous one ends");
            if (e.to_ms <= e.from_ms) return fail(where + " is empty");
            if (e.track != FLIGHT_TRACK_NONE && e.track >= r.header.track_count) return fail(where + " names no track");
            if (e.track != FLIGHT_TRACK_NONE && e.track != 0 && r.tracks[e.track].kind == FLIGHT_TRACK_SHOW) {
                return fail(where + " names the show as a return");
            }
            if (e.reserved != 0) return fail(where + ": reserved field is not 0");
            from = e.to_ms;
            r.return_table.push_back(e);
        }
        if (!r.return_table.empty() && from != FLIGHT_TIME_END) return fail("the return table doesn't run to the end");
    }

    const std::uint8_t* rec = p + records_offset(r);
    for (std::size_t i = 0; i < r.tracks.size(); ++i) {
        for (std::uint32_t k = 0; k < r.tracks[i].record_count; ++k) {
            const std::uint32_t t = get_u32(rec + static_cast<std::size_t>(r.tracks[i].first_record + k) * FLIGHT_FILE_RECORD_SIZE);
            if (t != k * static_cast<std::uint32_t>(r.header.sampling_dt_ms)) {
                return fail("track " + std::to_string(i) + " record " + std::to_string(k) +
                            " time_ms is not k * sampling_dt_ms");
            }
        }
    }
    r.ok = true;
    return r;
}

VerifyResult verify_flight_file(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        VerifyResult r;
        r.error = "cannot open " + path.string();
        return r;
    }
    std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    return verify_flight_image(bytes);
}

FlightImage decode_flight_image(const std::vector<std::uint8_t>& bytes) {
    const VerifyResult v = verify_flight_image(bytes);
    if (!v.ok) throw std::runtime_error("decode_flight_image: " + v.error);
    FlightImage image;
    image.drone_id = v.header.drone_id;
    image.sampling_dt_ms = v.header.sampling_dt_ms;
    image.pack_id = v.header.pack_id;
    image.return_table = v.return_table;
    const std::uint8_t* rec = bytes.data() + records_offset(v);
    for (const auto& t : v.tracks) {
        FlightTrack track{t.kind, t.formation, t.start_ms, {}};
        track.records.reserve(t.record_count);
        for (std::uint32_t k = 0; k < t.record_count; ++k) {
            track.records.push_back(get_record(rec + static_cast<std::size_t>(t.first_record + k) * FLIGHT_FILE_RECORD_SIZE));
        }
        image.tracks.push_back(std::move(track));
    }
    return image;
}

std::vector<TrajectoryRecord> decode_records(const std::vector<std::uint8_t>& bytes) {
    return decode_flight_image(bytes).tracks.front().records;
}

}  // namespace flight_packer
