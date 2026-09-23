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

}  // namespace

std::vector<std::uint8_t> encode_flight_file(std::uint16_t drone_id, const std::vector<TrajectoryRecord>& records,
                                             std::uint16_t sampling_dt_ms) {
    std::vector<std::uint8_t> out;
    out.reserve(FLIGHT_FILE_SIZE(records.size()));
    // Serialized field by field (explicit little-endian), not memcpy'd, so the
    // image is identical whatever the host's endianness or struct ABI.
    put_u32(out, FLIGHT_FILE_MAGIC);
    put_u16(out, FLIGHT_FILE_VERSION);
    put_u16(out, drone_id);
    put_u32(out, static_cast<std::uint32_t>(records.size()));
    put_u16(out, sampling_dt_ms);
    put_u16(out, 0);
    for (const auto& r : records) {
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
    put_u32(out, crc32_ieee(std::span<const std::uint8_t>(out.data(), out.size())));
    return out;
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
    if (bytes.size() < FLIGHT_FILE_HEADER_SIZE + FLIGHT_FILE_FOOTER_SIZE) return fail("file too short");

    const std::uint8_t* p = bytes.data();
    r.header.magic_number = get_u32(p);
    r.header.file_version = get_u16(p + 4);
    r.header.drone_id = get_u16(p + 6);
    r.header.total_records = get_u32(p + 8);
    r.header.sampling_dt_ms = get_u16(p + 12);
    r.header.reserved = get_u16(p + 14);

    if (r.header.magic_number != FLIGHT_FILE_MAGIC) return fail("bad magic number");
    if (r.header.file_version != FLIGHT_FILE_VERSION) return fail("unsupported file_version");
    if (r.header.sampling_dt_ms == 0) return fail("sampling_dt_ms is 0");
    if (r.header.reserved != 0) return fail("reserved field is not 0");
    const std::uint64_t expected = FLIGHT_FILE_HEADER_SIZE +
                                   static_cast<std::uint64_t>(FLIGHT_FILE_RECORD_SIZE) * r.header.total_records +
                                   FLIGHT_FILE_FOOTER_SIZE;
    if (bytes.size() != expected) {
        return fail("size " + std::to_string(bytes.size()) + " != 20 + 19 * " +
                    std::to_string(r.header.total_records) + " = " + std::to_string(expected));
    }

    const std::size_t body = bytes.size() - FLIGHT_FILE_FOOTER_SIZE;
    r.stored_crc = get_u32(p + body);
    r.computed_crc = crc32_ieee(std::span<const std::uint8_t>(p, body));
    if (r.stored_crc != r.computed_crc) return fail("CRC-32 mismatch");

    for (std::uint32_t k = 0; k < r.header.total_records; ++k) {
        const std::uint32_t t = get_u32(p + FLIGHT_FILE_HEADER_SIZE + k * FLIGHT_FILE_RECORD_SIZE);
        if (t != k * static_cast<std::uint32_t>(r.header.sampling_dt_ms)) {
            return fail("record " + std::to_string(k) + " time_ms is not k * sampling_dt_ms");
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

std::vector<TrajectoryRecord> decode_records(const std::vector<std::uint8_t>& bytes) {
    const VerifyResult v = verify_flight_image(bytes);
    if (!v.ok) throw std::runtime_error("decode_records: " + v.error);
    std::vector<TrajectoryRecord> out(v.header.total_records);
    const std::uint8_t* p = bytes.data() + FLIGHT_FILE_HEADER_SIZE;
    for (auto& r : out) {
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
        p += FLIGHT_FILE_RECORD_SIZE;
    }
    return out;
}

}  // namespace flight_packer
