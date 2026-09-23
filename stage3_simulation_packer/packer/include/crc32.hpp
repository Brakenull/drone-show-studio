#pragma once

// CRC-32-IEEE 802.3 (reflected polynomial 0xEDB88320, init 0xFFFFFFFF,
// final XOR 0xFFFFFFFF) -- the same CRC as zlib.crc32 / Ethernet / PNG.
// Check value: crc32("123456789") == 0xCBF43926.

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace flight_packer {

namespace detail {

constexpr std::array<std::uint32_t, 256> make_crc32_table() {
    std::array<std::uint32_t, 256> table{};
    for (std::uint32_t i = 0; i < 256; ++i) {
        std::uint32_t c = i;
        for (int k = 0; k < 8; ++k) c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        table[i] = c;
    }
    return table;
}

inline constexpr std::array<std::uint32_t, 256> kCrc32Table = make_crc32_table();

}  // namespace detail

// Incremental form: start from crc32_init(), feed chunks, finish with crc32_final().
constexpr std::uint32_t crc32_init() { return 0xFFFFFFFFu; }

constexpr std::uint32_t crc32_update(std::uint32_t state, std::span<const std::uint8_t> bytes) {
    for (std::uint8_t b : bytes) state = detail::kCrc32Table[(state ^ b) & 0xFFu] ^ (state >> 8);
    return state;
}

constexpr std::uint32_t crc32_final(std::uint32_t state) { return state ^ 0xFFFFFFFFu; }

constexpr std::uint32_t crc32_ieee(std::span<const std::uint8_t> bytes) {
    return crc32_final(crc32_update(crc32_init(), bytes));
}

}  // namespace flight_packer
