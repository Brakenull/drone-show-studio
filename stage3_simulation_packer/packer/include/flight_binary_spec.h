/*
 * Drone flight file layout, drone_<id>.bin (docs/3-phase-3.md §4.2).
 *
 * Shared by the Phase 3 packer, the drone firmware (STM32 / ESP32) and the
 * Phase 4 gateway, so this header is plain C99/C11 + C++ compatible.
 *
 *   [FlightFileHeader  16 B]
 *   [TrajectoryRecord  19 B] x total_records
 *   [uint32_t crc32     4 B]  CRC-32-IEEE over header + all records
 *
 *   file size = 20 + 19 * total_records bytes
 *
 * All multi-byte fields are little-endian (native on x86 and ARM Cortex-M),
 * so firmware can map a TrajectoryRecord pointer straight onto flash.
 */
#ifndef DRONE_SHOW_FLIGHT_BINARY_SPEC_H
#define DRONE_SHOW_FLIGHT_BINARY_SPEC_H

#include <stdint.h>

#define FLIGHT_FILE_MAGIC 0x44534857u         /* "DSHW" */
#define FLIGHT_FILE_VERSION 0x0101u
#define FLIGHT_FILE_SAMPLING_DT_MS 50u        /* 20 Hz */
#define FLIGHT_FILE_HEADER_SIZE 16u
#define FLIGHT_FILE_RECORD_SIZE 19u
#define FLIGHT_FILE_FOOTER_SIZE 4u
#define FLIGHT_FILE_SIZE(records) \
    (FLIGHT_FILE_HEADER_SIZE + FLIGHT_FILE_RECORD_SIZE * (uint32_t)(records) + FLIGHT_FILE_FOOTER_SIZE)

/* Quantization scales (§4.1). */
#define FLIGHT_POSITION_UNITS_PER_M 100.0     /* 1 cm   */
#define FLIGHT_VELOCITY_UNITS_PER_MPS 1000.0  /* 1 mm/s */

#pragma pack(push, 1)

typedef struct {
    uint32_t magic_number;   /* FLIGHT_FILE_MAGIC */
    uint16_t file_version;   /* FLIGHT_FILE_VERSION */
    uint16_t drone_id;       /* 0 <= id < N */
    uint32_t total_records;  /* K */
    uint16_t sampling_dt_ms; /* FLIGHT_FILE_SAMPLING_DT_MS */
    uint16_t reserved;       /* 0 */
} FlightFileHeader;

typedef struct {
    uint32_t time_ms;   /* ms since show start */
    int16_t pos_x_cm;   /* ENU */
    int16_t pos_y_cm;
    int16_t pos_z_cm;
    int16_t vel_x_mms;  /* feedforward velocity */
    int16_t vel_y_mms;
    int16_t vel_z_mms;
    uint8_t color_r;
    uint8_t color_g;
    uint8_t color_b;
} TrajectoryRecord;

#pragma pack(pop)

#ifdef __cplusplus
#define FLIGHT_STATIC_ASSERT(cond, msg) static_assert(cond, msg)
#else
#define FLIGHT_STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)
#endif

FLIGHT_STATIC_ASSERT(sizeof(FlightFileHeader) == FLIGHT_FILE_HEADER_SIZE, "FlightFileHeader must be 16 bytes");
FLIGHT_STATIC_ASSERT(sizeof(TrajectoryRecord) == FLIGHT_FILE_RECORD_SIZE, "TrajectoryRecord must be 19 bytes");

#endif /* DRONE_SHOW_FLIGHT_BINARY_SPEC_H */
