/*
 * Drone flight file layout, drone_<id>.bin (docs/3-phase-3.md §4.2), version 2.
 *
 * Shared by the Phase 3 packer, the drone firmware (STM32 / ESP32) and the
 * Phase 4 gateway, so this header is plain C99/C11 + C++ compatible.
 *
 *   [FlightFileHeader   24 B]
 *   [FlightTrackEntry   16 B] x track_count          track 0 is the show
 *   [FlightReturnEntry  12 B] x return_entry_count   the return table
 *   [TrajectoryRecord   19 B] x total_records        every track's records, track by track
 *   [uint32_t crc32      4 B]  CRC-32-IEEE over everything before it
 *
 *   file size = 24 + 16 * tracks + 12 * entries + 19 * total_records + 4 bytes
 *
 * Tracks (docs/4-condition_simulator.md §8.4): the show, then the planned
 * return paths to the holding area -- from a formation (or the takeoff flown
 * backwards) or from a moment inside a transition (an abort point). A track's
 * records are timed from its own start: record k is at k * sampling_dt_ms
 * after `start_ms` of show time.
 *
 * Return table: when the return command reaches the drone at show time u,
 * take the entry with from_ms <= u < to_ms. Track 0 means keep flying the
 * show (it ends with the show's own return leg, or it has already landed);
 * FLIGHT_TRACK_NONE means no planned way home. Otherwise keep flying the
 * show until max(u, start_ms) of that track, then fly the track from its
 * first record: it starts where the show is at start_ms, and when u is later
 * the drone is holding there (at a formation), so it starts the return then.
 * Entries are sorted and
 * contiguous from 0; the last ends at FLIGHT_TIME_END. Every drone's file of
 * one pack has the same table and the same pack_id, so the whole fleet picks
 * the same return.
 *
 * Version 1 files (0x0101: a 16-byte header, the show's records only) are
 * still accepted by the readers.
 *
 * All multi-byte fields are little-endian (native on x86 and ARM Cortex-M),
 * so firmware can map these structs straight onto flash.
 */
#ifndef DRONE_SHOW_FLIGHT_BINARY_SPEC_H
#define DRONE_SHOW_FLIGHT_BINARY_SPEC_H

#include <stdint.h>

#define FLIGHT_FILE_MAGIC 0x44534857u         /* "DSHW" */
#define FLIGHT_FILE_VERSION 0x0200u
#define FLIGHT_FILE_VERSION_1 0x0101u         /* show only, 16-byte header; still readable */
#define FLIGHT_FILE_SAMPLING_DT_MS 50u        /* 20 Hz */
#define FLIGHT_FILE_HEADER_SIZE 24u
#define FLIGHT_FILE_V1_HEADER_SIZE 16u
#define FLIGHT_TRACK_ENTRY_SIZE 16u
#define FLIGHT_RETURN_ENTRY_SIZE 12u
#define FLIGHT_FILE_RECORD_SIZE 19u
#define FLIGHT_FILE_FOOTER_SIZE 4u
#define FLIGHT_FILE_SIZE(tracks, entries, records)                                                   \
    (FLIGHT_FILE_HEADER_SIZE + FLIGHT_TRACK_ENTRY_SIZE * (uint32_t)(tracks) +                        \
     FLIGHT_RETURN_ENTRY_SIZE * (uint32_t)(entries) + FLIGHT_FILE_RECORD_SIZE * (uint32_t)(records) + \
     FLIGHT_FILE_FOOTER_SIZE)
#define FLIGHT_FILE_V1_SIZE(records) \
    (FLIGHT_FILE_V1_HEADER_SIZE + FLIGHT_FILE_RECORD_SIZE * (uint32_t)(records) + FLIGHT_FILE_FOOTER_SIZE)

/* FlightTrackEntry.kind */
#define FLIGHT_TRACK_SHOW 0u
#define FLIGHT_TRACK_RETURN 1u                /* from a formation, or the takeoff flown backwards */
#define FLIGHT_TRACK_ABORT_POINT 2u           /* from a moment inside the transition into `formation` */
#define FLIGHT_NO_FORMATION 0xFFFFu           /* the show track's formation */

/* FlightReturnEntry.track / .to_ms */
#define FLIGHT_TRACK_NONE 0xFFFFu             /* no planned way home */
#define FLIGHT_TIME_END 0xFFFFFFFFu           /* to the end of time */

/* Quantization scales (§4.1). */
#define FLIGHT_POSITION_UNITS_PER_M 100.0     /* 1 cm   */
#define FLIGHT_VELOCITY_UNITS_PER_MPS 1000.0  /* 1 mm/s */

#pragma pack(push, 1)

typedef struct {
    uint32_t magic_number;       /* FLIGHT_FILE_MAGIC */
    uint16_t file_version;       /* FLIGHT_FILE_VERSION */
    uint16_t drone_id;           /* 0 <= id < N */
    uint32_t total_records;      /* all tracks */
    uint16_t sampling_dt_ms;     /* FLIGHT_FILE_SAMPLING_DT_MS */
    uint16_t track_count;        /* >= 1 */
    uint16_t return_entry_count; /* 0: no return table packed */
    uint16_t reserved;           /* 0 */
    uint32_t pack_id;            /* the same in every file of one pack */
} FlightFileHeader;

typedef struct {
    uint32_t first_record;       /* index into the records */
    uint32_t record_count;
    uint32_t start_ms;           /* show time the track starts at (0 for the show) */
    uint16_t kind;               /* FLIGHT_TRACK_SHOW / _RETURN / _ABORT_POINT */
    uint16_t formation;          /* keyframe index; FLIGHT_NO_FORMATION for the show */
} FlightTrackEntry;

typedef struct {
    uint32_t from_ms;            /* return command at show time u: from_ms <= u < to_ms */
    uint32_t to_ms;              /* FLIGHT_TIME_END for the last entry */
    uint16_t track;              /* 0 = keep flying the show; FLIGHT_TRACK_NONE = no planned way home */
    uint16_t reserved;           /* 0 */
} FlightReturnEntry;

/* Version 1 header (0x0101), kept for reading older files. */
typedef struct {
    uint32_t magic_number;
    uint16_t file_version;
    uint16_t drone_id;
    uint32_t total_records;
    uint16_t sampling_dt_ms;
    uint16_t reserved;
} FlightFileHeaderV1;

typedef struct {
    uint32_t time_ms;   /* ms since the track's start */
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

FLIGHT_STATIC_ASSERT(sizeof(FlightFileHeader) == FLIGHT_FILE_HEADER_SIZE, "FlightFileHeader must be 24 bytes");
FLIGHT_STATIC_ASSERT(sizeof(FlightFileHeaderV1) == FLIGHT_FILE_V1_HEADER_SIZE, "FlightFileHeaderV1 must be 16 bytes");
FLIGHT_STATIC_ASSERT(sizeof(FlightTrackEntry) == FLIGHT_TRACK_ENTRY_SIZE, "FlightTrackEntry must be 16 bytes");
FLIGHT_STATIC_ASSERT(sizeof(FlightReturnEntry) == FLIGHT_RETURN_ENTRY_SIZE, "FlightReturnEntry must be 12 bytes");
FLIGHT_STATIC_ASSERT(sizeof(TrajectoryRecord) == FLIGHT_FILE_RECORD_SIZE, "TrajectoryRecord must be 19 bytes");

#endif /* DRONE_SHOW_FLIGHT_BINARY_SPEC_H */
