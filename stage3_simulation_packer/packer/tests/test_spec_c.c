/* Compiles flight_binary_spec.h as plain C11 (the firmware's view) and checks field offsets. */
#include <stddef.h>
#include <stdio.h>

#include "flight_binary_spec.h"

int main(void) {
    int ok = 1;
    ok &= offsetof(FlightFileHeader, file_version) == 4;
    ok &= offsetof(FlightFileHeader, drone_id) == 6;
    ok &= offsetof(FlightFileHeader, total_records) == 8;
    ok &= offsetof(FlightFileHeader, sampling_dt_ms) == 12;
    ok &= offsetof(FlightFileHeader, track_count) == 14;
    ok &= offsetof(FlightFileHeader, return_entry_count) == 16;
    ok &= offsetof(FlightFileHeader, reserved) == 18;
    ok &= offsetof(FlightFileHeader, pack_id) == 20;
    ok &= offsetof(FlightTrackEntry, record_count) == 4;
    ok &= offsetof(FlightTrackEntry, start_ms) == 8;
    ok &= offsetof(FlightTrackEntry, kind) == 12;
    ok &= offsetof(FlightTrackEntry, formation) == 14;
    ok &= offsetof(FlightReturnEntry, to_ms) == 4;
    ok &= offsetof(FlightReturnEntry, track) == 8;
    ok &= offsetof(FlightReturnEntry, reserved) == 10;
    ok &= offsetof(FlightFileHeaderV1, reserved) == 14;
    ok &= offsetof(TrajectoryRecord, pos_x_cm) == 4;
    ok &= offsetof(TrajectoryRecord, vel_x_mms) == 10;
    ok &= offsetof(TrajectoryRecord, color_r) == 16;
    ok &= offsetof(TrajectoryRecord, color_b) == 18;
    ok &= FLIGHT_FILE_SIZE(1, 0, 12000) == 228044u;
    ok &= FLIGHT_FILE_SIZE(3, 5, 100) == 24u + 48u + 60u + 1900u + 4u;
    ok &= FLIGHT_FILE_V1_SIZE(12000) == 228020u;
    printf(ok ? "test_spec_c: all checks passed\n" : "test_spec_c: FAILED\n");
    return ok ? 0 : 1;
}
