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
    ok &= offsetof(FlightFileHeader, reserved) == 14;
    ok &= offsetof(TrajectoryRecord, pos_x_cm) == 4;
    ok &= offsetof(TrajectoryRecord, vel_x_mms) == 10;
    ok &= offsetof(TrajectoryRecord, color_r) == 16;
    ok &= offsetof(TrajectoryRecord, color_b) == 18;
    ok &= FLIGHT_FILE_SIZE(12000) == 228020u;
    printf(ok ? "test_spec_c: all checks passed\n" : "test_spec_c: FAILED\n");
    return ok ? 0 : 1;
}
