"""Python reader/verifier for drone_<id>.bin flight files.

Mirrors packer/include/flight_binary_spec.h (docs/3-phase-3.md §4.2) with
NumPy structured dtypes; the CRC is zlib.crc32, which is exactly
CRC-32-IEEE. Used by the tests to cross-check the C++ packer and by tooling
that needs to inspect flight files without the C++ build.
"""

from __future__ import annotations

import zlib
from dataclasses import dataclass
from pathlib import Path

import numpy as np

MAGIC = 0x44534857
VERSION = 0x0101
HEADER_SIZE = 16
RECORD_SIZE = 19
FOOTER_SIZE = 4

HEADER_DTYPE = np.dtype([
    ("magic_number", "<u4"),
    ("file_version", "<u2"),
    ("drone_id", "<u2"),
    ("total_records", "<u4"),
    ("sampling_dt_ms", "<u2"),
    ("reserved", "<u2"),
])

RECORD_DTYPE = np.dtype([
    ("time_ms", "<u4"),
    ("pos_x_cm", "<i2"), ("pos_y_cm", "<i2"), ("pos_z_cm", "<i2"),
    ("vel_x_mms", "<i2"), ("vel_y_mms", "<i2"), ("vel_z_mms", "<i2"),
    ("color_r", "u1"), ("color_g", "u1"), ("color_b", "u1"),
])

assert HEADER_DTYPE.itemsize == HEADER_SIZE and RECORD_DTYPE.itemsize == RECORD_SIZE


class FlightFileError(ValueError):
    pass


def expected_file_size(records: int) -> int:
    return HEADER_SIZE + RECORD_SIZE * records + FOOTER_SIZE


@dataclass
class FlightFile:
    drone_id: int
    sampling_dt_ms: int
    crc32: int
    records: np.ndarray  # structured, RECORD_DTYPE

    @property
    def times_sec(self) -> np.ndarray:
        return self.records["time_ms"].astype(np.float64) / 1000.0

    @property
    def positions_m(self) -> np.ndarray:
        r = self.records
        return np.stack([r["pos_x_cm"], r["pos_y_cm"], r["pos_z_cm"]], axis=1).astype(np.float64) / 100.0

    @property
    def velocities_mps(self) -> np.ndarray:
        r = self.records
        return np.stack([r["vel_x_mms"], r["vel_y_mms"], r["vel_z_mms"]], axis=1).astype(np.float64) / 1000.0

    @property
    def colors(self) -> np.ndarray:
        r = self.records
        return np.stack([r["color_r"], r["color_g"], r["color_b"]], axis=1)


def parse_flight_bytes(data: bytes) -> FlightFile:
    if len(data) < HEADER_SIZE + FOOTER_SIZE:
        raise FlightFileError("file too short")
    header = np.frombuffer(data, HEADER_DTYPE, count=1)[0]
    if header["magic_number"] != MAGIC:
        raise FlightFileError(f"bad magic 0x{int(header['magic_number']):08x}")
    if header["file_version"] != VERSION:
        raise FlightFileError(f"unsupported file_version 0x{int(header['file_version']):04x}")
    count = int(header["total_records"])
    if len(data) != expected_file_size(count):
        raise FlightFileError(f"size {len(data)} != 20 + 19 * {count}")
    stored = int.from_bytes(data[-FOOTER_SIZE:], "little")
    computed = zlib.crc32(data[:-FOOTER_SIZE]) & 0xFFFFFFFF
    if stored != computed:
        raise FlightFileError(f"CRC-32 mismatch: stored 0x{stored:08x}, computed 0x{computed:08x}")
    records = np.frombuffer(data, RECORD_DTYPE, count=count, offset=HEADER_SIZE).copy()
    return FlightFile(drone_id=int(header["drone_id"]), sampling_dt_ms=int(header["sampling_dt_ms"]),
                      crc32=stored, records=records)


def read_flight_file(path: str | Path) -> FlightFile:
    return parse_flight_bytes(Path(path).read_bytes())
