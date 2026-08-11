# Copyright (c) 2026
# SPDX-License-Identifier: GPL-3.0-or-later

"""Decoder for LD06 / LD19 / D500 lidar frames arriving over UDP.

The firmware forwards UART bytes verbatim rather than parsing them, so all
framing knowledge lives here.  A frame is 47 bytes:

    0       header, 0x54
    1       ver_len, low 5 bits are the point count (12 in practice)
    2..3    rotation speed, u16 LE, degrees per second
    4..5    start angle, u16 LE, hundredths of a degree
    6..41   12 points of (distance u16 LE mm, intensity u8)
    42..43  end angle, u16 LE, hundredths of a degree
    44..45  timestamp, u16 LE ms
    46      CRC-8

Datagrams can be dropped or reordered, so the parser resynchronises on the
header rather than assuming it starts on a boundary.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field

HEADER = 0x54
POINTS_PER_FRAME = 12
FRAME_LENGTH = 47

# The datagram header the firmware prepends: magic 'L', version, sequence
DATAGRAM_HEADER = struct.Struct("<BBH")
DATAGRAM_MAGIC = ord("L")
DATAGRAM_VERSION = 1


def _crc_table() -> list[int]:
    """CRC-8 with polynomial 0x4D, as used by the LDROBOT firmware."""
    table = []
    for index in range(256):
        crc = index
        for _ in range(8):
            crc = ((crc << 1) ^ 0x4D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table.append(crc)
    return table


CRC_TABLE = _crc_table()


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc = CRC_TABLE[crc ^ byte]
    return crc


@dataclass
class Frame:
    """One decoded packet: twelve measurements spanning a small arc."""

    speed_deg_s: int
    start_angle_deg: float
    end_angle_deg: float
    timestamp_ms: int
    # (angle in degrees, distance in metres, intensity 0..255)
    points: list[tuple[float, float, int]]


@dataclass
class Stats:
    frames: int = 0
    crc_errors: int = 0
    resyncs: int = 0
    dropped_datagrams: int = 0
    bytes_in: int = 0


@dataclass
class FrameParser:
    """Accumulates bytes and yields whole frames.

    CRC failures are counted and the frame discarded, but a run of them is not
    treated as fatal: a clone with a different polynomial would otherwise look
    identical to a dead sensor, and the counters make the difference visible.
    """

    stats: Stats = field(default_factory=Stats)
    _buffer: bytearray = field(default_factory=bytearray)
    _expected_sequence: int | None = None

    def feed_datagram(self, datagram: bytes) -> list[Frame]:
        """Strip the firmware header, note any loss, then decode the payload."""
        if len(datagram) < DATAGRAM_HEADER.size:
            return []

        magic, version, sequence = DATAGRAM_HEADER.unpack_from(datagram)

        if magic != DATAGRAM_MAGIC or version != DATAGRAM_VERSION:
            # Not ours; most likely something else is sending to this port
            return []

        if self._expected_sequence is not None and sequence != self._expected_sequence:
            missing = (sequence - self._expected_sequence) & 0xFFFF
            self.stats.dropped_datagrams += missing
            # A gap means the byte stream has a hole: drop the partial frame
            self._buffer.clear()

        self._expected_sequence = (sequence + 1) & 0xFFFF

        return self.feed(datagram[DATAGRAM_HEADER.size :])

    def feed(self, data: bytes) -> list[Frame]:
        self.stats.bytes_in += len(data)
        self._buffer.extend(data)

        frames = []

        while True:
            start = self._buffer.find(HEADER)

            if start < 0:
                # Nothing usable; keep the tail in case a header straddles reads
                self._buffer.clear()
                break

            if start > 0:
                self.stats.resyncs += 1
                del self._buffer[:start]

            if len(self._buffer) < FRAME_LENGTH:
                break

            candidate = bytes(self._buffer[:FRAME_LENGTH])

            if crc8(candidate[:-1]) != candidate[-1]:
                self.stats.crc_errors += 1
                # Not a real frame boundary: step past this header and retry
                del self._buffer[:1]
                continue

            del self._buffer[:FRAME_LENGTH]

            frame = _decode(candidate)
            if frame is not None:
                self.stats.frames += 1
                frames.append(frame)

        return frames


def _decode(raw: bytes) -> Frame | None:
    count = raw[1] & 0x1F

    if count != POINTS_PER_FRAME:
        return None

    speed, start_raw = struct.unpack_from("<HH", raw, 2)
    end_raw, timestamp = struct.unpack_from("<HH", raw, 42)

    start_deg = start_raw / 100.0
    end_deg = end_raw / 100.0

    # The arc wraps through zero on roughly one frame per revolution
    span = (end_deg - start_deg) % 360.0
    step = span / (count - 1) if count > 1 else 0.0

    points = []
    for index in range(count):
        distance_mm, intensity = struct.unpack_from("<HB", raw, 6 + index * 3)
        angle = (start_deg + step * index) % 360.0
        points.append((angle, distance_mm / 1000.0, intensity))

    return Frame(
        speed_deg_s=speed,
        start_angle_deg=start_deg,
        end_angle_deg=end_deg,
        timestamp_ms=timestamp,
        points=points,
    )


class ScanAssembler:
    """Bins frames into fixed-width rays and emits one sweep per revolution.

    LaserScan wants evenly spaced rays, while the sensor delivers whatever
    angles the rotation happens to land on, so measurements are dropped into
    bins.  A sweep is closed when the angle wraps rather than after a fixed
    number of frames: the rotation rate drifts with battery voltage and load.
    """

    def __init__(self, bins: int, range_min: float, range_max: float) -> None:
        self.bins = bins
        self.range_min = range_min
        self.range_max = range_max
        self._ranges = [math.inf] * bins
        self._intensities = [0.0] * bins
        self._last_angle: float | None = None
        self._populated = 0

    @property
    def angle_increment(self) -> float:
        return 2.0 * math.pi / self.bins

    def add(self, frame: Frame) -> tuple[list[float], list[float]] | None:
        """Return (ranges, intensities) once a full revolution is complete."""
        completed = None

        for angle_deg, distance, intensity in frame.points:
            if self._last_angle is not None and angle_deg < self._last_angle - 180.0:
                completed = self._finish()

            self._last_angle = angle_deg

            index = int(angle_deg / 360.0 * self.bins) % self.bins

            if self.range_min <= distance <= self.range_max:
                self._ranges[index] = distance
                self._intensities[index] = float(intensity)
                self._populated += 1

        return completed

    def _finish(self) -> tuple[list[float], list[float]]:
        ranges = self._ranges
        intensities = self._intensities

        self._ranges = [math.inf] * self.bins
        self._intensities = [0.0] * self.bins
        self._populated = 0

        return ranges, intensities
