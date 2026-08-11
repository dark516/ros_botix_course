# Copyright (c) 2026
# SPDX-License-Identifier: GPL-3.0-or-later

"""Frame decoding tests, run without a sensor or a ROS graph."""

import math
import struct

from botix_driver.lidar import (
    DATAGRAM_HEADER,
    DATAGRAM_MAGIC,
    DATAGRAM_VERSION,
    FrameParser,
    ScanAssembler,
    crc8,
)


def make_frame(start_deg, end_deg, distances_mm, intensity=200):
    body = bytearray([0x54, 0x2C])
    body += struct.pack("<HH", 3600, int(start_deg * 100))
    for distance in distances_mm:
        body += struct.pack("<HB", distance, intensity)
    body += struct.pack("<HH", int(end_deg * 100), 1234)
    body.append(crc8(bytes(body)))
    assert len(body) == 47
    return bytes(body)


def wrap(sequence, payload):
    return DATAGRAM_HEADER.pack(DATAGRAM_MAGIC, DATAGRAM_VERSION, sequence) + payload


def parse_one(raw):
    return FrameParser().feed(raw)[0]


def test_decodes_a_clean_frame():
    parser = FrameParser()
    frames = parser.feed(make_frame(10.0, 21.0, range(100, 1300, 100)))

    assert len(frames) == 1
    assert parser.stats.crc_errors == 0

    points = frames[0].points
    assert points[0][0] == 10.0
    assert points[-1][0] == 21.0
    assert points[0][1] == 0.100  # millimetres become metres


def test_frame_split_across_reads():
    parser = FrameParser()
    raw = make_frame(0, 11, [500] * 12)

    assert parser.feed(raw[:20]) == []
    assert len(parser.feed(raw[20:])) == 1


def test_resynchronises_after_garbage():
    parser = FrameParser()

    assert len(parser.feed(b"\xde\xad\xbe\xef" + make_frame(0, 11, [500] * 12))) == 1
    assert parser.stats.resyncs >= 1


def test_rejects_a_corrupted_frame():
    parser = FrameParser()
    corrupted = bytearray(make_frame(0, 11, [500] * 12))
    corrupted[-1] ^= 0xFF

    assert parser.feed(bytes(corrupted)) == []
    assert parser.stats.crc_errors == 1


def test_header_byte_inside_payload_does_not_desynchronise():
    # 0x5454 as a distance puts the sync byte in the middle of the frame
    assert len(FrameParser().feed(make_frame(30, 41, [0x5454] * 12))) == 1


def test_datagram_loss_is_counted_and_discards_the_straddling_frame():
    parser = FrameParser()
    raw = make_frame(0, 11, [500] * 12)

    parser.feed_datagram(wrap(0, raw[:20]))
    assert parser.feed_datagram(wrap(5, raw[20:])) == []

    # sequences 1 through 4 never arrived
    assert parser.stats.dropped_datagrams == 4


def test_foreign_traffic_on_the_port_is_ignored():
    assert FrameParser().feed_datagram(b"\x00\x01\x02\x03whatever") == []


def test_sweep_is_emitted_once_the_angle_wraps():
    assembler = ScanAssembler(bins=360, range_min=0.02, range_max=12.0)

    for base in range(0, 360, 12):
        assert assembler.add(parse_one(make_frame(base, base + 11, [1000] * 12))) is None

    completed = assembler.add(parse_one(make_frame(0, 11, [1000] * 12)))
    assert completed is not None

    ranges, _ = completed
    assert len(ranges) == 360
    assert sum(1 for value in ranges if value != math.inf) > 300


def test_absent_returns_stay_infinite():
    # A zero reading means no echo, not an obstacle at the sensor
    assembler = ScanAssembler(bins=36, range_min=0.02, range_max=12.0)
    assembler.add(parse_one(make_frame(0, 11, [0] * 12)))

    ranges, _ = assembler._finish()
    assert all(value == math.inf for value in ranges)
