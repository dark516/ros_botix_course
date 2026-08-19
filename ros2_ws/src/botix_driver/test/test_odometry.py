# Copyright (c) 2026
# SPDX-License-Identifier: GPL-3.0-or-later

import math
from types import SimpleNamespace

import pytest

from botix_driver.odometry import DifferentialDriveOdometry, wheel_distances


def test_first_sample_establishes_baseline_without_motion():
    odometry = DifferentialDriveOdometry(wheel_separation=0.2, wheel_radius=0.05)

    assert odometry.update(1.0, 1.0, 0.1) is None
    assert odometry.pose == pytest.approx((0.0, 0.0, 0.0))


def test_straight_distance_and_velocity_use_calibrated_metres():
    odometry = DifferentialDriveOdometry(wheel_separation=0.2, wheel_radius=0.05)
    odometry.update(1.0, 1.0, 0.1)

    update = odometry.update(1.2, 1.2, 0.5)

    assert update.pose == pytest.approx((0.2, 0.0, 0.0))
    assert update.linear_velocity == pytest.approx(0.4)
    assert update.angular_velocity == pytest.approx(0.0)
    assert update.wheel_positions == pytest.approx((24.0, 24.0))
    assert update.wheel_velocities == pytest.approx((8.0, 8.0))


def test_opposite_wheel_motion_rotates_in_place():
    odometry = DifferentialDriveOdometry(wheel_separation=0.2, wheel_radius=0.05)
    odometry.update(0.0, 0.0, 0.1)

    update = odometry.update(-0.1, 0.1, 0.5)

    assert update.pose == pytest.approx((0.0, 0.0, 1.0))
    assert update.angular_velocity == pytest.approx(2.0)


def test_arc_is_integrated_about_its_midpoint():
    odometry = DifferentialDriveOdometry(wheel_separation=0.2, wheel_radius=0.05)
    odometry.update(0.0, 0.0, 0.1)

    update = odometry.update(0.1, 0.2, 1.0)

    assert update.pose[0] == pytest.approx(0.15 * math.cos(0.25))
    assert update.pose[1] == pytest.approx(0.15 * math.sin(0.25))
    assert update.pose[2] == pytest.approx(0.5)


def test_nonpositive_elapsed_time_does_not_advance_baseline():
    odometry = DifferentialDriveOdometry(wheel_separation=0.2, wheel_radius=0.05)
    odometry.update(0.0, 0.0, 0.1)

    assert odometry.update(0.1, 0.1, 0.0) is None
    update = odometry.update(0.2, 0.2, 1.0)

    assert update.pose[0] == pytest.approx(0.2)


def test_extracts_first_two_mavlink_wheel_distances():
    message = SimpleNamespace(count=2, distance=[1.25, -0.5] + [0.0] * 14)

    assert wheel_distances(message) == pytest.approx((1.25, -0.5))


@pytest.mark.parametrize(
    "message",
    [SimpleNamespace(count=1, distance=[1.0]), SimpleNamespace(count=2, distance=[1.0])],
)
def test_rejects_incomplete_mavlink_wheel_distances(message):
    assert wheel_distances(message) is None
