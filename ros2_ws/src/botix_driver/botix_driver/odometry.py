# Copyright (c) 2026
# SPDX-License-Identifier: GPL-3.0-or-later

"""Differential-drive integration for calibrated cumulative wheel distances."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any


def planar_covariance(
    x_variance: float, y_variance: float, yaw_variance: float
) -> list[float]:
    """Build a ROS covariance matrix for planar differential-drive data."""
    if min(x_variance, y_variance, yaw_variance) < 0.0:
        raise ValueError("variances must be non-negative")
    covariance = [0.0] * 36
    covariance[0] = x_variance
    covariance[7] = y_variance
    covariance[14] = 1e6
    covariance[21] = 1e6
    covariance[28] = 1e6
    covariance[35] = yaw_variance
    return covariance


@dataclass(frozen=True)
class OdometryUpdate:
    pose: tuple[float, float, float]
    linear_velocity: float
    angular_velocity: float
    wheel_positions: tuple[float, float]
    wheel_velocities: tuple[float, float]


def wheel_distances(message: Any) -> tuple[float, float] | None:
    """Return the first left/right distances from a MAVLink WHEEL_DISTANCE."""
    distances = message.distance
    if message.count < 2 or len(distances) < 2:
        return None
    return float(distances[0]), float(distances[1])


class DifferentialDriveOdometry:
    def __init__(self, wheel_separation: float, wheel_radius: float) -> None:
        if wheel_separation <= 0.0 or wheel_radius <= 0.0:
            raise ValueError("wheel geometry must be positive")
        self.wheel_separation = wheel_separation
        self.wheel_radius = wheel_radius
        self.pose = (0.0, 0.0, 0.0)
        self._previous: tuple[float, float] | None = None

    def update(
        self, left: float, right: float, elapsed: float
    ) -> OdometryUpdate | None:
        if self._previous is None:
            self._previous = (left, right)
            return None
        if elapsed <= 0.0:
            return None

        delta_left = left - self._previous[0]
        delta_right = right - self._previous[1]
        self._previous = (left, right)

        distance = (delta_left + delta_right) / 2.0
        rotation = (delta_right - delta_left) / self.wheel_separation
        x, y, yaw = self.pose
        heading = yaw + rotation / 2.0
        x += distance * math.cos(heading)
        y += distance * math.sin(heading)
        yaw = (yaw + rotation + math.pi) % (2.0 * math.pi) - math.pi
        self.pose = (x, y, yaw)

        return OdometryUpdate(
            pose=self.pose,
            linear_velocity=distance / elapsed,
            angular_velocity=rotation / elapsed,
            wheel_positions=(left / self.wheel_radius, right / self.wheel_radius),
            wheel_velocities=(
                delta_left / self.wheel_radius / elapsed,
                delta_right / self.wheel_radius / elapsed,
            ),
        )
