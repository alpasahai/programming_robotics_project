"""Geometry helpers for world-frame target sensing.

Pure functions for transforming target positions to azimuth/elevation/range
and for computing shortest signed angular differences. Designed to be reused
by Search Radar and FCR (Fire Control Radar) tracking loops.
"""
import math
from typing import Sequence


def azimuth_elevation_range(
    observer: Sequence[float],
    target: Sequence[float],
) -> tuple[float, float, float]:
    """Convert target position to azimuth, elevation, and range.

    Computes the spherical coordinates of a target relative to an observer,
    using Z-up ENU (East-North-Up) convention:
        azimuth   = atan2(dx, dy)  — 0° is North (+Y), ±90° is East/West
        elevation = atan2(dz, √(dx² + dy²))  — 0° is horizon, ±90° is vertical
        range     = √(dx² + dy² + dz²)  — Euclidean distance

    Args:
        observer: 3-element sequence [x, y, z] of observer position (metres).
        target: 3-element sequence [x, y, z] of target position (metres).

    Returns:
        Tuple (azimuth, elevation, range) where:
            - azimuth is in radians, range [-π, π]
            - elevation is in radians, range [-π/2, π/2]
            - range is in metres (always ≥ 0)
    """
    dx = target[0] - observer[0]
    dy = target[1] - observer[1]
    dz = target[2] - observer[2]

    azimuth = math.atan2(dx, dy)
    horiz_dist = math.sqrt(dx * dx + dy * dy)
    elevation = math.atan2(dz, horiz_dist)
    range_dist = math.sqrt(dx * dx + dy * dy + dz * dz)

    return azimuth, elevation, range_dist


def angle_diff(a: float, b: float) -> float:
    """Compute the shortest signed difference between two angles.

    Returns `a - b` wrapped to the interval [-π, π] using the atan2 identity:
        wrapped_diff = atan2(sin(diff), cos(diff))

    This ensures the result always represents the shortest rotational path
    from b to a, never exceeding π radians in magnitude.

    Args:
        a: First angle in radians.
        b: Second angle in radians.

    Returns:
        Shortest signed difference in radians, always in [-π, π].
    """
    diff = a - b
    return math.atan2(math.sin(diff), math.cos(diff))
