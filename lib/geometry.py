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
            - azimuth is in radians, range (-π, π]
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


def angular_separation(
    az1: float,
    el1: float,
    az2: float,
    el2: float,
) -> float:
    """Compute the great-circle angular separation between two (az, el) directions.

    Converts each (azimuth, elevation) pair to a unit vector using the Z-up ENU
    convention consistent with ``azimuth_elevation_range``:
        x = sin(az) * cos(el)
        y = cos(az) * cos(el)
        z = sin(el)
    Then returns ``acos(clamp(dot(v1, v2), -1, 1))``, the angle between the
    two unit vectors in [0, π].

    Args:
        az1: Azimuth of first direction in radians.
        el1: Elevation of first direction in radians.
        az2: Azimuth of second direction in radians.
        el2: Elevation of second direction in radians.

    Returns:
        Angular separation in radians, always in [0, π].
    """
    x1 = math.sin(az1) * math.cos(el1)
    y1 = math.cos(az1) * math.cos(el1)
    z1 = math.sin(el1)

    x2 = math.sin(az2) * math.cos(el2)
    y2 = math.cos(az2) * math.cos(el2)
    z2 = math.sin(el2)

    dot = x1 * x2 + y1 * y2 + z1 * z2
    clamped = max(-1.0, min(1.0, dot))
    return math.acos(clamped)


def angle_diff(a: float, b: float) -> float:
    """Compute the shortest signed difference between two angles.

    Returns `a - b` wrapped to the interval (-π, π] using the atan2 identity:
        wrapped_diff = atan2(sin(diff), cos(diff))

    This ensures the result always represents the shortest rotational path
    from b to a, never exceeding π radians in magnitude.

    Args:
        a: First angle in radians.
        b: Second angle in radians.

    Returns:
        Shortest signed difference in radians, always in (-π, π].
    """
    diff = a - b
    return math.atan2(math.sin(diff), math.cos(diff))
