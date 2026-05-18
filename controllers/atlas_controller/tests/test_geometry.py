"""Tests for geometry — world-frame azimuth/elevation/range helpers.

Verifies azimuth_elevation_range() transforms a target position relative to an
observer into spherical coordinates (azimuth, elevation, range) in Z-up ENU
convention. Also tests angle_diff() for shortest signed angular difference
wrapped to (-π, π].
"""
import math
import pytest
from geometry import azimuth_elevation_range, angle_diff


# ---------------------------------------------------------------------------
# Slice 1: Basic azimuth angles (XY plane)
# ---------------------------------------------------------------------------

def test_target_due_north_has_zero_azimuth():
    """Target directly along +Y from observer → azimuth = 0."""
    observer = [0.0, 0.0, 0.0]
    target = [0.0, 1.0, 0.0]
    az, el, r = azimuth_elevation_range(observer, target)
    assert az == pytest.approx(0.0, abs=1e-9)
    assert r == pytest.approx(1.0, abs=1e-9)


def test_target_due_east_has_pi_half_azimuth():
    """Target directly along +X from observer → azimuth = +π/2."""
    observer = [0.0, 0.0, 0.0]
    target = [1.0, 0.0, 0.0]
    az, el, r = azimuth_elevation_range(observer, target)
    assert az == pytest.approx(math.pi / 2, abs=1e-9)
    assert r == pytest.approx(1.0, abs=1e-9)


def test_target_due_south_has_pi_azimuth():
    """Target directly along -Y from observer → azimuth = π or -π (both valid)."""
    observer = [0.0, 0.0, 0.0]
    target = [0.0, -1.0, 0.0]
    az, el, r = azimuth_elevation_range(observer, target)
    # atan2(0, -1) = ±π — accept both due to floating-point sign
    assert abs(az) == pytest.approx(math.pi, abs=1e-9)
    assert r == pytest.approx(1.0, abs=1e-9)


def test_target_due_west_has_minus_pi_half_azimuth():
    """Target directly along -X from observer → azimuth = -π/2."""
    observer = [0.0, 0.0, 0.0]
    target = [-1.0, 0.0, 0.0]
    az, el, r = azimuth_elevation_range(observer, target)
    assert az == pytest.approx(-math.pi / 2, abs=1e-9)
    assert r == pytest.approx(1.0, abs=1e-9)


def test_target_northeast_has_pi_quarter_azimuth():
    """Target along +X and +Y (45°) → azimuth = π/4."""
    observer = [0.0, 0.0, 0.0]
    target = [1.0, 1.0, 0.0]
    az, el, r = azimuth_elevation_range(observer, target)
    assert az == pytest.approx(math.pi / 4, abs=1e-9)
    assert r == pytest.approx(math.sqrt(2), abs=1e-9)


# ---------------------------------------------------------------------------
# Slice 2: Elevation angles (vertical component)
# ---------------------------------------------------------------------------

def test_target_directly_overhead_has_pi_half_elevation():
    """Target directly above observer (same XY, higher Z) → elevation = +π/2."""
    observer = [0.0, 0.0, 0.0]
    target = [0.0, 0.0, 1.0]
    az, el, r = azimuth_elevation_range(observer, target)
    assert el == pytest.approx(math.pi / 2, abs=1e-9)
    assert r == pytest.approx(1.0, abs=1e-9)


def test_target_directly_below_has_minus_pi_half_elevation():
    """Target directly below observer (same XY, lower Z) → elevation = -π/2."""
    observer = [0.0, 0.0, 1.0]
    target = [0.0, 0.0, 0.0]
    az, el, r = azimuth_elevation_range(observer, target)
    assert el == pytest.approx(-math.pi / 2, abs=1e-9)
    assert r == pytest.approx(1.0, abs=1e-9)


def test_target_at_same_height_has_zero_elevation():
    """Target at same Z as observer → elevation = 0."""
    observer = [1.0, 2.0, 5.0]
    target = [2.0, 3.0, 5.0]
    az, el, r = azimuth_elevation_range(observer, target)
    assert el == pytest.approx(0.0, abs=1e-9)


def test_target_45_degrees_above_horizontal():
    """Target at 45° elevation (dz = horizontal distance) → elevation = π/4."""
    observer = [0.0, 0.0, 0.0]
    target = [1.0, 0.0, 1.0]  # horiz dist = 1, dz = 1
    az, el, r = azimuth_elevation_range(observer, target)
    assert el == pytest.approx(math.pi / 4, abs=1e-9)
    assert r == pytest.approx(math.sqrt(2), abs=1e-9)


# ---------------------------------------------------------------------------
# Slice 3: Observer offset is subtracted before angles
# ---------------------------------------------------------------------------

def test_observer_offset_subtracts_from_target():
    """Observer at (1,2,3), target at (2,3,4) → relative [1,1,1].
    Azimuth should match [1,1,0], elevation [1,1,1].
    """
    observer = [1.0, 2.0, 3.0]
    target = [2.0, 3.0, 4.0]
    az, el, r = azimuth_elevation_range(observer, target)

    # Relative position: [1, 1, 1]
    # Azimuth: atan2(1, 1) = π/4 (northeast)
    assert az == pytest.approx(math.pi / 4, abs=1e-9)
    # Elevation: atan2(1, sqrt(1²+1²)) = atan2(1, √2)
    expected_el = math.atan2(1, math.sqrt(2))
    assert el == pytest.approx(expected_el, abs=1e-9)
    # Range: sqrt(1²+1²+1²) = √3
    assert r == pytest.approx(math.sqrt(3), abs=1e-9)


def test_observer_negative_offset():
    """Observer at (-5,-5,-5), target at (0,0,0) → relative [5,5,5]."""
    observer = [-5.0, -5.0, -5.0]
    target = [0.0, 0.0, 0.0]
    az, el, r = azimuth_elevation_range(observer, target)
    # Relative [5,5,5]
    assert az == pytest.approx(math.pi / 4, abs=1e-9)
    expected_el = math.atan2(5, math.sqrt(50))
    assert el == pytest.approx(expected_el, abs=1e-9)
    assert r == pytest.approx(math.sqrt(75), abs=1e-9)


# ---------------------------------------------------------------------------
# Slice 4: angle_diff wraps to (-π, π]
# ---------------------------------------------------------------------------

def test_angle_diff_zero_difference():
    """angle_diff(0, 0) = 0."""
    assert angle_diff(0.0, 0.0) == pytest.approx(0.0, abs=1e-9)


def test_angle_diff_small_positive():
    """angle_diff(0.5, 0.2) = 0.3 (straightforward)."""
    assert angle_diff(0.5, 0.2) == pytest.approx(0.3, abs=1e-9)


def test_angle_diff_small_negative():
    """angle_diff(0.2, 0.5) = -0.3 (straightforward)."""
    assert angle_diff(0.2, 0.5) == pytest.approx(-0.3, abs=1e-9)


def test_angle_diff_wraps_large_positive_to_negative():
    """A raw difference greater than π wraps the short way to a negative angle.

    For a=π/2, b=-π/2-0.1 the raw difference is π+0.1 (> π), so angle_diff
    must return the equivalent short-path value -(π-0.1).
    """
    a = math.pi / 2
    b = -math.pi / 2 - 0.1
    expected = -math.pi + 0.1  # wraps the short way
    assert angle_diff(a, b) == pytest.approx(expected, abs=1e-9)


def test_angle_diff_right_at_pi_boundary():
    """angle_diff(π, -π) = 0 (they represent the same angle mod 2π)."""
    assert angle_diff(math.pi, -math.pi) == pytest.approx(0.0, abs=1e-9)


def test_angle_diff_just_over_pi_wraps():
    """angle_diff(π + 0.1, 0) should wrap to -(π - 0.1)."""
    result = angle_diff(math.pi + 0.1, 0.0)
    # Difference is π + 0.1, which is > π, so wraps to -(2π - (π + 0.1)) = -(π - 0.1) ≈ -3.04
    expected = -(math.pi - 0.1)
    assert result == pytest.approx(expected, abs=1e-9)


def test_angle_diff_magnitude_never_exceeds_pi():
    """angle_diff() always returns a value with magnitude ≤ π."""
    test_cases = [
        (0.0, 0.0),
        (math.pi, 0.0),
        (0.0, math.pi),
        (math.pi, -math.pi),
        (2.5, -2.5),
        (6.0, 0.5),
        (0.5, 6.0),
    ]
    for a, b in test_cases:
        result = angle_diff(a, b)
        assert abs(result) <= math.pi + 1e-9, (
            f"angle_diff({a}, {b}) = {result}, magnitude {abs(result)} exceeds π"
        )


def test_angle_diff_large_angles():
    """angle_diff handles angles > 2π by wrapping correctly."""
    # 10 radians ≈ 573°, 0 radians — difference should wrap to shortest path
    # 10 - 0 = 10 rad ≈ 1.43 rad (mod 2π), but atan2 handles it
    result = angle_diff(10.0, 0.0)
    # 10 ≈ 10 - 2π ≈ 3.72, so diff ≈ 3.72, wraps to 3.72 - 2π ≈ -2.56
    expected = math.atan2(math.sin(10.0), math.cos(10.0))
    assert result == pytest.approx(expected, abs=1e-9)


# ---------------------------------------------------------------------------
# Slice 5: Integration — realistic target tracking scenario
# ---------------------------------------------------------------------------

def test_realistic_search_scenario():
    """Turret at (0,0,2), target at (10,5,3) → typical search pose.

    Relative position: [10, 5, 1]
    Azimuth: atan2(10, 5) ≈ 1.107 rad (63.4°)
    Elevation: atan2(1, sqrt(125)) ≈ 0.089 rad (5.1°)
    Range: sqrt(126) ≈ 11.22 m
    """
    observer = [0.0, 0.0, 2.0]
    target = [10.0, 5.0, 3.0]
    az, el, r = azimuth_elevation_range(observer, target)

    expected_az = math.atan2(10, 5)
    expected_el = math.atan2(1, math.sqrt(125))
    expected_r = math.sqrt(126)

    assert az == pytest.approx(expected_az, abs=1e-9)
    assert el == pytest.approx(expected_el, abs=1e-9)
    assert r == pytest.approx(expected_r, abs=1e-9)


# ---------------------------------------------------------------------------
# Slice 6: angular_separation — great-circle angle between two (az, el) pairs
# ---------------------------------------------------------------------------

from geometry import angular_separation


def test_angular_separation_same_direction_is_zero():
    """Two identical (az, el) pairs point in the same direction → separation = 0."""
    az = math.pi / 4
    el = math.pi / 6
    assert angular_separation(az, el, az, el) == pytest.approx(0.0, abs=1e-9)


def test_angular_separation_north_vs_east_is_pi_half():
    """North (az=0, el=0) and East (az=π/2, el=0) are 90° apart → separation = π/2."""
    az1, el1 = 0.0, 0.0          # due North (dx=0, dy=1, dz=0)
    az2, el2 = math.pi / 2, 0.0  # due East  (dx=1, dy=0, dz=0)
    sep = angular_separation(az1, el1, az2, el2)
    assert sep == pytest.approx(math.pi / 2, abs=1e-9)


def test_angular_separation_antipodal_is_pi():
    """Opposite directions (az=0, el=0) vs (az=π, el=0) → separation = π."""
    sep = angular_separation(0.0, 0.0, math.pi, 0.0)
    assert sep == pytest.approx(math.pi, abs=1e-9)


def test_angular_separation_perpendicular_via_elevation():
    """Horizon (az=0, el=0) vs straight up (az=0, el=π/2) → separation = π/2."""
    sep = angular_separation(0.0, 0.0, 0.0, math.pi / 2)
    assert sep == pytest.approx(math.pi / 2, abs=1e-9)


def test_angular_separation_is_symmetric():
    """angular_separation(a, b) == angular_separation(b, a) for arbitrary angles."""
    az1, el1 = 0.3, 0.4
    az2, el2 = 1.1, -0.2
    assert angular_separation(az1, el1, az2, el2) == pytest.approx(
        angular_separation(az2, el2, az1, el1), abs=1e-9
    )


def test_angular_separation_small_angle():
    """Two directions very close together → separation ≈ their Euclidean angular gap."""
    delta = 1e-4  # very small offset
    sep = angular_separation(0.0, 0.0, delta, 0.0)
    assert sep == pytest.approx(delta, abs=1e-6)


def test_angular_separation_result_always_nonnegative():
    """angular_separation never returns a negative value."""
    pairs = [
        (0.0, 0.0, 0.0, 0.0),
        (math.pi, 0.0, -math.pi, 0.0),
        (1.5, 0.3, -0.7, -0.9),
    ]
    for az1, el1, az2, el2 in pairs:
        sep = angular_separation(az1, el1, az2, el2)
        assert sep >= 0.0, f"negative separation for ({az1},{el1},{az2},{el2}): {sep}"
