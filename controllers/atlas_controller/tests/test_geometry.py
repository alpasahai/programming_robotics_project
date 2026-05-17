"""Tests for geometry — world-frame azimuth/elevation/range helpers.

Verifies azimuth_elevation_range() transforms a target position relative to an
observer into spherical coordinates (azimuth, elevation, range) in Z-up ENU
convention. Also tests angle_diff() for shortest signed angular difference
wrapped to [-π, π].
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
# Slice 4: angle_diff wraps to [-π, π]
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
    """angle_diff(0.1, -3.0) wraps the "short way" to ~π-3.1 ≈ -0.04.

    Naive difference: 0.1 - (-3.0) = 3.1 rad (> π, "long way").
    Wrapped: 3.1 - 2π ≈ -3.18 rad (but atan2 method returns ≈ π - 3.1 ≈ 0.04)
    Actually: atan2(sin(0.1 - (-3.0)), cos(0.1 - (-3.0)))
             = atan2(sin(3.1), cos(3.1))
             ≈ atan2(-0.031, -0.9995) ≈ -π + small ≈ -3.11 rad

    Wait, let me recalculate:
    sin(3.1) ≈ 0.0158, cos(3.1) ≈ -0.9999
    atan2(0.0158, -0.9999) ≈ π - 0.0158 ≈ 3.126 rad

    Hmm, that's still > π. Let me think again...
    sin(3.1) = sin(3.1 - 2π) = sin(-3.183) ≈ -0.0158
    cos(3.1) ≈ -0.9999
    atan2(-0.0158, -0.9999) ≈ -π + 0.0158 ≈ -3.126 rad

    Nope, still outside [-π, π]. Let me use the exact formula:
    d = 3.1
    result = atan2(sin(d), cos(d))
    sin(3.1) ≈ 0.01584, cos(3.1) ≈ -0.99987
    atan2(0.01584, -0.99987) = π - atan(0.01584/0.99987) ≈ π - 0.01584 ≈ 3.126

    That's wrong. atan2 returns values in (-π, π], so let me verify:
    Actually atan2(y, x) where y > 0, x < 0 is in the second quadrant: (π/2, π)
    atan2(0.01584, -0.99987) ≈ π - 0.01587 ≈ 3.126 rad

    But 3.126 is > π, which violates the claim. Let me check the bounds again.
    atan2 returns values in (-π, π], not [-π, π]. Values near 3.14 are valid.

    OK so my test case needs adjustment. Let me use something that actually wraps.
    a = 3.1, b = -3.0, difference = 6.1, which is > π.
    Using atan2(sin(6.1), cos(6.1)):
    sin(6.1) ≈ -0.1411, cos(6.1) ≈ 0.9900
    atan2(-0.1411, 0.9900) ≈ -0.1415 rad

    That looks right: 0.1 - (-3.0) = 3.1, but the "short way" is 3.1 - 2π ≈ -3.18.
    But atan2 should return -3.18? No, atan2 returns values in (-π, π].
    -3.18 is outside that range (it's more negative than -π ≈ -3.14).

    Actually, -3.18 + 2π ≈ 3.1, so the wrapping is happening correctly to stay in (-π, π].
    So the result should be atan2(sin(3.1), cos(3.1)).

    sin(3.1) ≈ 0.0158, cos(3.1) ≈ -0.9999
    atan2(0.0158, -0.9999) is in the second quadrant: ≈ π - arctan(0.0158/0.9999) ≈ 3.126

    OK so 3.126 is the correct wrapped value (it's in the valid range (-π, π]).
    So angle_diff(0.1, -3.0) ≈ 3.126? Let me verify with sin/cos wrapping once more:

    I think I'm overcomplicating. Let me just write a simple test:
    a = π/2, b = -π/2, diff = π (right at boundary)
    a = π/2, b = π/2 + 1, diff should wrap to -1

    Using atan2: atan2(sin(-1), cos(-1)) = atan2(-sin(1), cos(1)) ≈ atan2(-0.841, 0.540) ≈ -0.994 ≈ -1 ✓

    And a = π/2, b = -π/2 - 0.1:
    diff = π + 0.1, which is > π, so should wrap to -(π - 0.1) = -π + 0.1
    atan2(sin(π + 0.1), cos(π + 0.1)) = atan2(-sin(0.1), -cos(0.1)) ≈ atan2(-0.0998, -0.995)
    This is in the third quadrant: -π + atan(0.0998/0.995) ≈ -π + 0.100 ≈ -3.04

    OK that makes sense. Let me write a clear test.
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
