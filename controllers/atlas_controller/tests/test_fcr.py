"""Tests for FireControlRadar — FOV-gated narrow-beam on-board sensor.

The FCR detects the projectile only when it lies inside a narrow cone
(half-angle ``fov_half_angle``) around the turret's current aim boresight
AND within ``max_range``. Detection is reported as a turret-relative position.
"""
import math
import random

import pytest
from stubs import StubProjectile
from fire_control_radar import FireControlRadar


# ---------------------------------------------------------------------------
# Shared geometry helpers for building test scenarios
# ---------------------------------------------------------------------------

def _position_at_az_el_range(origin, az, el, r):
    """Return world position of a point at (az, el, r) from origin.

    Inverse of azimuth_elevation_range: converts spherical (az, el, r)
    back to Cartesian world coordinates relative to ``origin``.

    Convention matches geometry.azimuth_elevation_range:
        dx = sin(az) * cos(el) * r
        dy = cos(az) * cos(el) * r
        dz = sin(el) * r
    """
    dx = math.sin(az) * math.cos(el) * r
    dy = math.cos(az) * math.cos(el) * r
    dz = math.sin(el) * r
    return [origin[0] + dx, origin[1] + dy, origin[2] + dz]


# FCR and turret are co-located at this world position.
FCR_POS = [1.0, 2.0, 0.5]
TURRET_POS = [1.0, 2.0, 0.5]
FOV = math.radians(10.0)   # ±10° half-angle
MAX_RANGE = 50.0            # metres

# Boresight pointing due North at zero elevation
BORESIGHT_AZ = 0.0
BORESIGHT_EL = 0.0


# ---------------------------------------------------------------------------
# Pre-update state
# ---------------------------------------------------------------------------

def test_get_target_position_returns_none_before_first_update():
    """get_target_position() must return None before any update() call."""
    proj = StubProjectile([_position_at_az_el_range(FCR_POS, BORESIGHT_AZ, BORESIGHT_EL, 10.0)])
    fcr = FireControlRadar(proj, FCR_POS, TURRET_POS, FOV, MAX_RANGE)
    assert fcr.get_target_position() is None


def test_is_locked_false_before_first_update():
    """is_locked() must return False before any update() call."""
    proj = StubProjectile([_position_at_az_el_range(FCR_POS, BORESIGHT_AZ, BORESIGHT_EL, 10.0)])
    fcr = FireControlRadar(proj, FCR_POS, TURRET_POS, FOV, MAX_RANGE)
    assert fcr.is_locked() is False


# ---------------------------------------------------------------------------
# Detection gating — inside cone and in range
# ---------------------------------------------------------------------------

def test_target_inside_cone_and_in_range_is_detected():
    """Target inside FOV cone and within max_range → detected, is_locked() True."""
    # Place target exactly on boresight at 20 m (well within 50 m max)
    world_pos = _position_at_az_el_range(FCR_POS, BORESIGHT_AZ, BORESIGHT_EL, 20.0)
    proj = StubProjectile([world_pos])
    fcr = FireControlRadar(proj, FCR_POS, TURRET_POS, FOV, MAX_RANGE, noise_std=0.0)
    fcr.update(BORESIGHT_AZ, BORESIGHT_EL)
    assert fcr.is_locked() is True
    assert fcr.get_target_position() is not None


def test_target_just_inside_cone_edge_is_detected():
    """Target just inside fov_half_angle from boresight → detected.

    Places the target at 99.9% of the FOV half-angle away from boresight to
    verify a clearly-inside target is detected without relying on exact
    floating-point equality at the boundary (which would be machine-epsilon
    sensitive due to the az-offset → angular_separation round-trip).
    """
    # 99.9 % of the FOV half-angle — well inside the cone
    target_az = BORESIGHT_AZ + FOV * 0.999
    world_pos = _position_at_az_el_range(FCR_POS, target_az, BORESIGHT_EL, 10.0)
    proj = StubProjectile([world_pos])
    fcr = FireControlRadar(proj, FCR_POS, TURRET_POS, FOV, MAX_RANGE, noise_std=0.0)
    fcr.update(BORESIGHT_AZ, BORESIGHT_EL)
    assert fcr.is_locked() is True


# ---------------------------------------------------------------------------
# Detection gating — outside cone
# ---------------------------------------------------------------------------

def test_target_outside_cone_is_not_detected():
    """Target outside FOV cone (separation > fov_half_angle) → not detected."""
    # Offset azimuth by twice the FOV half-angle — clearly outside
    target_az = BORESIGHT_AZ + 2 * FOV
    world_pos = _position_at_az_el_range(FCR_POS, target_az, BORESIGHT_EL, 10.0)
    proj = StubProjectile([world_pos])
    fcr = FireControlRadar(proj, FCR_POS, TURRET_POS, FOV, MAX_RANGE, noise_std=0.0)
    fcr.update(BORESIGHT_AZ, BORESIGHT_EL)
    assert fcr.is_locked() is False
    assert fcr.get_target_position() is None


def test_boresight_pointed_away_loses_lock():
    """After detection, rotating boresight away from target loses the lock."""
    world_pos = _position_at_az_el_range(FCR_POS, BORESIGHT_AZ, BORESIGHT_EL, 10.0)
    proj = StubProjectile([world_pos, world_pos])
    fcr = FireControlRadar(proj, FCR_POS, TURRET_POS, FOV, MAX_RANGE, noise_std=0.0)

    # First update: boresight on target → locked
    fcr.update(BORESIGHT_AZ, BORESIGHT_EL)
    assert fcr.is_locked() is True

    # Second update: boresight rotated 90° away → lost
    fcr.update(BORESIGHT_AZ + math.pi / 2, BORESIGHT_EL)
    assert fcr.is_locked() is False
    assert fcr.get_target_position() is None


# ---------------------------------------------------------------------------
# Detection gating — beyond max_range
# ---------------------------------------------------------------------------

def test_target_beyond_max_range_is_not_detected():
    """Target inside cone but beyond max_range → not detected."""
    world_pos = _position_at_az_el_range(FCR_POS, BORESIGHT_AZ, BORESIGHT_EL, MAX_RANGE + 1.0)
    proj = StubProjectile([world_pos])
    fcr = FireControlRadar(proj, FCR_POS, TURRET_POS, FOV, MAX_RANGE, noise_std=0.0)
    fcr.update(BORESIGHT_AZ, BORESIGHT_EL)
    assert fcr.is_locked() is False
    assert fcr.get_target_position() is None


def test_target_at_exact_max_range_is_detected():
    """Target at exactly max_range and inside cone → detected (≤ boundary)."""
    world_pos = _position_at_az_el_range(FCR_POS, BORESIGHT_AZ, BORESIGHT_EL, MAX_RANGE)
    proj = StubProjectile([world_pos])
    fcr = FireControlRadar(proj, FCR_POS, TURRET_POS, FOV, MAX_RANGE, noise_std=0.0)
    fcr.update(BORESIGHT_AZ, BORESIGHT_EL)
    assert fcr.is_locked() is True


# ---------------------------------------------------------------------------
# Reported position is turret-relative (world − turret_position)
# ---------------------------------------------------------------------------

def test_detected_position_is_turret_relative():
    """On detection, stored position is world_pos − turret_position (noiseless)."""
    world_pos = _position_at_az_el_range(FCR_POS, BORESIGHT_AZ, BORESIGHT_EL, 10.0)
    proj = StubProjectile([world_pos])
    fcr = FireControlRadar(proj, FCR_POS, TURRET_POS, FOV, MAX_RANGE, noise_std=0.0)
    fcr.update(BORESIGHT_AZ, BORESIGHT_EL)
    result = fcr.get_target_position()
    expected = [world_pos[i] - TURRET_POS[i] for i in range(3)]
    assert result == pytest.approx(expected, abs=1e-9)


def test_detected_position_with_nonzero_turret_offset():
    """Turret and FCR co-located but offset from origin — relative position is correct."""
    turret = [5.0, -3.0, 1.0]
    world_pos = _position_at_az_el_range(turret, BORESIGHT_AZ, BORESIGHT_EL, 8.0)
    proj = StubProjectile([world_pos])
    fcr = FireControlRadar(proj, turret, turret, FOV, MAX_RANGE, noise_std=0.0)
    fcr.update(BORESIGHT_AZ, BORESIGHT_EL)
    result = fcr.get_target_position()
    expected = [world_pos[i] - turret[i] for i in range(3)]
    assert result == pytest.approx(expected, abs=1e-9)


# ---------------------------------------------------------------------------
# Noise
# ---------------------------------------------------------------------------

def test_noise_perturbs_detected_position():
    """With noise_std > 0, detected position deviates from the exact relative value.

    Uses a seeded RNG for determinism. Runs many samples and checks the mean
    absolute deviation matches the expected Gaussian statistics.
    """
    noise_std = 1.0
    world_pos = _position_at_az_el_range(FCR_POS, BORESIGHT_AZ, BORESIGHT_EL, 10.0)
    N = 1000
    deviations = []

    seeded_rng = random.Random(42)
    proj = StubProjectile([world_pos] * N)
    fcr = FireControlRadar(
        proj, FCR_POS, TURRET_POS, FOV, MAX_RANGE,
        noise_std=noise_std, rng=seeded_rng
    )
    exact_relative = [world_pos[i] - TURRET_POS[i] for i in range(3)]

    for _ in range(N):
        fcr.update(BORESIGHT_AZ, BORESIGHT_EL)
        pos = fcr.get_target_position()
        assert pos is not None, "Should be locked on every update"
        deviations.extend([pos[i] - exact_relative[i] for i in range(3)])

    mean_abs = sum(abs(d) for d in deviations) / len(deviations)
    # For N(0, sigma), E[|X|] = sigma * sqrt(2/pi) ≈ 0.798 * sigma
    expected_mean_abs = noise_std * (2 / math.pi) ** 0.5
    # Allow ±30% tolerance for sampling variance
    assert 0.7 * expected_mean_abs < mean_abs < 1.3 * expected_mean_abs
