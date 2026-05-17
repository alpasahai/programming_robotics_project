"""Tests for SearchRadar — ATLAS's wide-beam external acquisition sensor."""
import math
import random

from stubs import StubProjectile
from search_radar import Detection, SearchRadar


TURRET_POS = [1.0, 2.0, 0.5]
TIMESTEP_MS = 32


def test_get_detections_returns_empty_list_before_update():
    """get_detections() must return an empty list until update() is called."""
    proj = StubProjectile([[5.0, 6.0, 7.0]])
    radar = SearchRadar([proj], TURRET_POS, noise_std=0.0, timestep_ms=TIMESTEP_MS)
    assert radar.get_detections() == []


def test_get_target_position_returns_none_in_initial_state():
    """get_target_position() must return None before any update() or set_target()."""
    proj = StubProjectile([[5.0, 6.0, 7.0]])
    radar = SearchRadar([proj], TURRET_POS, noise_std=0.0, timestep_ms=TIMESTEP_MS)
    assert radar.get_target_position() is None


def test_get_target_position_returns_none_before_set_target():
    """get_target_position() must return None until set_target() is called."""
    proj = StubProjectile([[5.0, 6.0, 7.0]])
    radar = SearchRadar([proj], TURRET_POS, noise_std=0.0, timestep_ms=TIMESTEP_MS)
    radar.update()
    assert radar.get_target_position() is None


def test_set_target_clears_stored_target_position():
    """set_target() resets stored position so get_target_position() returns None.

    Verifies that switching targets mid-flight does not leave a stale reading
    from the previous target visible to callers.

    Scenario: one radar, two projectiles. Lock track_id=0, update() to populate
    a position reading, then switch to track_id=1 — get_target_position() must
    return None immediately (before the next update()).
    """
    proj0 = StubProjectile([[3.0, 3.0, 3.0]])
    proj1 = StubProjectile([[1.0, 1.0, 1.0]])
    radar = SearchRadar([proj0, proj1], [0.0, 0.0, 0.0], noise_std=0.0, timestep_ms=TIMESTEP_MS)
    radar.set_target(0)
    radar.update()
    assert radar.get_target_position() == [3.0, 3.0, 3.0]  # baseline: target 0 is populated
    radar.set_target(1)
    assert radar.get_target_position() is None


def test_set_target_filters_to_locked_node():
    """set_target(track_id) causes get_target_position() to return only the locked target.

    Two projectiles are present; after locking track_id=1, get_target_position()
    must reflect that specific projectile and not the other.

    See ADR-0006: track_id is an integer index into the projectile list; the
    SearchRadar resolves it to a node internally.
    """
    proj_a = StubProjectile([[3.0, 4.0, 5.0]])
    proj_b = StubProjectile([[9.0, 8.0, 7.0]])
    radar = SearchRadar([proj_a, proj_b], [0.0, 0.0, 0.0], noise_std=0.0, timestep_ms=TIMESTEP_MS)
    radar.set_target(1)
    radar.update()

    result = radar.get_target_position()
    assert result == [9.0, 8.0, 7.0]


def test_update_with_nonzero_noise_perturbs_readings():
    """With noise_std > 0, readings must deviate from noiseless values.

    Uses a seeded ``random.Random`` injected via the ``rng`` parameter so the
    test is fully deterministic without monkey-patching. Samples many updates
    and asserts mean absolute deviation is within expected Gaussian bounds
    (E[|X|] = sigma * sqrt(2/pi) ≈ 0.798 * sigma for N(0, sigma)).
    """
    noise_std = 2.0
    world_pos = [0.0, 0.0, 0.0]
    N = 1000
    deviations = []

    seeded_rng = random.Random(42)
    proj = StubProjectile([world_pos])
    radar = SearchRadar([proj], [0.0, 0.0, 0.0], noise_std=noise_std, timestep_ms=TIMESTEP_MS, rng=seeded_rng)
    for _ in range(N):
        radar.update()
        detections = radar.get_detections()
        deviations.extend(detections[0].position)

    mean_abs = sum(abs(d) for d in deviations) / len(deviations)
    expected_mean_abs = noise_std * (2 / math.pi) ** 0.5
    # Allow generous ±30% tolerance given sampling variance
    assert 0.7 * expected_mean_abs < mean_abs < 1.3 * expected_mean_abs


def test_update_noiseless_returns_exact_relative_positions():
    """update() with noise_std=0.0 stores exact turret-relative positions.

    Covers subtraction logic and that get_detections() returns one Detection
    per projectile with the correct track_id and no noise contribution.

    See ADR-0006: track_id is the projectile's index in the constructor list.
    """
    world_pos_a = [4.0, 5.0, 3.0]
    world_pos_b = [10.0, 0.0, 1.0]
    proj_a = StubProjectile([world_pos_a])
    proj_b = StubProjectile([world_pos_b])
    radar = SearchRadar([proj_a, proj_b], TURRET_POS, noise_std=0.0, timestep_ms=TIMESTEP_MS)
    radar.update()

    detections = radar.get_detections()
    assert len(detections) == 2

    expected_a = [world_pos_a[i] - TURRET_POS[i] for i in range(3)]
    expected_b = [world_pos_b[i] - TURRET_POS[i] for i in range(3)]

    assert detections[0].track_id == 0
    assert detections[0].position == expected_a
    assert detections[1].track_id == 1
    assert detections[1].position == expected_b


def test_detections_are_detection_instances():
    """get_detections() must return Detection NamedTuple instances (ADR-0006)."""
    proj = StubProjectile([[1.0, 2.0, 3.0]])
    radar = SearchRadar([proj], [0.0, 0.0, 0.0], noise_std=0.0, timestep_ms=TIMESTEP_MS)
    radar.update()
    detections = radar.get_detections()
    assert len(detections) == 1
    assert isinstance(detections[0], Detection)


def test_get_detections_returns_fresh_list():
    """get_detections() must return a new list each call (callers cannot mutate state)."""
    proj = StubProjectile([[1.0, 2.0, 3.0]])
    radar = SearchRadar([proj], [0.0, 0.0, 0.0], noise_std=0.0, timestep_ms=TIMESTEP_MS)
    radar.update()
    list_a = radar.get_detections()
    list_b = radar.get_detections()
    assert list_a is not list_b
