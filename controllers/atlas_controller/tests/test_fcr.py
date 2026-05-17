"""Tests for FireControlRadar — ATLAS's narrow-beam on-board sensor."""
from stubs import StubProjectile
from fire_control_radar import FireControlRadar


TURRET_POS = [1.0, 2.0, 0.5]


def test_get_target_position_returns_none_before_first_update():
    """get_target_position() must return None until update() is called."""
    proj = StubProjectile([[5.0, 6.0, 7.0]])
    fcr = FireControlRadar(proj, TURRET_POS)
    assert fcr.get_target_position() is None


def test_update_subtracts_turret_position():
    """update() converts world-frame position to turret-relative [dx, dy, dz].

    Covers both the subtraction logic and the noiseless (noise_std=0.0) path.
    """
    world_pos = [4.0, 5.0, 3.0]
    proj = StubProjectile([world_pos])
    fcr = FireControlRadar(proj, TURRET_POS, noise_std=0.0)
    fcr.update()
    result = fcr.get_target_position()
    expected = [
        world_pos[0] - TURRET_POS[0],
        world_pos[1] - TURRET_POS[1],
        world_pos[2] - TURRET_POS[2],
    ]
    assert result == expected


def test_set_target_replaces_tracked_node():
    """set_target() causes subsequent update() to read from the new node."""
    original = StubProjectile([[0.0, 0.0, 0.0]])
    replacement = StubProjectile([[5.0, 5.0, 5.0]])
    fcr = FireControlRadar(original, [0.0, 0.0, 0.0], noise_std=0.0)
    fcr.update()
    assert fcr.get_target_position() == [0.0, 0.0, 0.0]

    fcr.set_target(replacement)
    fcr.update()
    assert fcr.get_target_position() == [5.0, 5.0, 5.0]


def test_set_target_clears_last_stored_position():
    """set_target() resets stored position so get_target_position() returns None."""
    proj = StubProjectile([[3.0, 3.0, 3.0]])
    fcr = FireControlRadar(proj, [0.0, 0.0, 0.0], noise_std=0.0)
    fcr.update()
    assert fcr.get_target_position() is not None

    new_node = StubProjectile([[1.0, 1.0, 1.0]])
    fcr.set_target(new_node)
    assert fcr.get_target_position() is None


def test_update_with_nonzero_noise_perturbs_reading():
    """With noise_std > 0, readings must differ from the noiseless value on at least one axis.

    Uses a seeded ``random.Random`` injected via the ``rng`` parameter so the
    test is fully deterministic without monkey-patching. Runs many samples and
    asserts the mean absolute deviation is within expected Gaussian bounds
    (1-sigma = noise_std), confirming noise is actually applied.
    """
    import random

    noise_std = 1.0
    world_pos = [0.0, 0.0, 0.0]
    N = 1000
    deviations = []

    # One seeded RNG shared across all update() calls so each draw is
    # independent — re-seeding inside the loop would repeat the same draw.
    seeded_rng = random.Random(42)
    proj = StubProjectile([world_pos])
    fcr = FireControlRadar(proj, [0.0, 0.0, 0.0], noise_std=noise_std, rng=seeded_rng)
    for _ in range(N):
        fcr.update()
        pos = fcr.get_target_position()
        deviations.extend(pos)

    mean_abs = sum(abs(d) for d in deviations) / len(deviations)
    # For N(0, sigma), E[|X|] = sigma * sqrt(2/pi) ≈ 0.798 * sigma
    expected_mean_abs = noise_std * (2 / 3.14159265358979) ** 0.5
    # Allow generous ±30% tolerance given sampling variance
    assert 0.7 * expected_mean_abs < mean_abs < 1.3 * expected_mean_abs
