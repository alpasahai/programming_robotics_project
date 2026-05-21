"""Unit tests for the attacker's Projectile lifecycle state machine."""

from projectile import Projectile, ProjectileConfig
from stubs import StubProjectileNode


def _make(config=None):
    node = StubProjectileNode()
    return node, Projectile(node, config)


def test_launch_sets_configured_velocity():
    config = ProjectileConfig(launch_velocity=[1, 2, 3, 0, 0, 0])
    node, projectile = _make(config)

    projectile.launch()

    assert node.set_velocity_calls == [[1, 2, 3, 0, 0, 0]]


def test_no_relaunch_while_in_flight():
    config = ProjectileConfig(launch_delay_ms=100)
    node, projectile = _make(config)
    projectile.launch()

    # Airborne and rising — nothing should happen no matter how much time passes.
    node.position = [0, 0, 5.0]
    node.velocity = [0, 0, 2.0]
    projectile.step(10_000)

    assert node.set_velocity_calls == [[0, 4, 6, 0, 0, 0]]  # only the initial launch
    assert node.reset_physics_calls == 0


def test_ground_hit_resets_to_spawn():
    config = ProjectileConfig(
        spawn_position=[1.0, 2.0, 3.0],
        ground_hit_threshold_m=0.05,
        launch_delay_ms=2000,
    )
    node, projectile = _make(config)
    projectile.launch()

    # Below the threshold and descending → counts as a ground hit.
    node.position = [0, 0, 0.04]
    node.velocity = [0, 0, -1.0]
    projectile.step(5000)

    assert node.reset_physics_calls == 1
    assert node.translation_field.value == [1.0, 2.0, 3.0]
    # Reset zeros velocity; no relaunch yet (delay not elapsed).
    assert node.set_velocity_calls[-1] == [0, 0, 0, 0, 0, 0]


def test_relaunch_after_delay_elapses():
    config = ProjectileConfig(launch_delay_ms=2000)
    node, projectile = _make(config)
    projectile.launch()

    # Ground hit at t=5000ms.
    node.position = [0, 0, 0.04]
    node.velocity = [0, 0, -1.0]
    projectile.step(5000)

    # Before the delay elapses: still waiting.
    projectile.step(6500)
    assert node.set_velocity_calls[-1] == [0, 0, 0, 0, 0, 0]

    # After the delay (5000 + 2000 = 7000): relaunch fires.
    projectile.step(7001)
    assert node.set_velocity_calls[-1] == [0, 4, 6, 0, 0, 0]


def test_descending_above_threshold_does_not_reset():
    config = ProjectileConfig(ground_hit_threshold_m=0.05)
    node, projectile = _make(config)
    projectile.launch()

    # Falling but still well above the ground threshold.
    node.position = [0, 0, 1.0]
    node.velocity = [0, 0, -3.0]
    projectile.step(5000)

    assert node.reset_physics_calls == 0
