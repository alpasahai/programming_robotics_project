"""Unit tests for the attacker's Projectile spawn/despawn state machine."""

from projectile import Projectile, ProjectileConfig
from stubs import StubProjectileNode


def _make(config=None, on_ground_hit=None):
    node = StubProjectileNode()
    return node, Projectile(node, config, on_ground_hit=on_ground_hit)


def test_spawn_teleports_to_spawn_point_and_launches():
    config = ProjectileConfig(
        spawn_position=[1.0, 2.0, 3.0], launch_velocity=[4, 5, 6, 0, 0, 0]
    )
    node, projectile = _make(config)

    projectile.spawn()

    assert node.translation_field.value == [1.0, 2.0, 3.0]
    assert node.set_velocity_calls[-1] == [4, 5, 6, 0, 0, 0]
    assert node.reset_physics_calls == 1


def test_no_despawn_while_in_flight():
    node, projectile = _make()
    projectile.spawn()
    reset_calls_after_spawn = node.reset_physics_calls

    # Airborne and rising — must not register a hit or despawn.
    node.position = [0, 0, 5.0]
    node.velocity = [0, 0, 2.0]
    projectile.step(10_000)

    assert projectile.ground_hits == 0
    assert node.reset_physics_calls == reset_calls_after_spawn


def test_ground_hit_registers_and_despawns():
    hits = []
    config = ProjectileConfig(
        despawn_position=[0.0, 0.0, -100.0], ground_hit_threshold_m=0.05
    )
    node, projectile = _make(config, on_ground_hit=hits.append)
    projectile.spawn()

    # Below threshold and descending → ground hit.
    node.position = [0, 0, 0.04]
    node.velocity = [0, 0, -1.0]
    projectile.step(5000)

    assert projectile.ground_hits == 1
    assert hits == [1]  # callback fired with the running count
    assert node.translation_field.value == [0.0, 0.0, -100.0]  # parked away
    assert node.set_velocity_calls[-1] == [0, 0, 0, 0, 0, 0]  # motion zeroed


def test_respawn_after_delay_elapses():
    config = ProjectileConfig(
        spawn_position=[1.0, 2.0, 3.0],
        launch_velocity=[7, 8, 9, 0, 0, 0],
        respawn_delay_ms=2000,
    )
    node, projectile = _make(config)
    projectile.spawn()

    # Ground hit at t=5000ms → despawned.
    node.position = [0, 0, 0.04]
    node.velocity = [0, 0, -1.0]
    projectile.step(5000)

    # Before delay elapses: still despawned, no new launch.
    projectile.step(6500)
    assert node.set_velocity_calls[-1] == [0, 0, 0, 0, 0, 0]

    # After delay (5000 + 2000 = 7000): respawn — back at spawn, relaunched.
    projectile.step(7001)
    assert node.translation_field.value == [1.0, 2.0, 3.0]
    assert node.set_velocity_calls[-1] == [7, 8, 9, 0, 0, 0]


def test_descending_above_threshold_does_not_register():
    config = ProjectileConfig(ground_hit_threshold_m=0.05)
    node, projectile = _make(config)
    projectile.spawn()

    # Falling but still well above the ground threshold.
    node.position = [0, 0, 1.0]
    node.velocity = [0, 0, -3.0]
    projectile.step(5000)

    assert projectile.ground_hits == 0


def test_repeated_hits_increment_count():
    config = ProjectileConfig(respawn_delay_ms=1000)
    node, projectile = _make(config)
    projectile.spawn()

    # First hit at t=1000.
    node.position = [0, 0, 0.04]
    node.velocity = [0, 0, -1.0]
    projectile.step(1000)
    assert projectile.ground_hits == 1

    # Respawn at t>2000, then a second hit.
    projectile.step(2001)  # respawns (back in flight)
    node.position = [0, 0, 0.04]
    node.velocity = [0, 0, -1.0]
    projectile.step(3000)
    assert projectile.ground_hits == 2
