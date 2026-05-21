"""Unit tests for the attacker's Projectile spawn/despawn state machine."""

from projectile import Projectile, ProjectileConfig
from stubs import StubProjectileNode, StubContact


def _make(config=None, on_ground_hit=None, on_bullet_hit=None):
    node = StubProjectileNode()
    projectile = Projectile(
        node, config, on_ground_hit=on_ground_hit, on_bullet_hit=on_bullet_hit
    )
    return node, projectile


def _floor(z=0.0):
    return [StubContact([0.0, 0.0, z])]


def _air(z=1.5):
    return [StubContact([0.0, 0.0, z])]


def test_spawn_teleports_to_spawn_point_and_launches():
    config = ProjectileConfig(
        spawn_position=[1.0, 2.0, 3.0], launch_velocity=[4, 5, 6, 0, 0, 0]
    )
    node, projectile = _make(config)

    projectile.spawn()

    assert node.translation_field.value == [1.0, 2.0, 3.0]
    assert node.set_velocity_calls[-1] == [4, 5, 6, 0, 0, 0]


def test_spawn_does_not_reset_physics_in_same_step_as_launch():
    # Regression: resetPhysics() in the same timestep as setVelocity() zeroes the
    # launch velocity in Webots, so spawn() must not call it (despawn does).
    node, projectile = _make()

    projectile.spawn()

    assert node.reset_physics_calls == 0


def test_floor_contact_at_launch_is_not_a_landing():
    # The ball touches the floor at spawn; until it has cleared the floor once,
    # a floor contact must not count as a ground hit.
    node, projectile = _make()
    projectile.spawn()

    node.contacts = _floor()
    projectile.step(100)

    assert projectile.ground_hits == 0


def test_no_hit_while_airborne_with_no_contacts():
    node, projectile = _make()
    projectile.spawn()

    node.contacts = []  # in flight, nothing touched
    projectile.step(100)

    assert projectile.ground_hits == 0
    assert projectile.bullet_hits == 0


def test_landing_after_liftoff_registers_ground_hit_and_despawns():
    hits = []
    config = ProjectileConfig(despawn_position=[0.0, 0.0, -100.0])
    node, projectile = _make(config, on_ground_hit=hits.append)
    projectile.spawn()

    node.contacts = []  # clears the floor → airborne
    projectile.step(100)
    node.contacts = _floor()  # comes back down and lands
    projectile.step(200)

    assert projectile.ground_hits == 1
    assert hits == [1]
    assert node.translation_field.value == [0.0, 0.0, -100.0]  # parked away
    assert node.set_velocity_calls[-1] == [0, 0, 0, 0, 0, 0]  # motion zeroed


def test_midair_contact_registers_bullet_hit_not_ground():
    bullets = []
    node, projectile = _make(on_bullet_hit=bullets.append)
    projectile.spawn()

    node.contacts = _air()  # struck mid-air by the bullet
    projectile.step(100)

    assert projectile.bullet_hits == 1
    assert projectile.ground_hits == 0
    assert bullets == [1]


def test_respawn_after_delay_relaunches():
    config = ProjectileConfig(
        spawn_position=[1.0, 2.0, 3.0],
        launch_velocity=[7, 8, 9, 0, 0, 0],
        respawn_delay_ms=2000,
    )
    node, projectile = _make(config)
    projectile.spawn()

    node.contacts = []
    projectile.step(100)  # airborne
    node.contacts = _floor()
    projectile.step(5000)  # lands → despawn at t=5000

    # Before delay elapses: still despawned, no relaunch.
    projectile.step(6500)
    assert node.set_velocity_calls[-1] == [0, 0, 0, 0, 0, 0]

    # After delay (5000 + 2000): respawn — back at spawn, relaunched.
    projectile.step(7001)
    assert node.translation_field.value == [1.0, 2.0, 3.0]
    assert node.set_velocity_calls[-1] == [7, 8, 9, 0, 0, 0]


def test_repeated_landings_increment_count():
    config = ProjectileConfig(respawn_delay_ms=1000)
    node, projectile = _make(config)
    projectile.spawn()

    # First cycle: lift off, land.
    node.contacts = []
    projectile.step(100)
    node.contacts = _floor()
    projectile.step(1000)
    assert projectile.ground_hits == 1

    # Respawn (>1000ms after despawn), then lift off and land again.
    projectile.step(2001)  # respawns
    node.contacts = []
    projectile.step(2100)  # airborne
    node.contacts = _floor()
    projectile.step(3000)  # lands
    assert projectile.ground_hits == 2
