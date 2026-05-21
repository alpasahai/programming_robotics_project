"""Unit tests for the turret Bullet recycle lifecycle.

The Bullet is one pre-placed Solid the atlas_controller supervisor recycles:
fire() teleports it to the muzzle and launches it toward the intercept;
recycle() zeroes motion and parks it. Tests use a node stub mirroring the
attacker's StubProjectileNode (getField('translation'), setVelocity,
resetPhysics).
"""
import math
from bullet import Bullet, BulletConfig


class StubField:
    def __init__(self):
        self.value = None

    def setSFVec3f(self, value):
        self.value = list(value)


class StubBulletNode:
    def __init__(self):
        self.translation_field = StubField()
        self.set_velocity_calls = []
        self.reset_physics_calls = 0

    def getField(self, name):
        assert name == "translation"
        return self.translation_field

    def setVelocity(self, velocity):
        self.set_velocity_calls.append(list(velocity))

    def resetPhysics(self):
        self.reset_physics_calls += 1


TURRET = [0.0, 0.0, 0.0]


def _make(config=None, turret_position=None):
    node = StubBulletNode()
    bullet = Bullet(
        node,
        turret_position=turret_position or TURRET,
        config=config or BulletConfig(),
    )
    return node, bullet


def test_starts_parked():
    _, bullet = _make()
    assert bullet.is_parked is True
    assert bullet.age_steps == 0


def test_fire_teleports_to_muzzle_along_aim_direction():
    # Intercept straight along +Y at 4 m; muzzle offset 0.6 m → spawn at y=0.6.
    config = BulletConfig(muzzle_speed=10.0, muzzle_offset_m=0.6)
    node, bullet = _make(config)

    bullet.fire([0.0, 4.0, 0.0])

    assert node.translation_field.value == [0.0, 0.6, 0.0]
    # Velocity = unit(intercept) * muzzle_speed, zero angular.
    assert node.set_velocity_calls[-1] == [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    assert bullet.is_parked is False


def test_fire_offsets_from_turret_world_position():
    # Turret not at origin: spawn = turret + unit(intercept) * offset.
    config = BulletConfig(muzzle_speed=10.0, muzzle_offset_m=1.0)
    node, bullet = _make(config, turret_position=[5.0, 0.0, 2.0])

    bullet.fire([0.0, 0.0, 3.0])  # straight up

    assert node.translation_field.value == [5.0, 0.0, 3.0]
    assert node.set_velocity_calls[-1] == [0.0, 0.0, 10.0, 0.0, 0.0, 0.0]


def test_fire_does_not_reset_physics_in_same_step_as_launch():
    # Regression: resetPhysics() with setVelocity() in one step zeroes the launch.
    node, bullet = _make()
    bullet.fire([1.0, 1.0, 1.0])
    assert node.reset_physics_calls == 0


def test_recycle_zeroes_motion_parks_and_resets_physics():
    config = BulletConfig(park_position=[0.0, 0.0, -100.0])
    node, bullet = _make(config)
    bullet.fire([0.0, 4.0, 0.0])

    bullet.recycle()

    assert node.set_velocity_calls[-1] == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert node.translation_field.value == [0.0, 0.0, -100.0]
    assert node.reset_physics_calls == 1
    assert bullet.is_parked is True
    assert bullet.age_steps == 0


def test_step_ages_only_in_flight():
    _, bullet = _make()
    bullet.step()
    assert bullet.age_steps == 0  # parked → no aging

    bullet.fire([0.0, 4.0, 0.0])
    bullet.step()
    bullet.step()
    assert bullet.age_steps == 2

    bullet.recycle()
    bullet.step()
    assert bullet.age_steps == 0  # parked again → reset and not aging


def test_fire_normalises_arbitrary_intercept():
    config = BulletConfig(muzzle_speed=12.0, muzzle_offset_m=0.0)
    node, bullet = _make(config)

    bullet.fire([3.0, 4.0, 0.0])  # |v| = 5

    vx, vy, vz = node.set_velocity_calls[-1][:3]
    speed = math.sqrt(vx * vx + vy * vy + vz * vz)
    assert abs(speed - 12.0) < 1e-9
    assert abs(vx - 12.0 * 3 / 5) < 1e-9
    assert abs(vy - 12.0 * 4 / 5) < 1e-9
