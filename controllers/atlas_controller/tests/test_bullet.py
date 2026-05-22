"""Unit tests for the turret Bullet recycle lifecycle.

The Bullet is one pre-placed Solid the atlas_controller supervisor recycles.
Launching is two-phase (see bullet.py): fire() teleports it to the muzzle and
arms it (state LAUNCHING); launch() applies the stashed velocity one step later
(state IN_FLIGHT); recycle() zeroes motion and parks it. The split exists because
Webots applies a translation-field teleport on the next robot.step() and that
teleport resets the body's velocity — so the launch velocity must be set a step
*after* the teleport, not in the same step. Tests use a node stub mirroring the
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
    assert bullet.is_launching is False
    assert bullet.age_steps == 0


def test_fire_arms_but_does_not_set_velocity_yet():
    """fire() teleports + resets physics and enters LAUNCHING, but the launch
    velocity is NOT applied in the same step (it would be wiped by the deferred
    teleport in Webots) — that is launch()'s job."""
    config = BulletConfig(muzzle_speed=10.0, muzzle_offset_m=0.6)
    node, bullet = _make(config)

    bullet.fire([0.0, 4.0, 0.0])

    # Teleported to the muzzle (offset 0.6 m along +Y) and physics committed.
    assert node.translation_field.value == [0.0, 0.6, 0.0]
    assert node.reset_physics_calls == 1
    # No velocity yet — only fire() ran, not launch().
    assert node.set_velocity_calls == []
    assert bullet.is_launching is True
    assert bullet.is_parked is False


def test_launch_applies_velocity_and_enters_flight():
    """launch() applies the stashed launch velocity and enters IN_FLIGHT."""
    config = BulletConfig(muzzle_speed=10.0, muzzle_offset_m=0.6)
    node, bullet = _make(config)

    bullet.fire([0.0, 4.0, 0.0])
    bullet.launch()

    # Velocity = unit(intercept) * muzzle_speed, zero angular.
    assert node.set_velocity_calls[-1] == [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    assert bullet.is_launching is False
    assert bullet.is_parked is False


def test_launch_is_a_noop_unless_launching():
    """launch() does nothing while PARKED or already IN_FLIGHT, so a stray call
    cannot fire a parked bullet or re-set velocity mid-flight."""
    node, bullet = _make()

    bullet.launch()  # PARKED → no effect
    assert node.set_velocity_calls == []
    assert bullet.is_parked is True

    bullet.fire([0.0, 4.0, 0.0])
    bullet.launch()  # LAUNCHING → applies velocity once
    bullet.launch()  # IN_FLIGHT → no second velocity write
    assert len(node.set_velocity_calls) == 1


def test_fire_offsets_from_turret_world_position():
    # Turret not at origin: spawn = turret + unit(intercept) * offset.
    config = BulletConfig(muzzle_speed=10.0, muzzle_offset_m=1.0)
    node, bullet = _make(config, turret_position=[5.0, 0.0, 2.0])

    bullet.fire([0.0, 0.0, 3.0])  # straight up

    assert node.translation_field.value == [5.0, 0.0, 3.0]

    bullet.launch()
    assert node.set_velocity_calls[-1] == [0.0, 0.0, 10.0, 0.0, 0.0, 0.0]


def test_fire_does_not_set_velocity_in_same_step_as_the_teleport():
    """Regression for the drop-at-muzzle bug: a setVelocity() issued in the same
    step as the translation-field teleport is wiped by Webots when the teleport
    lands. fire() must therefore NOT call setVelocity (launch() does, next step)."""
    node, bullet = _make()
    bullet.fire([1.0, 1.0, 1.0])
    assert node.set_velocity_calls == []


def test_recycle_zeroes_motion_parks_and_resets_physics():
    config = BulletConfig(park_position=[0.0, 0.0, -100.0])
    node, bullet = _make(config)
    bullet.fire([0.0, 4.0, 0.0])  # resetPhysics() #1 (commits the teleport)
    bullet.launch()

    bullet.recycle()  # resetPhysics() #2 (parks the body)

    assert node.set_velocity_calls[-1] == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert node.translation_field.value == [0.0, 0.0, -100.0]
    assert node.reset_physics_calls == 2
    assert bullet.is_parked is True
    assert bullet.age_steps == 0


def test_step_ages_only_in_flight():
    _, bullet = _make()
    bullet.step()
    assert bullet.age_steps == 0  # parked → no aging

    bullet.fire([0.0, 4.0, 0.0])
    bullet.step()
    assert bullet.age_steps == 0  # LAUNCHING (velocity not yet applied) → no aging

    bullet.launch()
    bullet.step()
    bullet.step()
    assert bullet.age_steps == 2  # IN_FLIGHT → ages

    bullet.recycle()
    bullet.step()
    assert bullet.age_steps == 0  # parked again → reset and not aging


def test_fire_normalises_arbitrary_intercept():
    config = BulletConfig(muzzle_speed=12.0, muzzle_offset_m=0.0)
    node, bullet = _make(config)

    bullet.fire([3.0, 4.0, 0.0])  # |v| = 5
    bullet.launch()

    vx, vy, vz = node.set_velocity_calls[-1][:3]
    speed = math.sqrt(vx * vx + vy * vy + vz * vz)
    assert abs(speed - 12.0) < 1e-9
    assert abs(vx - 12.0 * 3 / 5) < 1e-9
    assert abs(vy - 12.0 * 4 / 5) < 1e-9
