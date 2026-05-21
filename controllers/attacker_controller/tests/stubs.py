"""Test doubles for attacker_controller tests."""


class StubField:
    """Stand-in for a Webots SFVec3f field handle."""

    def __init__(self):
        self.value = None

    def setSFVec3f(self, value):
        self.value = list(value)


class StubProjectileNode:
    """Controllable stand-in for a Webots Solid node.

    position/velocity are driven by the test (set ``.position`` / ``.velocity``
    between ``step`` calls). Records calls so tests can assert on launch/reset.
    """

    def __init__(self, position=None, velocity=None):
        self.position = list(position) if position else [0.0, 0.0, 5.0]
        self.velocity = list(velocity) if velocity else [0.0, 0.0, 0.0]
        self.translation_field = StubField()
        self.set_velocity_calls = []
        self.reset_physics_calls = 0

    def getField(self, name):
        assert name == "translation"
        return self.translation_field

    def getPosition(self):
        return list(self.position)

    def getVelocity(self):
        return list(self.velocity)

    def setVelocity(self, velocity):
        self.set_velocity_calls.append(list(velocity))

    def resetPhysics(self):
        self.reset_physics_calls += 1
