"""Test doubles for attacker_controller tests."""


class StubField:
    """Stand-in for a Webots SFVec3f field handle."""

    def __init__(self):
        self.value = None

    def setSFVec3f(self, value):
        self.value = list(value)


class StubContact:
    """Stand-in for a Webots ContactPoint (world-frame .point)."""

    def __init__(self, point):
        self.point = list(point)


class StubProjectileNode:
    """Controllable stand-in for a Webots Solid node.

    Tests drive ``.contacts`` (a list of StubContact) between ``step`` calls to
    simulate floor / mid-air contacts. Records calls so tests can assert on
    launch/reset.
    """

    def __init__(self):
        self.contacts = []
        self.translation_field = StubField()
        self.set_velocity_calls = []
        self.last_velocity = None
        self.reset_physics_calls = 0

    def getField(self, name):
        assert name == "translation"
        return self.translation_field

    def getContactPoints(self, includeDescendants=False):
        return self.contacts

    def setVelocity(self, velocity):
        self.set_velocity_calls.append(list(velocity))
        self.last_velocity = list(velocity)

    def resetPhysics(self):
        self.reset_physics_calls += 1
