class StubProjectile:
    def __init__(self, positions):
        self._positions = positions
        self._index = 0

    def getPosition(self):
        pos = self._positions[min(self._index, len(self._positions) - 1)]
        self._index += 1
        return pos


class StubMotor:
    def __init__(self):
        self.position = 0.0

    def setPosition(self, angle):
        self.position = angle


class StubRadar:
    def __init__(self, position=None, velocity=None):
        self._position = position or [2.0, 1.0, 1.0]
        self._velocity = velocity or [1.0, 0.5, 0.5]

    def update(self):
        pass

    def get_target_position(self):
        return list(self._position)

    def get_target_velocity(self):
        return list(self._velocity)

    def get_target_state(self):
        return self.get_target_position(), self.get_target_velocity()
