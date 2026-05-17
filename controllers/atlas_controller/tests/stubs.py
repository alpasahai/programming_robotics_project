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


class StubFCR:
    def __init__(self, position, noise_std=0.0):
        self._position = position

    def update(self):
        pass

    def get_target_position(self):
        return list(self._position)

    def set_target(self, node):
        pass


class StubSearchRadar:
    def __init__(self, detections):
        self._detections = detections

    def update(self):
        pass

    def get_detections(self):
        return list(self._detections)

    def get_target_position(self):
        if not self._detections:
            return None
        return list(self._detections[0])

    def set_target(self, node):
        pass


class StubTrackFilter:
    def __init__(self, position, velocity):
        self._position = position
        self._velocity = velocity

    def predict(self):
        pass

    def update_fcr(self, m):
        pass

    def update_search(self, m):
        pass

    def reset(self):
        pass

    def get_position(self):
        return list(self._position)

    def get_velocity(self):
        return list(self._velocity)

    def is_initialised(self):
        return True
