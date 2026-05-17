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
    """Test double for SearchRadar.

    Accepts a list of Detection objects at construction and returns them from
    get_detections(). Implements the same interface as the real SearchRadar
    without any Webots dependency.

    See ADR-0006: set_target() accepts an integer track_id (no-op here).
    get_target_position() returns the first detection's position, or None.
    """

    def __init__(self, detections):
        """
        Args:
            detections: list of Detection objects (from search_radar.Detection)
                        to return from get_detections().
        """
        from search_radar import Detection  # local import avoids circular dep at module level
        self._detections = list(detections)

    def update(self):
        pass

    def get_detections(self):
        """Return a copy of the configured Detection list."""
        return list(self._detections)

    def get_target_position(self):
        """Return the position of the first detection, or None if the list is empty."""
        if not self._detections:
            return None
        return list(self._detections[0].position)

    def set_target(self, track_id):
        """No-op. Accepts the integer track_id per ADR-0006."""
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
