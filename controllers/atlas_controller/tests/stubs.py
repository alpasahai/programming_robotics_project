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
    """Test double for FireControlRadar.

    Reworked for the cue-handoff FSM: ``set_target`` is gone (the real FCR is
    cued at construction). ``update(az, el)`` accepts the turret boresight.
    ``is_locked()`` returns a configurable lock flag, defaulting to True.
    """

    def __init__(self, position, noise_std=0.0, locked=True):
        self._position = position
        self._locked = locked
        self.update_calls = []

    def update(self, boresight_az=0.0, boresight_el=0.0):
        """Record the commanded boresight. No gating is simulated."""
        self.update_calls.append((boresight_az, boresight_el))

    def get_target_position(self):
        return list(self._position)

    def is_locked(self):
        """Return the configured lock flag (default True)."""
        return self._locked


class StubCueLink:
    """Test double for SearchRadarLink.

    ``get_cue()`` returns a configurable world-frame cue ``[x, y, z]`` or
    ``None``. ``update()`` is a no-op. The cue can be reassigned between steps
    (set ``.cue``) to simulate the link going briefly silent.
    """

    def __init__(self, cue=None):
        """
        Args:
            cue: world-frame [x, y, z] returned by get_cue(), or None.
        """
        self.cue = cue

    def update(self):
        pass

    def clear(self):
        """Discard the stored cue, mirroring SearchRadarLink.clear()."""
        self.cue = None

    def get_cue(self):
        """Return the configured world-frame cue, or None."""
        return self.cue


class StubGroundHitLink:
    """Test double for AttackerGroundHitLink.

    ``hit_this_step()`` returns the configured flag (default False).
    ``update()`` is a no-op. Set ``.hit`` between steps to simulate a pulse.
    """

    def __init__(self, hit=False, count=0):
        self.hit = hit
        self.count = count

    def update(self):
        pass

    def hit_this_step(self):
        return self.hit


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
