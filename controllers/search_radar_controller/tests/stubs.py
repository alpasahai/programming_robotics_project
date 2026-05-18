"""Test stubs for search_radar_controller tests.

Contains only the stubs required by test_search_radar.py. The full atlas
stubs live in controllers/atlas_controller/tests/stubs.py.
"""


class StubProjectile:
    def __init__(self, positions):
        self._positions = positions
        self._index = 0

    def getPosition(self):
        pos = self._positions[min(self._index, len(self._positions) - 1)]
        self._index += 1
        return pos
