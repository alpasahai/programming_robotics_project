"""SectorMap — online discovery of launch directions by angular clustering.

ATLAS is not told how many launch sectors exist or where they are. SectorMap
clusters the stream of observed launch bearings online (1-D angular leader
clustering): each bearing joins the nearest existing cluster centre within an
angular tolerance, or opens a new cluster with a fresh stable id. The number of
sectors emerges from the data — no k. Centres are nudged toward their members
(exponential moving average) so they track slow sensor bias. Handles the ±π
wrap-around. See the attack-plan spec.
"""
import math

_TWO_PI = 2.0 * math.pi


def _ang_diff(a, b):
    """Signed smallest angle a−b, in [−π, π)."""
    return (a - b + math.pi) % _TWO_PI - math.pi


class SectorMap:
    """Online angular clustering of launch bearings into stable sector ids.

    Args:
        tol_rad: a bearing within this angular distance of an existing centre
            joins that cluster; otherwise a new cluster opens. A *perception*
            threshold (set from radar bearing noise), not a learning knob.
        nudge:   EMA factor for moving a centre toward new members (0 = frozen
            centres, 1 = centre snaps to the latest bearing).
    """

    def __init__(self, tol_rad, nudge=0.2):
        self._tol = tol_rad
        self._nudge = nudge
        self._centers: list[float] = []

    def observe(self, bearing) -> int:
        """Assign ``bearing`` to a sector id, discovering a new one if needed."""
        best_id, best_dist = None, None
        for i, c in enumerate(self._centers):
            d = abs(_ang_diff(bearing, c))
            if best_dist is None or d < best_dist:
                best_id, best_dist = i, d
        if best_id is not None and best_dist <= self._tol:
            # Nudge the centre toward the new bearing (wrap-safe).
            updated = self._centers[best_id] + self._nudge * _ang_diff(
                bearing, self._centers[best_id]
            )
            self._centers[best_id] = (updated + math.pi) % _TWO_PI - math.pi
            return best_id
        self._centers.append(bearing)
        return len(self._centers) - 1

    def bearing(self, sector_id) -> float:
        """Return the current centre bearing of a discovered sector."""
        return self._centers[sector_id]

    def __len__(self) -> int:
        return len(self._centers)
