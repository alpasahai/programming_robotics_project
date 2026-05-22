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
        max_clusters: hard cap on the number of sectors. ``None`` (default) is
            unbounded. When at capacity, a bearing that matches no existing
            cluster within ``tol_rad`` folds into the nearest existing cluster
            instead of opening a new one — so returned ids stay in
            ``[0, max_clusters)``. Protects fixed-width downstream consumers
            (one-hot features, ``partial_fit`` classes) from out-of-range ids.
    """

    def __init__(self, tol_rad, nudge=0.2, max_clusters=None):
        self._tol = tol_rad
        self._nudge = nudge
        self._max_clusters = max_clusters
        self._centers: list[float] = []

    def observe(self, bearing) -> int:
        """Assign ``bearing`` to a sector id, discovering a new one if needed."""
        best_id, best_dist = None, None
        for i, c in enumerate(self._centers):
            d = abs(_ang_diff(bearing, c))
            if best_dist is None or d < best_dist:
                best_id, best_dist = i, d
        at_capacity = (
            self._max_clusters is not None
            and len(self._centers) >= self._max_clusters
        )
        if best_id is not None and (best_dist <= self._tol or at_capacity):
            # Join the nearest cluster and nudge its centre (wrap-safe). When at
            # capacity we fold even out-of-tol bearings here rather than opening
            # a new (out-of-range) cluster.
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
