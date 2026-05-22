"""Scoreboard — operational outcome tally for the ATLAS engagement loop.

Counts projectiles ATLAS shot down (bullet hits) versus those that reached the
ground (ground hits = misses), and reports the shoot-down rate. This is the
operational measure of the system's effectiveness — and, run with pre-aim on vs
off, the measure of the adaptive launch-direction feature's value. Pure: the
controller feeds it resolution events and logs summary().
"""


class Scoreboard:
    """Running tally of shoot-downs vs ground hits."""

    def __init__(self):
        self.shot_down = 0
        self.ground_hits = 0

    def record_shoot_down(self) -> None:
        """Register a projectile destroyed by a turret bullet."""
        self.shot_down += 1

    def record_ground_hit(self) -> None:
        """Register a projectile that reached the ground (a miss)."""
        self.ground_hits += 1

    @property
    def total(self) -> int:
        """Total resolved engagements."""
        return self.shot_down + self.ground_hits

    @property
    def shoot_down_rate(self):
        """Fraction of resolved engagements that were shot down, or None if none."""
        return None if self.total == 0 else self.shot_down / self.total

    def summary(self) -> str:
        """One-line scoreboard for logging."""
        if self.total == 0:
            return "SCORE: no engagements resolved yet"
        rate = 100.0 * self.shoot_down_rate
        return (f"SCORE: shot down {self.shot_down}/{self.total} ({rate:.0f}%) "
                f"| ground hits {self.ground_hits}")
