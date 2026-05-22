"""LaunchLatch — segment the Search Radar cue stream into one launch observation
per engagement.

ATLAS perceives launches radar-only. A launch is the FIRST fresh cue after the
previous engagement's cue has been cleared. The FSM clears the cue on RESET, but
RESET runs a step LATER than the resolution is signalled, so the cue lingers as a
stale in-flight position for at least one step (and through the respawn gap).
Observing it directly would train the predictor on the resolved projectile, not
the next launch. This latch therefore waits to actually SEE the cue go None
(cleared) before accepting the next non-None cue — order-independent of when
RESET runs. See the attack-plan spec and ADR-0014.
"""


class LaunchLatch:
    """One launch observation per engagement: the first fresh cue post-clear."""

    def __init__(self):
        self._armed = False
        self._cleared_since_arm = False

    def update(self, resolved, cue):
        """Advance the latch one step; return the cue to observe, or None.

        Args:
            resolved: True on a step where a ground/bullet hit resolved the
                current engagement (re-arms the latch).
            cue: the current Search Radar cue ([x, y, z] world frame) or None.

        Returns:
            The cue to treat as a launch observation, or None if this step does
            not yield one.
        """
        if resolved:
            self._armed = True
            self._cleared_since_arm = False
            return None
        if not self._armed:
            return None
        if cue is None:
            self._cleared_since_arm = True  # RESET has cleared the stale cue
            return None
        if self._cleared_since_arm:
            self._armed = False
            self._cleared_since_arm = False
            return cue
        return None  # stale cue (not yet cleared) → ignore
