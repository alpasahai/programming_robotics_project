"""Launch-observation latch for the ATLAS turret.

Encapsulates the arm/disarm logic that ensures exactly ONE launch-origin
observation per engagement, avoiding stale cues and mid-flight re-acquisitions.

State machine
-------------
DISARMED (initial)
  - observation(resolved=True,  cue) → arm, return None   (stale-cue guard)
  - observation(resolved=False, cue) → return None         (not yet armed)

ARMED
  - observation(resolved=True,  cue) → re-arm, return None (back-to-back resolutions)
  - observation(resolved=False, cue=not None) → disarm, return cue
  - observation(resolved=False, cue=None)     → return None

Cold-start (first engagement): the gate starts DISARMED so the very first
radar acquisition — which may be mid-flight rather than near the launch origin
— is never observed. The gate arms only after the first hit is registered.
"""


class LaunchObservationGate:
    """Arm/disarm latch that yields at most one launch-origin observation per engagement.

    Usage::

        gate = LaunchObservationGate()

        # Inside the step loop:
        obs = gate.observation(
            resolved = ground_hit_link.hit_this_step() or bullet_hit_link.hit_this_step(),
            cue      = cue_link.get_cue(),
        )
        if obs is not None:
            launch_point_estimator.observe(obs)

    The gate is DISARMED at construction so the cold-start (first engagement,
    before any resolution) contributes no observation; the turret falls back to
    its fixed idle beam (ADR-0014).
    """

    def __init__(self):
        """Initialise DISARMED."""
        self._armed: bool = False

    def observation(self, resolved: bool, cue) -> list | None:
        """Advance the latch one step and return the cue to observe, or None.

        Args:
            resolved: True when a ground-hit or bullet-hit was received this
                      step (i.e. the engagement just ended / the ball was
                      destroyed). This arms the gate for the *next* step.
            cue:      The current search-radar cue (a ``[x, y, z]`` list), or
                      None when no cue is available this step.

        Returns:
            The cue to pass to ``LaunchPointEstimator.observe()``, or None.
            Exactly one non-None value is returned per engagement (debounce).

        Behaviour:
            - resolution step (``resolved=True``): arm the gate, always return
              None — the cue present on this step is stale (mid-flight or
              impact position, not the launch origin).
            - non-resolution step, ARMED, cue available: return the cue and
              disarm (debounce — subsequent cues in the same engagement are
              suppressed until the next resolution).
            - all other cases: return None.
        """
        if resolved:
            self._armed = True
            return None

        if self._armed and cue is not None:
            self._armed = False
            return cue

        return None
