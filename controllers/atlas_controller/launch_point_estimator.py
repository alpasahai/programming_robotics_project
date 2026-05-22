"""LaunchPointEstimator — online recursive estimate of the attacker's launch origin.

The Think layer of the adaptive launch-point feature. Each launch is observed as
a noisy world-frame position (the Search Radar cue at first acquisition). The
estimator fuses observations into a running per-axis estimate with exponential
forgetting:

    first obs:  mu = obs
    later:      mu += alpha * (obs - mu)

This is classical noise reduction (a moving-average / 1-state-per-axis recursive
filter): variance falls as cues accumulate, while the forgetting factor `alpha`
lets the estimate track a slowly drifting launch point. `alpha` is the single
tuning knob — the dial between noise-robustness (small alpha) and adaptation
speed (large alpha). It exposes a ready-aim (pan, tilt) for the FSM IDLE state to
pre-slew toward. No dataset, no serialized model. See the launch-point spec.
"""
import math


class LaunchPointEstimator:
    """Recursive estimate of the launch origin from noisy launch observations.

    Args:
        alpha: forgetting/adaptation rate in (0, 1]. Smaller = smoother/less
            reactive; larger = tracks drift faster.
    """

    def __init__(self, alpha=0.15):
        self._alpha = alpha
        self._mu: list[float] | None = None
        self.samples = 0

    def observe(self, position) -> None:
        """Fuse one noisy launch-origin observation (world-frame [x, y, z])."""
        if self._mu is None:
            self._mu = [float(position[0]), float(position[1]), float(position[2])]
        else:
            for i in range(3):
                self._mu[i] += self._alpha * (float(position[i]) - self._mu[i])
        self.samples += 1

    def get_estimate(self):
        """Return the current launch-origin estimate [x, y, z], or None (cold start)."""
        return None if self._mu is None else list(self._mu)

    def get_ready_aim(self, turret_position):
        """Return (pan, tilt) aiming at the estimate relative to the turret, or None.

        Uses the FSM's convention: pan = atan2(dx, dy),
        tilt = atan2(dz, sqrt(dx² + dy²)). None before any observation.

        Args:
            turret_position: World-frame [x, y, z] of the turret origin (metres,
                Z-up ENU). Used to convert the world-frame estimate to a
                turret-relative bearing.

        Returns:
            (pan, tilt) in radians, or None if no observation has been made yet.
        """
        if self._mu is None:
            return None
        dx = self._mu[0] - turret_position[0]
        dy = self._mu[1] - turret_position[1]
        dz = self._mu[2] - turret_position[2]
        pan = math.atan2(dx, dy)
        tilt = math.atan2(dz, math.sqrt(dx * dx + dy * dy))
        return (pan, tilt)
