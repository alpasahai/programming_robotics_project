"""Fire-Control Radar — ATLAS's own narrow-beam on-board sensor."""
from __future__ import annotations


class FireControlRadar:
    """ATLAS's narrow-beam on-board radar sensor.

    Currently simulated via Webots Supervisor getPosition() with optional
    Gaussian noise. Returns target position relative to the turret in
    Cartesian coordinates [dx, dy, dz] (Z-up ENU convention).

    See ADR-0004 for upgrade path to a real Webots sensor node.
    """

    def __init__(
        self,
        projectile,
        turret_position: list[float],
        noise_std: float = 0.0,
    ) -> None:
        """
        Args:
            projectile:      Webots Solid node (or StubProjectile in tests).
            turret_position: World-frame [x, y, z] of the turret origin.
                             Used to convert world coordinates to relative.
            noise_std:       Standard deviation of Gaussian noise added to
                             each axis. Default 0.0 (noiseless simulation).
        """
        raise NotImplementedError

    def update(self) -> None:
        """Read projectile world position, add noise, store as relative [dx, dy, dz]."""
        raise NotImplementedError

    def get_target_position(self) -> list[float] | None:
        """Return most recent relative position [dx, dy, dz] from turret.

        Returns None before the first call to update().
        """
        raise NotImplementedError

    def set_target(self, node) -> None:
        """Lock onto a specific Webots node as the tracked target.

        Called by the FSM at the ACQUIRE transition when multiple projectiles
        are present. Clears the last stored position.

        Args:
            node: Webots Solid node to track.
        """
        raise NotImplementedError
