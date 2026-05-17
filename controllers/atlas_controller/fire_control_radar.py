"""Fire-Control Radar — ATLAS's own narrow-beam on-board sensor."""
from __future__ import annotations

import random


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
        """Initialise the radar, binding it to a target node and turret origin.

        Args:
            projectile:      Webots Solid node (or StubProjectile in tests).
                             Must expose ``getPosition() -> [x, y, z]``.
            turret_position: World-frame [x, y, z] of the turret origin.
                             Subtracted from every reading to produce relative
                             coordinates.
            noise_std:       Standard deviation (metres) of zero-mean Gaussian
                             noise added independently to each axis on every
                             ``update()`` call. Default 0.0 (noiseless).
        """
        self._projectile = projectile
        self._turret_position = list(turret_position)
        self._noise_std = noise_std
        self._last_position: list[float] | None = None

    def update(self) -> None:
        """Read the tracked node's world position, convert to turret-relative coords, and store.

        Subtracts ``turret_position`` from the world-frame reading, then adds
        independent zero-mean Gaussian noise (std ``noise_std``) to each axis.
        The result is stored and returned by the next ``get_target_position()``
        call.
        """
        world = self._projectile.getPosition()
        relative = [world[i] - self._turret_position[i] for i in range(3)]
        if self._noise_std != 0.0:
            relative = [
                relative[i] + random.gauss(0.0, self._noise_std)
                for i in range(3)
            ]
        self._last_position = relative

    def get_target_position(self) -> list[float] | None:
        """Return the most recent turret-relative position [dx, dy, dz] in metres.

        The coordinate frame is Z-up ENU: dx = East, dy = North, dz = Up,
        all measured from the turret origin.

        Returns:
            ``[dx, dy, dz]`` after the first ``update()`` call; ``None`` before.
        """
        return self._last_position

    def set_target(self, node) -> None:
        """Lock onto a specific Webots node as the tracked target.

        Called by the FSM at the ACQUIRE transition when a specific projectile
        is selected from among several candidates. Replaces the previously
        tracked node and clears the stored position, so ``get_target_position()``
        returns ``None`` until the next ``update()`` call.

        Args:
            node: Webots Solid node (or stub) to track. Must expose
                  ``getPosition() -> [x, y, z]``.
        """
        self._projectile = node
        self._last_position = None
