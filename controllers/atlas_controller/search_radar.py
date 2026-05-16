"""Search Radar — wide-beam external acquisition sensor (simulated)."""
from __future__ import annotations


class SearchRadar:
    """Wide-beam search radar, external to ATLAS, simulated.

    Reads true projectile positions via Webots Supervisor getPosition() and
    adds high Gaussian noise to simulate coarse wide-beam acquisition cues.

    During SEARCH: get_detections() returns all detected objects so the FSM
    can select a target to lock onto.

    After set_target(): get_target_position() returns noisy measurements of
    the locked target, which are fused into the TrackFilter alongside FCR.

    See ADR-0003 for the continuous sensor fusion rationale.
    """

    def __init__(
        self,
        projectiles: list,
        turret_position: list[float],
        noise_std: float,
        timestep_ms: int,
    ) -> None:
        """
        Args:
            projectiles:     List of Webots Solid nodes in the scene.
            turret_position: World-frame [x, y, z] of the turret origin.
            noise_std:       Standard deviation of Gaussian noise (metres).
                             Should be significantly higher than FCR noise_std.
            timestep_ms:     Simulation timestep in milliseconds.
        """
        raise NotImplementedError

    def update(self) -> None:
        """Read all projectile positions, add noise, store as relative positions."""
        raise NotImplementedError

    def get_detections(self) -> list[list[float]]:
        """Return noisy relative positions of all detected objects.

        Used during SEARCH so the FSM can evaluate all returns and select
        one to lock onto. Returns an empty list if nothing is detected.
        """
        raise NotImplementedError

    def get_target_position(self) -> list[float] | None:
        """Return noisy relative position of the locked target [dx, dy, dz].

        Only valid after set_target() has been called. Returns None if no
        target is locked yet.
        """
        raise NotImplementedError

    def set_target(self, node) -> None:
        """Lock onto a specific node as the tracked target.

        Called at ACQUIRE alongside FireControlRadar.set_target(). After this,
        get_target_position() returns measurements for this node only.

        Args:
            node: Webots Solid node to track.
        """
        raise NotImplementedError
