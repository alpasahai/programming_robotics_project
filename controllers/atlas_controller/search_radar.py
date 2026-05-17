"""Search Radar — wide-beam external acquisition sensor (simulated)."""
import random


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
        rng: random.Random | None = None,
    ) -> None:
        """
        Args:
            projectiles:     List of Webots Solid nodes in the scene.
            turret_position: World-frame [x, y, z] of the turret origin.
            noise_std:       Standard deviation of Gaussian noise (metres).
                             Should be significantly higher than FCR noise_std.
            timestep_ms:     Simulation timestep in milliseconds.
                             Stored for interface consistency; currently unused
                             by any method.
            rng:             Optional seeded ``random.Random`` instance for
                             deterministic noise in tests. When ``None``, a
                             fresh ``random.Random()`` is created.
        """
        self._projectiles = projectiles
        self._turret_position = turret_position
        self._noise_std = noise_std
        self._timestep_ms = timestep_ms
        self._rng = rng if rng is not None else random.Random()
        self._detections: list[list[float]] = []
        self._target_node = None
        self._target_position: list[float] | None = None

    def update(self) -> None:
        """Read all projectile positions, add noise, store as relative positions.

        For each projectile in ``self._projectiles``:
        1. Reads its world-frame position via ``getPosition()``.
        2. Subtracts ``turret_position`` to get a turret-relative vector.
        3. Adds independent Gaussian noise (std ``noise_std``) on each axis.

        Populates both the full detections list (``get_detections()``) and,
        when a target has been locked via ``set_target()``, the single-target
        measurement (``get_target_position()``).
        """
        self._detections = []
        for proj in self._projectiles:
            world = proj.getPosition()
            noisy = [
                world[i] - self._turret_position[i] + self._rng.gauss(0.0, self._noise_std)
                for i in range(3)
            ]
            self._detections.append(noisy)
            if proj is self._target_node:
                self._target_position = noisy

    def get_detections(self) -> list[list[float]]:
        """Return noisy relative positions of all detected objects.

        Used during SEARCH so the FSM can evaluate all returns and select
        one to lock onto. Returns an empty list before the first ``update()``
        or when no projectiles are present.
        """
        return self._detections

    def get_target_position(self) -> list[float] | None:
        """Return noisy relative position of the locked target [dx, dy, dz].

        Returns:
            Noisy turret-relative ``[dx, dy, dz]`` (metres) of the locked
            target, updated each ``update()`` call.
            ``None`` if ``set_target()`` has not been called yet, or
            immediately after ``set_target()`` before the next ``update()``.
        """
        return self._target_position

    def set_target(self, node) -> None:
        """Lock onto a specific node as the tracked target.

        Called at ACQUIRE alongside ``FireControlRadar.set_target()``. After
        this call, ``get_target_position()`` returns measurements for this
        node only. Clears any previously stored target position so that
        ``get_target_position()`` returns ``None`` until the next
        ``update()``.

        Args:
            node: Webots Solid node (or ``StubProjectile`` in tests) to track.
        """
        self._target_node = node
        self._target_position = None
