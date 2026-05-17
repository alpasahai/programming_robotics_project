"""Search Radar — wide-beam external acquisition sensor (simulated).

See ADR-0006 for the sensor-membrane and track-id identity contract.
"""
import random
from typing import NamedTuple


class Detection(NamedTuple):
    """An immutable detection returned by SearchRadar.get_detections().

    Carries only the information the FSM is permitted to see: an opaque
    integer ``track_id`` and a noisy turret-relative position. No Webots
    node handle escapes the SearchRadar membrane (ADR-0006).

    Attributes:
        track_id: Integer index of the detected projectile in the list
                  passed to SearchRadar at construction. Stable for the
                  lifetime of the SearchRadar instance.
        position: Noisy turret-relative ``[dx, dy, dz]`` in metres
                  (Z-up ENU). Updated each ``SearchRadar.update()`` call.
    """
    track_id: int
    position: list


class SearchRadar:
    """Wide-beam search radar, external to ATLAS, simulated.

    Reads true projectile positions via Webots Supervisor getPosition() and
    adds high Gaussian noise to simulate coarse wide-beam acquisition cues.

    Holds the ``track_id → Webots node`` mapping internally. Nothing outside
    this class holds a node handle (ADR-0006: sensor membrane).

    During SEARCH: get_detections() returns Detection objects so the FSM can
    select a target by track_id.

    After set_target(track_id): get_target_position() returns noisy
    measurements of the locked target, which are fused into the TrackFilter
    alongside FCR.

    See ADR-0003 for the continuous sensor fusion rationale.
    See ADR-0006 for the sensor-membrane and track-id identity contract.
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
            projectiles:     List of Webots Solid nodes in the scene. The
                             index of each node in this list is its track_id
                             (ADR-0006). The list must not be reordered after
                             construction so that track_ids remain stable.
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
        self._turret_position = list(turret_position)
        self._noise_std = noise_std
        self._timestep_ms = timestep_ms
        self._rng = rng if rng is not None else random.Random()
        self._detections: list[Detection] = []
        self._target_node = None
        self._target_position: list[float] | None = None

    def update(self) -> None:
        """Read all projectile positions, add noise, store as Detection objects.

        For each projectile in ``self._projectiles`` (enumerated so that the
        index serves as track_id per ADR-0006):
        1. Reads its world-frame position via ``getPosition()``.
        2. Subtracts ``turret_position`` to get a turret-relative vector.
        3. Adds independent Gaussian noise (std ``noise_std``) on each axis.
        4. Stores a ``Detection(track_id=index, position=noisy_vector)``.

        Populates both the full detections list (``get_detections()``) and,
        when a target has been locked via ``set_target()``, the single-target
        measurement (``get_target_position()``).
        """
        self._detections = []
        for index, proj in enumerate(self._projectiles):
            world = proj.getPosition()
            noisy = [
                world[i] - self._turret_position[i] + self._rng.gauss(0.0, self._noise_std)
                for i in range(3)
            ]
            self._detections.append(Detection(track_id=index, position=noisy))
            if proj is self._target_node:
                self._target_position = noisy

    def get_detections(self) -> list[Detection]:
        """Return Detection objects for all detected projectiles.

        Each Detection carries an integer track_id (the projectile's index in
        the constructor list) and a noisy turret-relative position. The FSM
        uses track_id to select and lock a target; it never holds a node
        (ADR-0006).

        Returns a fresh list each call so callers cannot mutate internal state.
        Detection is immutable (NamedTuple) so the elements are safe to share.

        Used during SEARCH so the FSM can evaluate all returns and select
        one to lock onto. Returns an empty list before the first ``update()``
        or when no projectiles are present.
        """
        return list(self._detections)

    def get_target_position(self) -> list[float] | None:
        """Return noisy relative position of the locked target [dx, dy, dz].

        Returns:
            Noisy turret-relative ``[dx, dy, dz]`` (metres) of the locked
            target, updated each ``update()`` call.
            ``None`` if ``set_target()`` has not been called yet, or
            immediately after ``set_target()`` before the next ``update()``.
        """
        return self._target_position

    def set_target(self, track_id: int) -> None:
        """Lock onto a specific projectile as the tracked target.

        Resolves the integer ``track_id`` to the corresponding Webots node
        internally, keeping the node handle inside the sensor membrane
        (ADR-0006). The FSM holds only the integer ``track_id`` — it never
        receives or stores a node handle.

        Called at ACQUIRE (SearchRadar only — FCR is single-target, cued at
        construction and not re-targeted by the FSM; see ADR-0006). After
        this call, ``get_target_position()`` returns measurements for this
        projectile only. Clears any previously stored target position so that
        ``get_target_position()`` returns ``None`` until the next
        ``update()``.

        Args:
            track_id: Integer index of the projectile to track, as returned
                      in ``Detection.track_id`` by ``get_detections()``.
        """
        self._target_node = self._projectiles[track_id]
        self._target_position = None
