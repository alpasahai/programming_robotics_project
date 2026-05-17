"""Search Radar — wide-beam external acquisition sensor (simulated).

See ADR-0006 for the sensor-membrane and track-id identity contract.
"""
import math
import random
from typing import NamedTuple

from geometry import azimuth_elevation_range


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
    position: list[float]


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

    Gating: each projectile is only reported when both:
      - Euclidean distance from ``radar_position`` ≤ ``max_range``
      - |elevation| (Z-up ENU) ≤ ``vertical_fov / 2``

    Note: ``radar_position`` is used for gating only. ``Detection.position``
    is always reported relative to ``turret_position`` (ADR-0006 membrane).

    Deferred scan parameters (``beam_width``, ``scan_rate``, ``track_timeout``)
    are accepted and stored now so the constructor signature is stable, but no
    scan logic is active yet (Tasks 3–4).

    See ADR-0003 for the continuous sensor fusion rationale.
    See ADR-0006 for the sensor-membrane and track-id identity contract.
    """

    def __init__(
        self,
        projectiles: list,
        turret_position: list[float],
        radar_position: list[float],
        noise_std: float,
        timestep_ms: int,
        max_range: float,
        vertical_fov: float,
        beam_width: float = math.radians(10),
        scan_rate: float = math.radians(30),
        track_timeout: int = 3,
        rng: random.Random | None = None,
    ) -> None:
        """
        Args:
            projectiles:     List of Webots Solid nodes in the scene. The
                             index of each node in this list is its track_id
                             (ADR-0006). The list must not be reordered after
                             construction so that track_ids remain stable.
            turret_position: World-frame [x, y, z] of the turret origin.
                             Used only to compute the turret-relative output
                             position in Detection objects.
            radar_position:  World-frame [x, y, z] of the radar antenna phase
                             centre. Used for range and elevation gating.
                             May differ from turret_position.
            noise_std:       Standard deviation of Gaussian noise (metres).
                             Should be significantly higher than FCR noise_std.
            timestep_ms:     Simulation timestep in milliseconds.
                             Stored for interface consistency; currently unused
                             by any method.
            max_range:       Maximum detection range in metres. Projectiles
                             farther than this from radar_position are ignored.
            vertical_fov:    Full vertical field of view in radians. A
                             projectile is accepted only when
                             |elevation| ≤ vertical_fov / 2.
            beam_width:      (Deferred — Task 3) Half-power beam width in
                             radians. Stored but not yet used in scan logic.
            scan_rate:       (Deferred — Task 3) Scan rate in radians per
                             second. Stored but not yet used in scan logic.
            track_timeout:   (Deferred — Task 4) Number of missed updates
                             before a track is dropped. Stored but not yet
                             used.
            rng:             Optional seeded ``random.Random`` instance for
                             deterministic noise in tests. When ``None``, a
                             fresh ``random.Random()`` is created.
        """
        self._projectiles = projectiles
        self._turret_position = list(turret_position)
        self._radar_position = list(radar_position)
        self._noise_std = noise_std
        self._timestep_ms = timestep_ms
        self._max_range = max_range
        self._half_fov = vertical_fov / 2.0
        # Deferred scan parameters (Tasks 3–4): stored, not yet used.
        self._beam_width = beam_width
        self._scan_rate = scan_rate
        self._track_timeout = track_timeout
        self._rng = rng if rng is not None else random.Random()
        self._detections: list[Detection] = []
        self._target_node = None
        self._target_position: list[float] | None = None

    def update(self) -> None:
        """Read all projectile positions, gate by range and FOV, add noise.

        For each projectile in ``self._projectiles`` (enumerated so that the
        index serves as track_id per ADR-0006):
        1. Reads its world-frame position via ``getPosition()``.
        2. Computes azimuth, elevation, and range relative to radar_position
           using ``geometry.azimuth_elevation_range``.
        3. Rejects the projectile if range > max_range or
           |elevation| > vertical_fov / 2.
        4. For accepted projectiles: subtracts turret_position (not
           radar_position) and adds independent Gaussian noise on each axis.
        5. Stores a ``Detection(track_id=index, position=noisy_vector)``.

        The locked target (set via ``set_target()``) is only updated when it
        passes the gate. If it fails the gate in a given cycle,
        ``get_target_position()`` retains the previous reading (stale but
        non-None, does not crash).
        """
        self._detections = []
        for index, proj in enumerate(self._projectiles):
            world = proj.getPosition()
            _az, elevation, rng = azimuth_elevation_range(self._radar_position, world)

            if rng > self._max_range:
                continue
            if abs(elevation) > self._half_fov:
                continue

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
        or when no projectiles are present or all are gated out.
        """
        return list(self._detections)

    def get_target_position(self) -> list[float] | None:
        """Return noisy relative position of the locked target [dx, dy, dz].

        Returns:
            Noisy turret-relative ``[dx, dy, dz]`` (metres) of the locked
            target, updated each ``update()`` call when the target passes the
            range and FOV gates.
            ``None`` if ``set_target()`` has not been called yet, or
            immediately after ``set_target()`` before the next ``update()``.
            When the locked target fails the gate in a cycle, the previous
            reading is retained (stale) and this method does not crash.
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

        Raises:
            IndexError: If ``track_id`` is not a valid index into the
                        projectile list passed at construction. Failing fast
                        is intentional — callers must not pass a stale or
                        out-of-bounds track_id.
        """
        self._target_node = self._projectiles[track_id]
        self._target_position = None
