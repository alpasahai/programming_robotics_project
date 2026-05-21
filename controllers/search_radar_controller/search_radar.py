"""Search Radar — wide-beam external acquisition sensor (simulated).

See ADR-0006 for the sensor-membrane and track-id identity contract.
"""
import math
import random
from dataclasses import dataclass
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


@dataclass(frozen=True)
class RadarBeamVisualSpec:
    """Renderable cone matching the SearchRadar detection beam.

    The software gate and visualisation both use this same cone volume.
    """

    origin_world: list[float]
    centre_azimuth: float
    max_range: float
    beam_width: float
    vertical_fov: float
    half_angle: float
    cone_height: float
    cone_radius: float
    cone_center_local: list[float]


class SearchRadar:
    """Wide-beam search radar, external to ATLAS, simulated.

    Reads true projectile positions via Webots Supervisor getPosition() and
    adds high Gaussian noise to simulate coarse wide-beam acquisition cues.

    Holds the projectile list (indexed by ``track_id``) and a ``track_id``-keyed
    track buffer of ``(Detection, age)`` tuples internally. Nothing outside
    this class holds a Webots node handle (ADR-0006: sensor membrane).

    During SEARCH: get_detections() returns Detection objects so the FSM can
    select a target by track_id.

    After set_target(track_id): get_target_position() returns noisy
    measurements of the locked target, which are fused into the TrackFilter
    alongside FCR.

    Gating: each projectile is only reported when:
      - target is inside the beam-aligned cone returned by
        ``get_beam_visual_spec()``

    Note: ``radar_position`` is used for gating only. ``Detection.position``
    is always reported relative to ``turret_position`` (ADR-0006 membrane).

    Scan beam parameters (``beam_width``, ``scan_rate``) rotate a narrow beam
    through all azimuths; see Task 3. Track buffer persistence (``track_timeout``)
    holds a track between beam passes: a detection refreshes the track to age 0;
    each update without a detection ages the track by 1; a track is dropped when
    ``age > track_timeout`` (i.e. after ``track_timeout + 1`` consecutive missed
    updates).

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
            beam_width:      Half-power beam width in radians. The beam is
                             gated to accept targets within beam_width / 2
                             radians of the current scan azimuth.
            scan_rate:       Scan rate in radians per update() call (per
                             simulation step). The beam azimuth advances by
                             this amount each update().
            track_timeout:   Number of consecutive missed updates before a
                             track is dropped. With ``track_timeout=N`` a
                             track survives N updates with no re-detection
                             (age 1 … N) and is dropped on update N+1
                             (age N+1 > N). Use 0 to drop after the very
                             first missed update.
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
        # Scan beam parameters (Task 3): beam advances and gates by azimuth.
        self._beam_width = beam_width
        self._scan_rate = scan_rate
        self._beam_azimuth = 0.0  # initialized to 0; advances each update()
        self._track_timeout = track_timeout
        self._rng = rng if rng is not None else random.Random()
        # Track buffer: maps track_id → (Detection, age).
        # age=0 means detected this cycle; incremented each update() without detection;
        # dropped when age > track_timeout.
        self._track_buffer: dict[int, tuple[Detection, int]] = {}
        self._target_id: int | None = None

    def get_beam_visual_spec(self) -> RadarBeamVisualSpec:
        """Return the render spec for the beam currently used by detection."""
        half_angle = self._beam_half_angle()
        cone_radius = math.tan(half_angle) * self._max_range
        return RadarBeamVisualSpec(
            origin_world=list(self._radar_position),
            centre_azimuth=self._beam_azimuth,
            max_range=self._max_range,
            beam_width=self._beam_width,
            vertical_fov=self._half_fov * 2.0,
            half_angle=half_angle,
            cone_height=self._max_range,
            cone_radius=cone_radius,
            cone_center_local=[0.0, self._max_range / 2.0, 0.0],
        )

    def _beam_half_angle(self) -> float:
        """Return the cone half-angle used by both detection and visualisation."""
        if self._beam_width >= 2 * math.pi:
            return self._half_fov
        return min(self._beam_width / 2.0, self._half_fov)

    def _beam_local_position(self, world: list[float]) -> list[float]:
        """Transform a world position into the beam's local cone frame."""
        dx = world[0] - self._radar_position[0]
        dy = world[1] - self._radar_position[1]
        dz = world[2] - self._radar_position[2]

        forward_x = math.sin(self._beam_azimuth)
        forward_y = math.cos(self._beam_azimuth)
        right_x = math.cos(self._beam_azimuth)
        right_y = -math.sin(self._beam_azimuth)

        return [
            dx * right_x + dy * right_y,
            dx * forward_x + dy * forward_y,
            dz,
        ]

    def _is_inside_beam_cone(self, world: list[float]) -> bool:
        """Return whether a world position is inside the rendered FOV cone."""
        spec = self.get_beam_visual_spec()

        if self._beam_width >= 2 * math.pi:
            _az, elevation, rng = azimuth_elevation_range(self._radar_position, world)
            return rng <= spec.max_range and abs(elevation) <= self._half_fov

        local = self._beam_local_position(world)
        if local[1] < 0.0 or local[1] > spec.max_range:
            return False
        radius_at_y = math.tan(spec.half_angle) * local[1]
        return math.sqrt(local[0] * local[0] + local[2] * local[2]) <= radius_at_y

    def update(self) -> None:
        """Read all projectile positions, gate by range/elevation/azimuth, add noise.

        Each call performs four steps in order:

        1. Advance the beam azimuth by ``scan_rate`` (wraps at 2π).
        2. Age every existing track buffer entry by +1.
        3. For each projectile in ``self._projectiles`` (index = track_id,
           per ADR-0006): reject if it falls outside the same beam-aligned cone
           volume returned by ``get_beam_visual_spec()``;
           for accepted projectiles, subtract ``turret_position``, add
           independent Gaussian noise (std=``noise_std``) on each axis, and
           overwrite the buffer entry for that track_id at age 0 with a fresh
           ``Detection``.
        4. Drop any buffer entry whose age exceeds ``track_timeout``.

        Track buffer persistence: a detected projectile refreshes its entry to
        age 0 each update it is directly detected. Between beam passes the entry
        ages by 1 per update and persists until ``age > track_timeout``; with
        ``track_timeout=N`` the track survives N updates with no re-detection.
        The buffered ``Detection`` holds the position measured at the last
        detection — it is NOT updated while the beam is away.
        """
        # (a) Advance the beam azimuth before gating.
        self._beam_azimuth = (self._beam_azimuth + self._scan_rate) % (2 * math.pi)

        # (b) Age all existing tracks by 1.
        self._track_buffer = {
            tid: (det, age + 1)
            for tid, (det, age) in self._track_buffer.items()
        }

        # (c) Gate projectiles; refresh buffer entries for those that pass.
        for index, proj in enumerate(self._projectiles):
            world = proj.getPosition()
            if not self._is_inside_beam_cone(world):
                continue

            noisy = [
                world[i] - self._turret_position[i] + self._rng.gauss(0.0, self._noise_std)
                for i in range(3)
            ]
            # Overwrite (or create) buffer entry at age 0 with fresh Detection.
            self._track_buffer[index] = (Detection(track_id=index, position=noisy), 0)

        # (d) Drop entries whose age exceeds track_timeout.
        self._track_buffer = {
            tid: (det, age)
            for tid, (det, age) in self._track_buffer.items()
            if age <= self._track_timeout
        }

    def get_detections(self) -> list[Detection]:
        """Return Detection objects for all live tracks in the buffer.

        Each Detection carries an integer track_id (the projectile's index in
        the constructor list) and the most recently measured noisy turret-relative
        position for that track. The FSM uses track_id to select and lock a target;
        it never holds a node (ADR-0006).

        Tracks persist between beam passes for up to ``track_timeout`` updates
        after the last direct detection; their Detection reflects the last fresh
        measurement taken when the beam was on the target.

        Returns a fresh list each call so callers cannot mutate internal state.
        Detection is immutable (NamedTuple) so the elements are safe to share.

        Used during SEARCH so the FSM can evaluate all returns and select
        one to lock onto. Returns an empty list before the first ``update()``
        or when no projectiles are present and the buffer is empty.
        """
        return [det for det, _age in self._track_buffer.values()]

    def get_fresh_detections(self) -> list[Detection]:
        """Return Detection objects for tracks detected on the most recent update().

        A track is *fresh* when its buffer age is 0 — the beam was directly on it
        during the last ``update()``. Tracks held between beam passes (age > 0, kept
        alive by ``track_timeout``) are excluded. This is the membrane-safe
        replacement for the controller reaching into ``_track_buffer`` to test age.

        Returns a fresh list each call; ``Detection`` is immutable so elements are
        safe to share. Returns an empty list before the first ``update()``.
        """
        return [det for det, age in self._track_buffer.values() if age == 0]

    def get_target_position(self) -> list[float] | None:
        """Return the buffered noisy position of the locked target [dx, dy, dz].

        Returns the most recently measured turret-relative position for the
        locked target, as long as its track survives in the buffer. The value
        is the last fresh measurement taken when the beam directly detected the
        target; it persists through ``track_timeout`` missed updates.

        Returns:
            Noisy turret-relative ``[dx, dy, dz]`` (metres) if the locked
            target's track is live in the buffer.
            ``None`` if ``set_target()`` has not been called yet, immediately
            after ``set_target()`` before the next ``update()``, or once the
            locked target's track has been dropped from the buffer (timed out).
        """
        if self._target_id is None:
            return None
        entry = self._track_buffer.get(self._target_id)
        if entry is None:
            return None
        det, _age = entry
        return det.position

    def set_target(self, track_id: int) -> None:
        """Lock onto a specific projectile as the tracked target.

        Validates ``track_id`` against the projectile list, keeping the
        ``track_id → index`` identity inside the sensor membrane (ADR-0006).
        The FSM holds only the integer ``track_id`` — it never receives or
        stores a node handle.

        Called at ACQUIRE (SearchRadar only — FCR is single-target, cued at
        construction and not re-targeted by the FSM; see ADR-0006). After
        this call, ``get_target_position()`` returns buffered measurements for
        this projectile only. Clears the buffer entry for the newly locked
        target so that ``get_target_position()`` returns ``None`` until the
        next ``update()`` detects and buffers it.

        Args:
            track_id: Integer index of the projectile to track, as returned
                      in ``Detection.track_id`` by ``get_detections()``.

        Raises:
            IndexError: If ``track_id`` is not a valid index into the
                        projectile list passed at construction. Failing fast
                        is intentional — callers must not pass a stale or
                        out-of-bounds track_id.
        """
        # Bounds-check: raises IndexError for an invalid track_id.
        # The node handle is not stored — track_id is sufficient for all
        # buffer lookups after the Task 4 rewrite.
        _ = self._projectiles[track_id]
        self._target_id = track_id
        # Clear any stale buffer entry for the newly locked target so that
        # get_target_position() returns None until the next update() detects it.
        self._track_buffer.pop(track_id, None)
