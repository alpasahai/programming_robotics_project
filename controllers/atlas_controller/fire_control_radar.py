"""Fire-Control Radar — ATLAS's own narrow-beam on-board sensor.

Detection is gated by a narrow FOV cone around the turret's current aim
boresight and by a maximum range. The boresight is passed in on every
``update()`` call, since the FCR has no boresight of its own — it uses
the turret's real aim direction.
"""
import math
import random
from typing import NamedTuple

import geometry


class GateDiagnostic(NamedTuple):
    """Snapshot of the gating values from the most recent ``update()`` call.

    All angle values are in radians; range values are in metres.

    Attributes:
        boresight_az:    Azimuth commanded to ``update()`` (Z-up ENU, radians).
        boresight_el:    Elevation commanded to ``update()`` (radians).
        target_az:       Computed azimuth of the projectile from ``fcr_position``
                         (radians).
        target_el:       Computed elevation of the projectile from ``fcr_position``
                         (radians).
        target_range:    Distance from ``fcr_position`` to the projectile (metres).
        separation:      Angular separation between boresight and target (radians).
        fov_half_angle:  The FCR's cone half-angle threshold (radians, constant).
        max_range:       The FCR's maximum detection range (metres, constant).
        rejection_reason: ``None`` when both gates pass (target detected).
                          A string containing ``'cone'`` when
                          ``separation > fov_half_angle``, ``'range'`` when
                          ``target_range > max_range``, or both words when both
                          conditions fail simultaneously.
    """

    boresight_az: float
    boresight_el: float
    target_az: float
    target_el: float
    target_range: float
    separation: float
    fov_half_angle: float
    max_range: float
    rejection_reason: str | None


class FireControlRadar:
    """ATLAS's narrow-beam on-board radar sensor with FOV cone gating.

    Simulated via Webots Supervisor ``getPosition()`` with optional Gaussian
    noise. On each ``update(boresight_az, boresight_el)`` call the radar:

    1. Fetches the projectile's world position.
    2. Converts it to (azimuth, elevation, range) relative to ``fcr_position``.
    3. Gates detection: the target must be within ``fov_half_angle`` of the
       boresight direction AND within ``max_range``.
    4. On detection: stores ``(world − turret_position)`` + Gaussian noise.
       On no detection: stores ``None``.

    ``get_target_position()`` returns the most recently stored value.
    ``is_locked()`` returns True iff the last ``update()`` detected the target.

    The position frame is Z-up ENU: dx = East, dy = North, dz = Up, all
    measured from the turret origin.

    See ADR-0008 (supersedes ADR-0004): the FCR remains permanently simulated
    as the turret-aim sensor; the former upgrade path to a generic Webots
    ``Radar`` or ``RangeFinder`` node is explicitly rejected.
    """

    def __init__(
        self,
        projectile,
        fcr_position: list[float],
        turret_position: list[float],
        fov_half_angle: float,
        max_range: float,
        noise_std: float = 0.0,
        rng: random.Random | None = None,
    ) -> None:
        """Initialise the radar, binding it to a target node and geometry.

        Args:
            projectile:      Webots Solid node (or StubProjectile in tests).
                             Must expose ``getPosition() -> [x, y, z]``.
            fcr_position:    World-frame [x, y, z] of the FCR sensor origin
                             (co-located with the turret). Used as the observer
                             origin for azimuth/elevation/range computation.
            turret_position: World-frame [x, y, z] of the turret origin.
                             Subtracted from every detected world position to
                             produce turret-relative coordinates.
            fov_half_angle:  Half-angle of the detection cone in radians.
                             The target is detected when its angular separation
                             from the boresight is ≤ ``fov_half_angle``.
            max_range:       Maximum detection range in metres. The target must
                             be within this distance of ``fcr_position``.
            noise_std:       Standard deviation (metres) of zero-mean Gaussian
                             noise added independently to each axis of the
                             detected position. Default 0.0 (noiseless).
            rng:             Optional ``random.Random`` instance used for noise
                             sampling. When ``None`` (default), a fresh
                             ``random.Random()`` instance is created. Pass a
                             seeded instance (e.g. ``random.Random(42)``) to
                             make noise deterministic in tests.
        """
        self._projectile = projectile
        self._fcr_position = list(fcr_position)
        self._turret_position = list(turret_position)
        self._fov_half_angle = fov_half_angle
        self._max_range = max_range
        self._noise_std = noise_std
        self._rng = rng if rng is not None else random.Random()
        self._last_position: list[float] | None = None
        self._locked: bool = False
        self._last_gate_diagnostic: GateDiagnostic | None = None

    def update(self, boresight_az: float, boresight_el: float) -> None:
        """Gate detection against the turret's current aim, then store the result.

        Fetches the projectile's world position, computes its azimuth, elevation,
        and range relative to ``fcr_position``, and checks:
            - angular separation from boresight ≤ ``fov_half_angle``
            - range ≤ ``max_range``

        On detection: stores ``(world − turret_position)`` with optional
        Gaussian noise on each axis.
        On no detection: stores ``None`` and sets ``is_locked()`` to False.

        Args:
            boresight_az: Turret's current pan (azimuth) in radians, Z-up ENU.
            boresight_el: Turret's current tilt (elevation) in radians.
        """
        world = self._projectile.getPosition()
        target_az, target_el, target_range = geometry.azimuth_elevation_range(
            self._fcr_position, world
        )

        separation = geometry.angular_separation(
            boresight_az, boresight_el, target_az, target_el
        )

        cone_fail = separation > self._fov_half_angle
        range_fail = target_range > self._max_range

        if cone_fail or range_fail:
            parts = []
            if cone_fail:
                parts.append("cone")
            if range_fail:
                parts.append("range")
            rejection_reason: str | None = "+".join(parts)
        else:
            rejection_reason = None

        self._last_gate_diagnostic = GateDiagnostic(
            boresight_az=boresight_az,
            boresight_el=boresight_el,
            target_az=target_az,
            target_el=target_el,
            target_range=target_range,
            separation=separation,
            fov_half_angle=self._fov_half_angle,
            max_range=self._max_range,
            rejection_reason=rejection_reason,
        )

        if not cone_fail and not range_fail:
            relative = [world[i] - self._turret_position[i] for i in range(3)]
            if self._noise_std:
                relative = [
                    relative[i] + self._rng.gauss(0.0, self._noise_std)
                    for i in range(3)
                ]
            self._last_position = relative
            self._locked = True
        else:
            self._last_position = None
            self._locked = False

    def get_target_position(self) -> list[float] | None:
        """Return the most recent turret-relative detection, or None.

        The coordinate frame is Z-up ENU: dx = East, dy = North, dz = Up,
        all measured from the turret origin.

        Returns:
            ``[dx, dy, dz]`` when the last ``update()`` detected the target;
            ``None`` before the first ``update()`` or when the target is
            outside the FOV cone or beyond ``max_range``.
        """
        return self._last_position

    def is_locked(self) -> bool:
        """Return True iff the most recent ``update()`` detected the target.

        Returns:
            True when the target is inside the FOV cone and within
            ``max_range`` as of the last ``update()`` call. False before any
            ``update()`` has been called, or when gating fails.
        """
        return self._locked

    def get_last_gate_diagnostic(self) -> GateDiagnostic | None:
        """Return the gating values recorded during the most recent ``update()``.

        Intended for diagnostic telemetry only: callers must not use this to
        make engagement decisions. The returned snapshot is immutable
        (``NamedTuple``) and safe to log without copying.

        Returns:
            A ``GateDiagnostic`` with boresight, target angles/range,
            angular separation, gate thresholds, and the rejection reason
            (``None`` when the target was inside both gates). Returns
            ``None`` before the first ``update()`` call.
        """
        return self._last_gate_diagnostic
