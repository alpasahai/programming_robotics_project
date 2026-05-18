"""Atlas FSM — 7-state finite state machine governing turret behaviour."""

import logging
import math
from dataclasses import dataclass

_log = logging.getLogger("AtlasFSM")

# Dedicated logger for raw motor commands. Its distinct name ("AtlasFSM.motors")
# makes every setPosition() call easy to spot/filter in the Webots console and
# atlas_telemetry.log — used to diagnose twitchy turret motion.
_motor_log = logging.getLogger("AtlasFSM.motors")


@dataclass
class SensorSuite:
    """All sensing and processing components (Sense + Think layers).

    Injected into AtlasFSM at construction. Swap out in tests by substituting
    stub implementations for each field.
    """

    search_radar: object  # SearchRadar
    fcr: object  # FireControlRadar
    track_filter: object  # TrackFilter
    ballistic_predictor: object  # BallisticTrajectoryPredictor


@dataclass
class TurretHardware:
    """All actuation components and physical configuration (Act layer).

    Injected into AtlasFSM at construction.
    """

    pan_motor: object  # Webots RotationalMotor
    tilt_motor: object  # Webots RotationalMotor
    turret_position: list[float]  # World-frame [x, y, z] of turret origin
    timestep_ms: int


@dataclass
class FSMConfig:
    """Tunable parameters for AtlasFSM behaviour.

    All fields have sensible defaults for normal operation. Override in tests
    to speed up state transitions (e.g. ACQUIRE_FRAMES=1) or adjust thresholds.
    """

    max_range: float = 10.0  # metres — target beyond this → RESET
    ground_threshold: float = 0.1  # metres world-Z — below this → landed
    acquire_frames: int = 3  # consecutive detections to leave SEARCH
    min_track_frames: int = 15  # TrackFilter frames before PREDICT
    # lookahead_steps tuned for the ATLAS projectile (~1.6 m apex, ~1.2 s flight):
    # at a 32 ms timestep, 10 steps = 0.32 s, keeping the predicted intercept
    # airborne. A longer lookahead overshoots the projectile's landing and the
    # intercept falls below ground_threshold — see ADR-0001 and the regression
    # test test_default_lookahead_keeps_intercept_above_ground.
    aim_error_threshold: float = (
        0.05  # radians — guards the AIMING→ENGAGING transition.
    )
    # _do_aim measures the angular offset between the desired
    # intercept angles and the FSM's own last-commanded angles;
    # since the intercept is static (fixed by PREDICT), this
    # reaches zero within ~2 steps regardless of the threshold.
    # A true mechanical-convergence check would require a Webots
    # PositionSensor (not currently fitted). See _do_aim for the
    # full rationale.
    search_speed: float = 0.02  # radians per step during pan sweep
    search_pan_limit: float = 1.4  # radians (~80°) sweep extent
    lookahead_steps: int = 10  # timesteps ahead for intercept (see note above)


class AtlasFSM:
    """7-state FSM governing all ATLAS turret behaviour.

    States
    ------
    SEARCH   : Pan sweep. SearchRadar scans for detections.
    ACQUIRE  : Target selected. FCR, SearchRadar, and TrackFilter locked onto
               chosen node. Waits for TrackFilter to initialise.
    TRACK    : TrackFilter fusing both sensors. Turret follows filtered position.
               Waits for MIN_TRACK_FRAMES before attempting PREDICT.
    PREDICT  : BallisticTrajectoryPredictor computes intercept point. Validates
               intercept is above ground and in range before AIMING.
    AIMING   : Turret slews to intercept point. Waits for aim error < threshold.
    ENGAGING : Laser active. Turret holds aim. Transitions to RESET when target
               goes out of range or hits ground.
    RESET    : Clears all state. Returns to SEARCH.

    Sensor fusion
    -------------
    Both FCR and SearchRadar feed the TrackFilter every timestep throughout
    all states (continuous fusion). FCR dominates due to lower R. The FSM
    does not gate sensors — it only calls set_target() at ACQUIRE to lock
    both sensors onto the chosen node.

    Coordinate system: Z-up ENU.
        pan  = atan2(dx, dy)
        tilt = atan2(dz, sqrt(dx² + dy²))

    See ADR-0003 for continuous fusion rationale.
    """

    SEARCH = "SEARCH"
    ACQUIRE = "ACQUIRE"
    TRACK = "TRACK"
    PREDICT = "PREDICT"
    AIMING = "AIMING"
    ENGAGING = "ENGAGING"
    RESET = "RESET"

    def __init__(
        self,
        sensors: SensorSuite,
        hardware: TurretHardware,
        config: FSMConfig | None = None,
    ) -> None:
        """Initialise the FSM in SEARCH state with all per-state bookkeeping.

        Args:
            sensors:  All sensing/processing components. See SensorSuite.
            hardware: Turret motors, world position, and timestep. See TurretHardware.
            config:   Tunable FSM parameters. Defaults to FSMConfig() if not provided.
        """
        self.sensors = sensors
        self.hardware = hardware
        self.config = config if config is not None else FSMConfig()

        self.state = self.SEARCH

        # SEARCH bookkeeping
        self._detection_count = 0  # consecutive steps with at least one detection
        self._pan_angle = 0.0  # current pan motor angle (radians)
        self._pan_direction = 1  # +1 sweeping positive, -1 sweeping negative

        # Last angle actually commanded to the pan motor (radians, unwrapped).
        # Needed because pan = atan2(dx, dy) is discontinuous: when the target
        # is roughly behind the turret it snaps between +pi and -pi as dx jitters
        # around zero. Those are the same heading, but feeding them to the motor
        # as absolute positions ~2*pi apart makes it whip a near-full turn each
        # step. _aim_at() unwraps each new pan command relative to this value so
        # the motor always takes the short way round. See _unwrap_pan().
        self._last_pan_cmd = 0.0

        # ACQUIRE bookkeeping
        self._target = None  # selected target (detection position vector)
        self._acquire_entry_done = False  # guard: entry actions fire exactly once

        # TRACK bookkeeping
        self._track_frames = 0  # consecutive steps spent in TRACK

        # PREDICT bookkeeping
        self._intercept = None  # validated intercept point for AIMING

        # AIMING / ENGAGING bookkeeping
        self._commanded_pan = 0.0  # last pan angle sent to the pan motor (radians)
        self._commanded_tilt = 0.0  # last tilt angle sent to the tilt motor (radians)
        self.laser_active = False  # True while ENGAGING; read by the controller

    def step(self) -> None:
        """Advance FSM by one timestep.

        Executes the active state's logic and commands motors. Call once per
        simulation step, after search_radar.update(), fcr.update(),
        track_filter.predict(), track_filter.update_fcr(), and
        track_filter.update_search() have all been called in the main loop.
        """
        if self.state == self.SEARCH:
            self._do_search()
        elif self.state == self.ACQUIRE:
            self._do_acquire()
        elif self.state == self.TRACK:
            self._do_track()
        elif self.state == self.PREDICT:
            self._do_predict()
        elif self.state == self.AIMING:
            self._do_aim()
        elif self.state == self.ENGAGING:
            self._do_engage()
        elif self.state == self.RESET:
            self._do_reset()

    # --------------------------------------------------------- state handlers

    def _do_search(self) -> None:
        """Execute one timestep of SEARCH state logic.

        Pan sweep: oscillates the pan motor between -search_pan_limit and
        +search_pan_limit, advancing search_speed radians per step and
        reversing direction when a limit is reached.

        Simultaneously reads search_radar detections and counts consecutive
        steps that contain at least one detection. Any step with no detections
        resets the counter to zero. When the count reaches acquire_frames the
        first detection is selected as the target and the FSM transitions to
        ACQUIRE.
        """
        # --- pan sweep ---
        self._pan_angle += self._pan_direction * self.config.search_speed
        limit = self.config.search_pan_limit

        # Clamp to limit and reverse direction for the next step
        if self._pan_angle >= limit:
            self._pan_angle = limit
            self._pan_direction = -1
        elif self._pan_angle <= -limit:
            self._pan_angle = -limit
            self._pan_direction = 1

        self.hardware.pan_motor.setPosition(self._pan_angle)
        # Keep the unwrap reference current so the first TRACK aim unwraps
        # relative to where the sweep actually left the pan motor.
        self._last_pan_cmd = self._pan_angle
        _motor_log.debug(
            "[SEARCH sweep] pan_motor.setPosition(%.4f rad)  dir=%+d  limit=%.4f",
            self._pan_angle,
            self._pan_direction,
            limit,
        )

        # --- detection counting ---
        detections = self.sensors.search_radar.get_detections()
        if detections:
            self._detection_count += 1
        else:
            self._detection_count = 0

        if self._detection_count >= self.config.acquire_frames:
            self._target = detections[0].track_id
            self._transition(self.ACQUIRE)

    def _do_acquire(self) -> None:
        """Execute one timestep of ACQUIRE state logic.

        Entry (first call only): locks the SearchRadar onto the selected
        target via ``search_radar.set_target(track_id)`` and resets the
        TrackFilter. These entry actions fire exactly once, guarded by
        ``_acquire_entry_done``.

        FCR is NOT re-targeted here. Per ADR-0006 the Fire-Control Radar is
        single-target and cued at construction; the FSM does not call
        ``fcr.set_target``. Only the SearchRadar (wide-beam) is locked by the
        FSM at ACQUIRE.

        Each step: checks ``track_filter.is_initialised()``. When True,
        transitions to TRACK.

        ``self._target`` holds an integer ``track_id`` (ADR-0006) — never a
        Webots node handle.

        See ADR-0006 for the sensor-membrane and track-id identity contract.
        """
        if not self._acquire_entry_done:
            self.sensors.search_radar.set_target(self._target)
            self.sensors.track_filter.reset()
            self._acquire_entry_done = True

        if self.sensors.track_filter.is_initialised():
            self._transition(self.TRACK)

    def _do_track(self) -> None:
        """Execute one timestep of TRACK state logic.

        Reads the current filtered target position from the TrackFilter, commands
        both motors to aim at it, and increments the frame counter. When the
        counter reaches config.min_track_frames the FSM transitions to PREDICT.

        Position is relative [dx, dy, dz] from the turret origin (metres, Z-up ENU).
        Motor angles are computed via _compute_aim_angles().
        """
        position = self.sensors.track_filter.get_position()
        self._aim_at(position)

        self._track_frames += 1
        if self._track_frames >= self.config.min_track_frames:
            self._transition(self.PREDICT)

    def _do_predict(self) -> None:
        """Execute one timestep of PREDICT state logic.

        Computes a ballistic intercept point for config.lookahead_steps ahead
        and validates it. If the intercept is within max_range AND above
        ground_threshold, stores it on self._intercept and transitions to AIMING.
        If the intercept is invalid (out of range or below ground), transitions
        back to TRACK to continue refining the estimate before retrying.

        The intercept is stored as relative [dx, dy, dz] (metres, Z-up ENU)
        from the turret origin, ready for AIMING to consume.
        """
        intercept = self.sensors.ballistic_predictor.get_intercept(
            self.config.lookahead_steps
        )
        if self._target_in_range(intercept):
            self._intercept = intercept
            self._transition(self.AIMING)
        else:
            self._transition(self.TRACK)

    def _do_aim(self) -> None:
        """Execute one timestep of AIMING state logic.

        Slews both motors toward the stored intercept point (self._intercept, relative
        [dx, dy, dz] from the turret origin, set by PREDICT). The aim error is measured
        as the Euclidean angular distance between the desired angles (from the intercept)
        and the angles most-recently commanded to the motors (_commanded_pan,
        _commanded_tilt). The error is evaluated BEFORE issuing new commands so that
        the first AIMING step — where the motors are still at whatever TRACK left them —
        has a nonzero error and the FSM stays in AIMING. After the motors are commanded
        the stored commanded angles are updated; on the next step the error is zero
        (desired == newly commanded), which is below any positive threshold, and the FSM
        transitions to ENGAGING.

        Aim-error design rationale
        --------------------------
        Webots RotationalMotor has no position readback without an attached
        PositionSensor. Adding a sensor purely for AIMING would leak hardware
        concerns into tests and complicate the controller. Instead, the FSM tracks
        the last angle it sent to each motor (_commanded_pan, _commanded_tilt). This
        is an internally-consistent measure of "how far the turret still needs to
        slew" — it is zero the step after the desired angles are first commanded,
        which is the earliest the motors could realistically be pointing at the
        target. This is a deliberate simplification: the real mechanical slew
        latency is ignored, which is acceptable for the scope of Task 8. A concern
        is noted: if the simulation timestep is large relative to the motor slew
        speed, this may fire ENGAGING prematurely. Consider adding a configurable
        dwell count in a future task if needed.

        Transitions to ENGAGING when aim error < config.aim_error_threshold (radians).

        Precondition: self._intercept is not None (set by PREDICT before entering AIMING).
        """
        desired_pan, desired_tilt = self._compute_aim_angles(self._intercept)

        # Measure error against PREVIOUS commanded angles (before this step's command)
        error = math.sqrt(
            (desired_pan - self._commanded_pan) ** 2
            + (desired_tilt - self._commanded_tilt) ** 2
        )

        # Command motors and record what we sent
        self._aim_at(self._intercept)
        self._commanded_pan = desired_pan
        self._commanded_tilt = desired_tilt

        if error < self.config.aim_error_threshold:
            self._transition(self.ENGAGING)

    def _do_engage(self) -> None:
        """Execute one timestep of ENGAGING state logic.

        Activates the laser (self.laser_active = True) and holds aim on the stored
        intercept point by re-commanding both motors each step. Reads the current
        target position from sensors.track_filter.get_position() and checks whether
        the target is still within engagement range via _target_in_range(). When the
        target goes out of range (beyond max_range or at/below ground_threshold),
        transitions to RESET.

        self.laser_active is readable by the controller (Task 9) to drive the
        actual laser hardware.

        Precondition: self._intercept is not None (set by PREDICT, unchanged since AIMING).
        """
        self.laser_active = True

        # Hold aim on the fixed intercept
        self._aim_at(self._intercept)

        # Monitor live target position; exit when target leaves the engagement envelope
        target_position = self.sensors.track_filter.get_position()
        if not self._target_in_range(target_position):
            self._transition(self.RESET)

    def _do_reset(self) -> None:
        """Execute one timestep of RESET state logic.

        Clears all engagement and tracking bookkeeping accumulated since SEARCH,
        then transitions to SEARCH so the turret begins a fresh scan cycle.

        What this handler clears explicitly (not covered by _transition(SEARCH)):
          - laser_active       → False  (engagement flag)
          - _target            → None   (selected detection node)
          - _intercept         → None   (PREDICT output; normally cleared on TRACK
                                         entry by _transition, but cleared here too
                                         for clarity since RESET always bypasses TRACK)
          - _track_frames      → 0      (_transition(SEARCH) does not reset this;
                                         it is reset by _transition(TRACK), which is
                                         not visited during a RESET→SEARCH path)
          - _acquire_entry_done → False (_transition(ACQUIRE) would reset this, but
                                         RESET→SEARCH skips ACQUIRE, so the guard must
                                         be explicitly cleared here for the next cycle)

        _transition(SEARCH) covers: _detection_count, _pan_direction, _pan_angle.
        No logic is duplicated between this handler and _transition().
        """
        self.laser_active = False
        self._target = None
        self._intercept = None
        self._track_frames = 0
        self._acquire_entry_done = False
        self._transition(self.SEARCH)

    # ---------------------------------------------------------------- helpers

    def _aim_at(self, rel_position: list[float]) -> tuple[float, float]:
        """Command both motors to point at a relative position and return the angles.

        Computes pan/tilt angles via _compute_aim_angles() and issues setPosition()
        to both motors in one atomic call, ensuring neither motor is ever commanded
        without the other.

        Args:
            rel_position: Relative [dx, dy, dz] from turret origin (metres, Z-up ENU).

        Returns:
            (pan_angle, tilt_angle) in radians — the angles sent to the motors this step.
        """
        pan, tilt = self._compute_aim_angles(rel_position)

        # pan from atan2 is in [-pi, pi] and is DISCONTINUOUS across the +/-pi
        # seam: a target behind the turret makes pan flip between +pi and -pi as
        # dx jitters around zero. Both are the same heading, but commanded as
        # absolute motor positions they are ~2*pi apart, so the motor whips a
        # near-full revolution every step (the "twitchy" turret). Unwrapping the
        # raw angle to the equivalent nearest the last command keeps every
        # commanded step <= pi, so the motor always takes the short way round.
        raw_pan = pan
        pan = self._unwrap_pan(raw_pan, self._last_pan_cmd)
        self._last_pan_cmd = pan

        self.hardware.pan_motor.setPosition(pan)
        self.hardware.tilt_motor.setPosition(tilt)
        _motor_log.debug(
            "[%s aim] target_rel=[%.3f, %.3f, %.3f] -> "
            "pan_motor.setPosition(%.4f rad)  tilt_motor.setPosition(%.4f rad)  "
            "(raw_pan=%.4f, unwrapped by %+.4f)",
            self.state,
            rel_position[0],
            rel_position[1],
            rel_position[2],
            pan,
            tilt,
            raw_pan,
            pan - raw_pan,
        )
        return pan, tilt

    @staticmethod
    def _unwrap_pan(new_angle: float, prev_angle: float) -> float:
        """Return the rotation equivalent to new_angle that is nearest prev_angle.

        Shifts new_angle by whole turns so it lands within +/-pi of prev_angle.
        This removes the +/-pi discontinuity of atan2 (see _aim_at): without it
        the pan motor is told to spin ~360 degrees back and forth whenever the
        target sits roughly behind the turret, producing visible twitching.

        math.remainder(x, tau) reduces x into [-pi, pi], so applying it to the
        delta gives the shortest signed step from prev_angle to new_angle; adding
        that back to prev_angle yields the unwrapped (continuous) command.

        Note: pan may wind past +/-pi over many laps. That is intentional and
        safe here because PAN_MOTOR has no minPosition/maxPosition limits set.

        Args:
            new_angle:  Freshly computed pan angle from atan2 (radians, [-pi, pi]).
            prev_angle: Previous pan angle actually commanded to the motor (radians).

        Returns:
            new_angle adjusted by a whole number of turns to be within +/-pi
            of prev_angle.
        """
        return prev_angle + math.remainder(new_angle - prev_angle, math.tau)

    def _compute_aim_angles(self, rel_target: list[float]) -> tuple[float, float]:
        """Convert relative Cartesian position to pan/tilt motor angles.

        Z-up ENU convention:
            pan  = atan2(dx, dy)
            tilt = atan2(dz, sqrt(dx² + dy²))

        Args:
            rel_target: Relative [dx, dy, dz] from turret origin (metres).

        Returns:
            (pan_angle, tilt_angle) in radians.
        """
        dx, dy, dz = rel_target
        pan = math.atan2(dx, dy)
        tilt = math.atan2(dz, math.sqrt(dx * dx + dy * dy))
        return pan, tilt

    def _target_in_range(self, rel_position: list[float]) -> bool:
        """Return True if the target is within range and above the ground threshold.

        A target is considered valid when:
        - Its Euclidean distance from the turret is ≤ config.max_range (metres).
        - Its world-Z component (rel_position[2]) is > config.ground_threshold
          (metres), meaning it has not hit the ground.

        Args:
            rel_position: Relative [dx, dy, dz] from turret origin (metres).

        Returns:
            True if within max_range AND above ground_threshold, False otherwise.
        """
        dx, dy, dz = rel_position
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        return distance <= self.config.max_range and dz > self.config.ground_threshold

    def _transition(self, new_state: str) -> None:
        """Log and execute a state transition, resetting per-state bookkeeping.

        Args:
            new_state: One of the AtlasFSM state constants (SEARCH, ACQUIRE, …).
        """
        _log.info("%s -> %s", self.state, new_state)
        self.state = new_state

        # Reset bookkeeping for the state we are ENTERING
        if new_state == self.SEARCH:
            self._detection_count = 0
            self._pan_direction = 1
            self._pan_angle = 0.0
        elif new_state == self.ACQUIRE:
            self._acquire_entry_done = False
        elif new_state == self.TRACK:
            self._track_frames = 0
            self._intercept = None
