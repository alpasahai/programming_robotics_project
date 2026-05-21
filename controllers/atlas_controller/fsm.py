"""Atlas FSM — 5-state finite state machine governing turret behaviour."""

import logging
import math
from collections import deque
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

    cue_link: object  # SearchRadarLink — world-frame cues over the radio link
    fcr: object  # FireControlRadar
    track_filter: object  # TrackFilter
    ballistic_predictor: object  # BallisticTrajectoryPredictor
    ground_hit_link: object  # AttackerGroundHitLink — attacker ground-hit pulse


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
    to speed up state transitions (e.g. acquire_frames=1) or adjust thresholds.
    """

    max_range: float = 10.0  # metres — target/intercept beyond this → RESET
    ground_threshold: float = 0.1  # metres world-Z — telemetry readout only; not used by FSM logic
    acquire_frames: int = 3  # consecutive cues to leave IDLE
    # IDLE beam direction (radians, Z-up ENU). The turret holds this fixed aim
    # while waiting for a Search Radar cue — no pan sweep. Defaults to straight
    # ahead and level.
    idle_pan: float = 0.0
    idle_tilt: float = 0.0
    # TRACK_PREDICT → ENGAGING gate. Each step compares the intercept predicted
    # lookahead_steps ago FOR the current step against the filter's current
    # position estimate. When that prediction error stays below
    # track_error_threshold for converge_frames consecutive steps, the
    # prediction is trustworthy enough to fire. See _do_track_predict.
    track_error_threshold: float = 0.3  # metres
    converge_frames: int = 3  # consecutive sub-threshold steps before firing
    # Estimated turret slew rate (rad/s). The turret has no motor PositionSensor,
    # so the FSM models the physical aim: each step it advances its reported aim
    # (commanded_aim) toward the commanded target by at most slew_rate * dt. The
    # FCR gates its lock on commanded_aim, so AIM only advances to TRACK_PREDICT
    # once the turret has actually had time to slew onto the cued bearing —
    # rather than the instant the angle is commanded. MUST match the PAN/TILT
    # motor maxVelocity in AtlasTurret.proto for the estimate to track the real
    # motor.
    slew_rate: float = 10.0  # rad/s
    # lookahead_steps tuned for the ATLAS projectile (~1.6 m apex, ~1.2 s flight):
    # at a 32 ms timestep, 10 steps = 0.32 s, keeping the predicted intercept
    # airborne. A longer lookahead overshoots the projectile's landing and the
    # intercept falls below ground_threshold — see ADR-0001.
    lookahead_steps: int = 10  # timesteps ahead for intercept (see note above)


class AtlasFSM:
    """5-state FSM governing all ATLAS turret behaviour.

    States
    ------
    IDLE          : Turret holds a fixed beam direction (idle_pan, idle_tilt) and
                    waits for Search Radar cues over the radio link. No pan sweep.
    AIM           : Cue received. Turret slews toward the cued world position and
                    the TrackFilter is reset. Aims off the raw cue only (no fused
                    estimate yet). Waits for the FCR to lock onto the target.
    TRACK_PREDICT : TrackFilter fuses both sensors — the FCR continuously (main
                    loop) and the Search Radar cue here. Turret follows the filtered
                    position while the BallisticTrajectoryPredictor computes the
                    intercept. Fires when the predicted-vs-observed error converges.
    ENGAGING      : Holds aim at the fired intercept. No more predictions.
                    Transitions to RESET when the attacker's ground-hit cue fires
                    or the target leaves max_range.
    RESET         : Wipes the Kalman filter and all bookkeeping. Returns to IDLE.

    Readiness to fire is the ENGAGING state itself — there is no separate flag.
    Consumers check ``fsm.state == AtlasFSM.ENGAGING``.

    Sensor fusion
    -------------
    The FCR feeds the TrackFilter every timestep (continuous fusion, main loop).
    The Search Radar runs as a separate process and reaches the FSM only as a
    world-frame cue over the radio link; the FSM fuses it into the filter during
    TRACK_PREDICT (dual-sensor fusion). See ADR-0003.

    Coordinate system: Z-up ENU.
        pan  = atan2(dx, dy)
        tilt = atan2(dz, sqrt(dx² + dy²))
    """

    IDLE = "IDLE"
    AIM = "AIM"
    TRACK_PREDICT = "TRACK_PREDICT"
    ENGAGING = "ENGAGING"
    RESET = "RESET"

    def __init__(
        self,
        sensors: SensorSuite,
        hardware: TurretHardware,
        config: FSMConfig | None = None,
    ) -> None:
        """Initialise the FSM in IDLE state with all per-state bookkeeping.

        Args:
            sensors:  All sensing/processing components. See SensorSuite.
            hardware: Turret motors, world position, and timestep. See TurretHardware.
            config:   Tunable FSM parameters. Defaults to FSMConfig() if not provided.
        """
        self.sensors = sensors
        self.hardware = hardware
        self.config = config if config is not None else FSMConfig()

        self.state = self.IDLE

        # IDLE bookkeeping
        self._detection_count = 0  # consecutive steps with a non-None cue from the cue link

        # Last angle actually commanded to the pan motor (radians, unwrapped).
        # Needed because pan = atan2(dx, dy) is discontinuous: when the target
        # is roughly behind the turret it snaps between +pi and -pi as dx jitters
        # around zero. Those are the same heading, but feeding them to the motor
        # as absolute positions ~2*pi apart makes it whip a near-full turn each
        # step. _aim_at() unwraps each new pan command relative to this value so
        # the motor always takes the short way round. See _unwrap_pan().
        self._last_pan_cmd = 0.0

        # AIM bookkeeping
        self._target = None  # cued target — world-frame position [x, y, z]
        self._aim_entry_done = False  # guard: entry actions fire exactly once

        # TRACK_PREDICT bookkeeping
        self._intercept = None  # validated intercept point, set at the ENGAGING transition
        # Rolling history of recent predicted intercepts. When full,
        # _pred_history[0] is the intercept predicted lookahead_steps ago FOR the
        # current step, so it can be compared against the current estimate.
        self._pred_history = deque(maxlen=self.config.lookahead_steps + 1)
        self._converge_count = 0  # consecutive sub-threshold prediction-error steps

        # Aim bookkeeping. These hold the FSM's *estimate* of the turret's
        # physical aim (radians), not the absolute target last commanded to the
        # motors. _command_angles advances them toward each commanded target at
        # config.slew_rate, modelling the motor's slew in the absence of a
        # PositionSensor. commanded_aim exposes them so the FCR gates its lock on
        # the estimated physical aim. The absolute target (with the pan seam
        # unwrapped) is tracked separately by _last_pan_cmd.
        self._commanded_pan = 0.0
        self._commanded_tilt = 0.0

    def step(self) -> None:
        """Advance FSM by one timestep.

        Executes the active state's logic and commands motors. Call once per
        simulation step, after cue_link.update(), fcr.update(),
        track_filter.predict(), and track_filter.update_fcr() have all been
        called in the main loop.
        """
        if self.state == self.IDLE:
            self._do_idle()
        elif self.state == self.AIM:
            self._do_aim()
        elif self.state == self.TRACK_PREDICT:
            self._do_track_predict()
        elif self.state == self.ENGAGING:
            self._do_engage()
        elif self.state == self.RESET:
            self._do_reset()

    @property
    def commanded_aim(self) -> tuple[float, float]:
        """Return the FSM's estimate of the turret's physical aim as ``(pan, tilt)``.

        Both angles are in radians, Z-up ENU (pan = azimuth, tilt = elevation).
        This is NOT the absolute target last sent to the motors — it is an
        estimate that slews toward each commanded target at ``config.slew_rate``,
        modelling the motor's real motion since the turret has no PositionSensor.
        Before any command is issued it is ``(0.0, 0.0)``.

        The controller reads this each sense phase to feed the FCR its
        boresight: ``fcr.update(*fsm.commanded_aim)``. Because it tracks the
        physical slew rather than the instantaneous target, the FCR only locks
        once the turret has actually swung onto the target — which is what keeps
        AIM from advancing to TRACK_PREDICT prematurely.
        """
        return (self._commanded_pan, self._commanded_tilt)

    # --------------------------------------------------------- state handlers

    def _do_idle(self) -> None:
        """Execute one timestep of IDLE state logic.

        Holds the turret at the fixed (idle_pan, idle_tilt) beam direction —
        there is no pan sweep. Simultaneously reads the latest Search Radar cue
        from the cue link and counts consecutive steps that carry a non-None
        cue. Any step with no cue resets the counter to zero. When the count
        reaches acquire_frames the cue (a world-frame [x, y, z] position) is
        stored as self._target and the FSM transitions to AIM.

        A ground-hit cue takes precedence and sends the FSM to RESET.
        """
        if self.sensors.ground_hit_link.hit_this_step():
            self._transition(self.RESET)
            return

        # Hold the fixed idle direction.
        self._command_angles(self.config.idle_pan, self.config.idle_tilt)

        cue = self.sensors.cue_link.get_cue()
        if cue is not None:
            self._detection_count += 1
        else:
            self._detection_count = 0

        if self._detection_count >= self.config.acquire_frames:
            # Store the world-frame cue; _world_to_relative converts it to
            # turret-relative each AIM step.
            self._target = cue
            self._transition(self.AIM)

    def _do_aim(self) -> None:
        """Execute one timestep of AIM state logic.

        Entry (first call only): resets the TrackFilter so the new engagement
        starts from a clean estimate. Guarded by ``_aim_entry_done``.

        Each step: reads the latest Search Radar cue, converts the world-frame
        cue to a turret-relative bearing, and slews the motors toward it via
        ``_aim_at`` — aiming off the raw cue only, since the fused estimate is
        not yet trustworthy. If the cue link is briefly silent the FSM holds its
        last aim and does not error.

        Transitions to TRACK_PREDICT when ``fcr.is_locked()`` — the FCR has the
        target inside its narrow FOV cone (the design's "FCR beam finds the
        target"). A ground-hit cue takes precedence and sends the FSM to RESET.
        """
        if self.sensors.ground_hit_link.hit_this_step():
            self._transition(self.RESET)
            return

        if not self._aim_entry_done:
            self.sensors.track_filter.reset()
            self._aim_entry_done = True

        cue = self.sensors.cue_link.get_cue()
        if cue is not None:
            self._aim_at(self._world_to_relative(cue))

        if self.sensors.fcr.is_locked():
            self._transition(self.TRACK_PREDICT)

    def _do_track_predict(self) -> None:
        """Execute one timestep of TRACK_PREDICT state logic.

        Dual-sensor fusion + prediction, merging the old TRACK, PREDICT and
        AIMING states:

          1. Fuse the latest Search Radar cue into the TrackFilter
             (``update_search``). The FCR is fused continuously by the main
             loop; fusing the Search Radar here is what makes this genuine
             dual-sensor fusion (see ADR-0003).
          2. Aim the turret at the current filtered position.
          3. Compute the ballistic intercept lookahead_steps ahead and push it
             onto a rolling history. Once the history is full, _pred_history[0]
             is the intercept predicted lookahead_steps ago FOR the current
             step; the prediction error is its distance from the current
             filtered position. When that error stays below
             track_error_threshold for converge_frames consecutive steps AND the
             intercept is within max_range, the prediction is trustworthy: store
             it on self._intercept and transition to ENGAGING.

        A ground-hit cue takes precedence and sends the FSM to RESET.
        """
        if self.sensors.ground_hit_link.hit_this_step():
            self._transition(self.RESET)
            return

        # 1. Fuse the Search Radar cue (FCR is fused continuously by the loop).
        cue = self.sensors.cue_link.get_cue()
        if cue is not None:
            self.sensors.track_filter.update_search(self._world_to_relative(cue))

        # 2. Aim at the current filtered estimate.
        position = self.sensors.track_filter.get_position()
        self._aim_at(position)

        # 3. Predict and test convergence of predicted-vs-observed error.
        intercept = self.sensors.ballistic_predictor.get_intercept(
            self.config.lookahead_steps
        )
        pred_error = self._record_intercept_and_error(intercept, position)

        if (
            pred_error is not None
            and pred_error < self.config.track_error_threshold
            and self._target_within_range(intercept)
        ):
            self._converge_count += 1
        else:
            self._converge_count = 0

        if self._converge_count >= self.config.converge_frames:
            self._intercept = intercept
            self._transition(self.ENGAGING)

    def _do_engage(self) -> None:
        """Execute one timestep of ENGAGING state logic.

        Holds aim on the fixed intercept (self._intercept, set at the
        TRACK_PREDICT → ENGAGING transition) by re-commanding both motors each
        step. No further predictions happen here. Entering ENGAGING is itself
        the "ready to fire" signal — there is no separate flag.

        Transitions to RESET when the attacker's ground-hit cue fires this step
        (sensors.ground_hit_link) or the target leaves max_range. The ground hit
        is never inferred from the track Z — the emitted cue is the only
        ground-hit signal.

        Precondition: self._intercept is not None (set by TRACK_PREDICT).
        """
        # Hold aim on the fixed intercept.
        self._aim_at(self._intercept)

        # Exit on the authoritative ground-hit cue from the attacker, or when the
        # target leaves the range envelope. Ground hits are NEVER inferred from
        # the track Z here — the emitted cue is the only ground-hit signal.
        # TODO(projectile-destroyed-cue): when the turret-weapon / bullet-hit cue
        # is built, add a "projectile destroyed → RESET" exit here.
        target_position = self.sensors.track_filter.get_position()
        if self.sensors.ground_hit_link.hit_this_step() or not self._target_within_range(
            target_position
        ):
            self._transition(self.RESET)

    def _do_reset(self) -> None:
        """Execute one timestep of RESET state logic.

        Wipes the Kalman filter memory and all engagement/tracking bookkeeping
        accumulated since IDLE, then transitions to IDLE so the turret begins a
        fresh cycle.

        What this handler clears explicitly (beyond _transition(IDLE)):
          - track_filter.reset()  → wipes the Kalman estimate (per the redesign)
          - cue_link.clear()      → discards the previous engagement's Search Radar
                                    cue. The cue link has no expiry, so without this
                                    the stale cue would immediately re-trigger
                                    IDLE → AIM even though no fresh cue has arrived.
          - _target               → None
          - _intercept            → None
          - _aim_entry_done       → False (RESET→IDLE skips AIM, so the entry
                                    guard must be cleared here for the next cycle)
          - _converge_count       → 0
          - _pred_history         → cleared

        _transition(IDLE) covers: _detection_count.
        """
        self.sensors.track_filter.reset()
        self.sensors.cue_link.clear()
        self._target = None
        self._intercept = None
        self._aim_entry_done = False
        self._converge_count = 0
        self._pred_history.clear()
        self._transition(self.IDLE)

    # ---------------------------------------------------------------- helpers

    def _record_intercept_and_error(self, intercept, current_position):
        """Append this step's intercept and return the prediction error for the
        intercept that targeted *this* step, or None while warming up.

        Pure bookkeeping over the rolling history. When the history is full,
        _pred_history[0] is the intercept predicted lookahead_steps ago FOR the
        current step; its distance from current_position is how accurate that
        prediction turned out to be. Mirrors
        telemetry.TrackTelemetry.record_intercept_and_error.

        Args:
            intercept:        This step's predicted intercept [dx, dy, dz].
            current_position: The current filtered position [dx, dy, dz].

        Returns:
            Euclidean prediction error (metres), or None if not enough history.
        """
        full = len(self._pred_history) == self._pred_history.maxlen
        oldest = self._pred_history[0] if full else None
        self._pred_history.append(intercept)
        if oldest is not None:
            return math.dist(oldest, current_position)
        return None

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
        return self._command_angles(pan, tilt, rel_position=rel_position)

    def _command_angles(
        self, pan: float, tilt: float, rel_position=None
    ) -> tuple[float, float]:
        """Issue pan/tilt setPosition() commands, unwrapping the pan seam.

        pan from atan2 is in [-pi, pi] and is DISCONTINUOUS across the +/-pi
        seam: a target behind the turret makes pan flip between +pi and -pi as
        dx jitters around zero. Both are the same heading, but commanded as
        absolute motor positions they are ~2*pi apart, so the motor whips a
        near-full revolution every step (the "twitchy" turret). Unwrapping the
        raw angle to the equivalent nearest the last command keeps every
        commanded step <= pi, so the motor always takes the short way round.

        Args:
            pan:          Desired pan angle (radians, may be the raw atan2 value).
            tilt:         Desired tilt angle (radians).
            rel_position: Optional [dx, dy, dz] for the debug log line.

        Returns:
            (pan_angle, tilt_angle) actually commanded, in radians.
        """
        raw_pan = pan
        target_pan = self._unwrap_pan(raw_pan, self._last_pan_cmd)
        self._last_pan_cmd = target_pan

        # Tell the motors where to end up. Webots slews them there at their
        # maxVelocity over several steps — we command the absolute target, not a
        # rate-limited step.
        self.hardware.pan_motor.setPosition(target_pan)
        self.hardware.tilt_motor.setPosition(tilt)

        # Model the physical aim. With no PositionSensor we cannot read the real
        # motor angle, so advance our estimate toward the commanded target at the
        # turret's slew rate. commanded_aim exposes this estimate; the FCR gates
        # its lock on it, so the FSM only believes it is on-target once the turret
        # has actually had time to slew there. slew_rate must match the motor
        # maxVelocity for the estimate to stay in step with the real motor.
        max_step = self.config.slew_rate * (self.hardware.timestep_ms / 1000.0)
        self._commanded_pan = self._step_toward(self._commanded_pan, target_pan, max_step)
        self._commanded_tilt = self._step_toward(self._commanded_tilt, tilt, max_step)
        _motor_log.debug(
            "[%s aim] target_rel=%s -> "
            "pan_motor.setPosition(%.4f rad)  tilt_motor.setPosition(%.4f rad)  "
            "(raw_pan=%.4f)  est_aim=(%.4f, %.4f)",
            self.state,
            rel_position,
            target_pan,
            tilt,
            raw_pan,
            self._commanded_pan,
            self._commanded_tilt,
        )
        return target_pan, tilt

    @staticmethod
    def _step_toward(current: float, target: float, max_step: float) -> float:
        """Return ``current`` moved toward ``target`` by at most ``max_step``.

        Snaps to ``target`` once it is within reach. Used to advance the
        estimated physical aim at the turret's slew rate (see _command_angles).
        """
        delta = target - current
        if abs(delta) <= max_step:
            return target
        return current + math.copysign(max_step, delta)

    @staticmethod
    def _unwrap_pan(new_angle: float, prev_angle: float) -> float:
        """Return the rotation equivalent to new_angle that is nearest prev_angle.

        Shifts new_angle by whole turns so it lands within +/-pi of prev_angle.
        This removes the +/-pi discontinuity of atan2 (see _command_angles):
        without it the pan motor is told to spin ~360 degrees back and forth
        whenever the target sits roughly behind the turret, producing visible
        twitching.

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

    def _world_to_relative(self, world_position: list[float]) -> list[float]:
        """Convert a world-frame position to a turret-relative vector.

        Subtracts the turret's world origin (``hardware.turret_position``) from
        the given world position, producing a relative ``[dx, dy, dz]`` ready
        for ``_aim_at`` / ``_compute_aim_angles``.

        Args:
            world_position: World-frame [x, y, z] in metres (ENU, Z-up) — e.g.
                a Search Radar cue.

        Returns:
            Relative [dx, dy, dz] from the turret origin (metres, Z-up ENU).
        """
        turret = self.hardware.turret_position
        return [world_position[i] - turret[i] for i in range(3)]

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

    def _target_within_range(self, rel_position: list[float]) -> bool:
        """Return True if the target/intercept is within engagement range.

        In range means the Euclidean distance from the turret is
        ≤ config.max_range (metres). Ground reasoning deliberately lives
        nowhere in the FSM: a ground hit is signalled only by the attacker's
        emitted cue (sensors.ground_hit_link), never inferred from Z.

        Args:
            rel_position: Relative [dx, dy, dz] from turret origin (metres).

        Returns:
            True if within max_range, False otherwise.
        """
        dx, dy, dz = rel_position
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        return distance <= self.config.max_range

    def _transition(self, new_state: str) -> None:
        """Log and execute a state transition, resetting per-state bookkeeping.

        Args:
            new_state: One of the AtlasFSM state constants (IDLE, AIM, …).
        """
        _log.info("%s -> %s", self.state, new_state)
        self.state = new_state

        # Reset bookkeeping for the state we are ENTERING
        if new_state == self.IDLE:
            self._detection_count = 0
        elif new_state == self.AIM:
            self._aim_entry_done = False
        elif new_state == self.TRACK_PREDICT:
            self._intercept = None
            self._converge_count = 0
            self._pred_history.clear()
