"""Atlas FSM — 7-state finite state machine governing turret behaviour."""
import math
from dataclasses import dataclass


@dataclass
class SensorSuite:
    """All sensing and processing components (Sense + Think layers).

    Injected into AtlasFSM at construction. Swap out in tests by substituting
    stub implementations for each field.
    """
    search_radar:        object  # SearchRadar
    fcr:                 object  # FireControlRadar
    track_filter:        object  # TrackFilter
    ballistic_predictor: object  # BallisticTrajectoryPredictor


@dataclass
class TurretHardware:
    """All actuation components and physical configuration (Act layer).

    Injected into AtlasFSM at construction.
    """
    pan_motor:       object        # Webots RotationalMotor
    tilt_motor:      object        # Webots RotationalMotor
    turret_position: list[float]   # World-frame [x, y, z] of turret origin
    timestep_ms:     int


@dataclass
class FSMConfig:
    """Tunable parameters for AtlasFSM behaviour.

    All fields have sensible defaults for normal operation. Override in tests
    to speed up state transitions (e.g. ACQUIRE_FRAMES=1) or adjust thresholds.
    """
    max_range:           float = 10.0   # metres — target beyond this → RESET
    ground_threshold:    float = 0.1    # metres world-Z — below this → landed
    acquire_frames:      int   = 3      # consecutive detections to leave SEARCH
    min_track_frames:    int   = 15     # TrackFilter frames before PREDICT
    aim_error_threshold: float = 0.05   # radians — convergence for ENGAGING
    search_speed:        float = 0.02   # radians per step during pan sweep
    search_pan_limit:    float = 1.4    # radians (~80°) sweep extent
    lookahead_steps:     int   = 20     # timesteps ahead for intercept


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

    SEARCH   = "SEARCH"
    ACQUIRE  = "ACQUIRE"
    TRACK    = "TRACK"
    PREDICT  = "PREDICT"
    AIMING   = "AIMING"
    ENGAGING = "ENGAGING"
    RESET    = "RESET"

    def __init__(
        self,
        sensors: SensorSuite,
        hardware: TurretHardware,
        config: FSMConfig = None,
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
        self._detection_count = 0       # consecutive steps with at least one detection
        self._pan_angle = 0.0           # current pan motor angle (radians)
        self._pan_direction = 1         # +1 sweeping positive, -1 sweeping negative

        # ACQUIRE bookkeeping
        self._target = None             # selected target (detection position vector)
        self._acquire_entry_done = False  # guard: entry actions fire exactly once

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

        # --- detection counting ---
        detections = self.sensors.search_radar.get_detections()
        if detections:
            self._detection_count += 1
        else:
            self._detection_count = 0

        if self._detection_count >= self.config.acquire_frames:
            self._target = detections[0]
            self._transition(self.ACQUIRE)
            self._do_acquire()

    def _do_acquire(self) -> None:
        """Execute one timestep of ACQUIRE state logic.

        Entry (first call only): calls set_target() on both FCR and
        SearchRadar with the selected target, and resets the TrackFilter.
        These entry actions fire exactly once, guarded by _acquire_entry_done.

        Each step: checks track_filter.is_initialised(). When True, transitions
        to TRACK.

        DESIGN NOTE — node vs position mismatch: The selected target stored in
        self._target is a detection position vector ([dx, dy, dz]), not a Webots
        node handle. Both set_target() interfaces expect a node handle. This
        mismatch is benign for Task 6 because the stub set_target() is a no-op
        accepting any argument, but MUST be resolved when wiring the real
        controller in Task 9. See the Task 6 report for details.
        """
        if not self._acquire_entry_done:
            self.sensors.fcr.set_target(self._target)
            self.sensors.search_radar.set_target(self._target)
            self.sensors.track_filter.reset()
            self._acquire_entry_done = True

        if self.sensors.track_filter.is_initialised():
            self._transition(self.TRACK)

    def _do_track(self) -> None:
        """Execute one timestep of TRACK state logic. (Task 7 — not yet implemented.)"""
        raise NotImplementedError

    def _do_predict(self) -> None:
        """Execute one timestep of PREDICT state logic. (Task 7 — not yet implemented.)"""
        raise NotImplementedError

    def _do_aim(self) -> None:
        """Execute one timestep of AIMING state logic. (Task 8 — not yet implemented.)"""
        raise NotImplementedError

    def _do_engage(self) -> None:
        """Execute one timestep of ENGAGING state logic. (Task 8 — not yet implemented.)"""
        raise NotImplementedError

    def _do_reset(self) -> None:
        """Execute one timestep of RESET state logic. (Task 8 — not yet implemented.)"""
        raise NotImplementedError

    # ---------------------------------------------------------------- helpers

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
        print(f"[AtlasFSM] {self.state} → {new_state}")
        self.state = new_state

        # Reset bookkeeping for the state we are ENTERING
        if new_state == self.SEARCH:
            self._detection_count = 0
            self._pan_direction = 1
            self._pan_angle = 0.0
        elif new_state == self.ACQUIRE:
            self._acquire_entry_done = False
