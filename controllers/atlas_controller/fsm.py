"""Atlas FSM — 7-state finite state machine governing turret behaviour."""
from __future__ import annotations
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
        """
        Args:
            sensors:  All sensing/processing components. See SensorSuite.
            hardware: Turret motors, world position, and timestep. See TurretHardware.
            config:   Tunable FSM parameters. Defaults to FSMConfig() if not provided.
        """
        raise NotImplementedError

    def step(self) -> None:
        """Advance FSM by one timestep.

        Executes the active state's logic and commands motors. Call once per
        simulation step, after search_radar.update(), fcr.update(),
        track_filter.predict(), track_filter.update_fcr(), and
        track_filter.update_search() have all been called in the main loop.
        """
        raise NotImplementedError

    # ---------------------------------------------------------------- helpers

    def _compute_aim_angles(self, rel_target: list[float]) -> tuple[float, float]:
        """Convert relative Cartesian position to pan/tilt motor angles.

        Z-up ENU:
            pan  = atan2(dx, dy)
            tilt = atan2(dz, sqrt(dx² + dy²))

        Args:
            rel_target: Relative [dx, dy, dz] from turret.

        Returns:
            (pan_angle, tilt_angle) in radians.
        """
        raise NotImplementedError

    def _target_in_range(self, rel_position: list[float]) -> bool:
        """Return True if position is within MAX_RANGE and above GROUND_THRESHOLD."""
        raise NotImplementedError

    def _transition(self, new_state: str) -> None:
        """Log and execute a state transition."""
        raise NotImplementedError
