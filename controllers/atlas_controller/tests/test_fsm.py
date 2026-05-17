"""Tests for AtlasFSM — SEARCH and ACQUIRE states (Tasks 6).

Covers:
- Initial state is SEARCH
- _do_search moves pan motor each step
- Pan motor reverses direction at the limit
- Consecutive detections count up; transition to ACQUIRE after acquire_frames
- A no-detection frame resets the consecutive counter
- ACQUIRE calls set_target on both sensors and track_filter.reset() exactly once
- ACQUIRE transitions to TRACK when track_filter.is_initialised() returns True
- _compute_aim_angles returns correct (pan, tilt) for known inputs
- _target_in_range returns True/False for known positions
"""
import math
import pytest
from stubs import StubFCR, StubSearchRadar, StubTrackFilter, StubMotor
from fsm import AtlasFSM, SensorSuite, TurretHardware, FSMConfig


# ---------------------------------------------------------------------------
# Helpers / fixtures
# ---------------------------------------------------------------------------

TURRET_POS = [0.0, 0.0, 0.0]


def _make_fsm(
    detections=None,
    acquire_frames=3,
    search_speed=0.1,
    search_pan_limit=1.0,
    track_filter=None,
):
    """Build an AtlasFSM wired to stubs.

    Args:
        detections: list of detection vectors returned by StubSearchRadar.
            Defaults to [] (no detections).
        acquire_frames: FSMConfig.acquire_frames — kept small for fast tests.
        search_speed: radians per step during pan sweep.
        search_pan_limit: pan sweep extent in radians.
        track_filter: inject a custom track_filter stub; defaults to
            StubTrackFilter with is_initialised() == True.

    Returns:
        (fsm, pan_motor, tilt_motor) tuple for inspection.
    """
    if detections is None:
        detections = []
    if track_filter is None:
        track_filter = StubTrackFilter(position=[1.0, 1.0, 1.0], velocity=[0.0, 0.0, 0.0])

    pan_motor = StubMotor()
    tilt_motor = StubMotor()
    sensors = SensorSuite(
        search_radar=StubSearchRadar(detections),
        fcr=StubFCR(position=[1.0, 1.0, 1.0]),
        track_filter=track_filter,
        ballistic_predictor=None,
    )
    hardware = TurretHardware(
        pan_motor=pan_motor,
        tilt_motor=tilt_motor,
        turret_position=TURRET_POS,
        timestep_ms=32,
    )
    config = FSMConfig(
        acquire_frames=acquire_frames,
        search_speed=search_speed,
        search_pan_limit=search_pan_limit,
    )
    fsm = AtlasFSM(sensors, hardware, config)
    return fsm, pan_motor, tilt_motor


# ---------------------------------------------------------------------------
# Initial state
# ---------------------------------------------------------------------------

def test_initial_state_is_search():
    """FSM must start in the SEARCH state."""
    fsm, _, _ = _make_fsm()
    assert fsm.state == AtlasFSM.SEARCH


# ---------------------------------------------------------------------------
# _do_search — pan motor movement
# ---------------------------------------------------------------------------

def test_search_moves_pan_motor_by_search_speed():
    """Each step in SEARCH must advance the pan motor by search_speed."""
    fsm, pan, _ = _make_fsm(search_speed=0.1, search_pan_limit=1.0)
    fsm.step()
    assert abs(pan.position) == pytest.approx(0.1)


def test_search_pan_motor_advances_incrementally():
    """After N steps, pan must be at N * search_speed (before any reversal)."""
    fsm, pan, _ = _make_fsm(search_speed=0.05, search_pan_limit=1.0)
    for _ in range(4):
        fsm.step()
    assert pan.position == pytest.approx(4 * 0.05)


def test_search_pan_reverses_at_positive_limit():
    """Pan must reverse direction when it reaches +search_pan_limit."""
    speed = 0.5
    limit = 1.0
    fsm, pan, _ = _make_fsm(search_speed=speed, search_pan_limit=limit)
    # Step until we definitely would exceed the limit without reversal
    # 3 steps: 0.5 → 1.0 (hits limit) → 0.5 (reversed)
    fsm.step()  # 0.5
    assert pan.position == pytest.approx(0.5)
    fsm.step()  # 1.0 — at limit, reverse after this
    assert pan.position == pytest.approx(1.0)
    fsm.step()  # direction reversed → 0.5
    assert pan.position == pytest.approx(0.5)


def test_search_pan_reverses_at_negative_limit():
    """Pan must reverse direction when it reaches -search_pan_limit."""
    speed = 0.5
    limit = 1.0
    fsm, pan, _ = _make_fsm(search_speed=speed, search_pan_limit=limit)
    # Force direction to be negative initially by doing a forward sweep then reversal
    # Steps: 0.5, 1.0, 0.5, 0.0, -0.5, -1.0 (hit -limit), -0.5 (reversed)
    for _ in range(6):
        fsm.step()
    assert pan.position == pytest.approx(-1.0)
    fsm.step()  # reverse back → -0.5
    assert pan.position == pytest.approx(-0.5)


# ---------------------------------------------------------------------------
# _do_search — detection counting and SEARCH→ACQUIRE transition
# ---------------------------------------------------------------------------

def test_search_no_detections_stays_in_search():
    """With no detections the FSM must remain in SEARCH indefinitely."""
    fsm, _, _ = _make_fsm(detections=[], acquire_frames=3)
    for _ in range(10):
        fsm.step()
    assert fsm.state == AtlasFSM.SEARCH


def test_search_consecutive_detections_transition_out_of_search():
    """After acquire_frames consecutive detection steps, FSM must have left SEARCH.

    Uses initialised=False so ACQUIRE entry actions fire but the FSM stays in
    ACQUIRE (rather than immediately falling through to TRACK), making it easy
    to assert state == ACQUIRE.
    """
    detection = [[1.0, 2.0, 0.5]]
    track_filter = StubTrackFilter.__new__(StubTrackFilter)
    track_filter._position = [1.0, 1.0, 1.0]
    track_filter._velocity = [0.0, 0.0, 0.0]

    class NotYetInitFilter:
        def reset(self): pass
        def is_initialised(self): return False

    fsm, _, _ = _make_fsm(
        detections=detection, acquire_frames=3,
        track_filter=NotYetInitFilter()
    )
    for _ in range(3):
        fsm.step()
    assert fsm.state == AtlasFSM.ACQUIRE


def test_search_requires_consecutive_detections():
    """A no-detection frame resets the counter; acquire_frames must start over."""
    detection = [[1.0, 2.0, 0.5]]
    no_detection = []

    # Use a custom StubSearchRadar that alternates detections and blanks
    class AlternatingRadar:
        def __init__(self):
            self._calls = 0
            self.set_target_called = False

        def get_detections(self):
            self._calls += 1
            # Two detections, one blank, two more detections (total 5 calls)
            if self._calls in (1, 2, 4, 5):
                return [[1.0, 2.0, 0.5]]
            return []

        def set_target(self, node):
            self.set_target_called = True

    radar = AlternatingRadar()
    pan_motor = StubMotor()
    tilt_motor = StubMotor()
    sensors = SensorSuite(
        search_radar=radar,
        fcr=StubFCR(position=[1.0, 1.0, 1.0]),
        track_filter=StubTrackFilter(position=[1.0, 1.0, 1.0], velocity=[0.0, 0.0, 0.0]),
        ballistic_predictor=None,
    )
    hardware = TurretHardware(
        pan_motor=pan_motor,
        tilt_motor=tilt_motor,
        turret_position=TURRET_POS,
        timestep_ms=32,
    )
    config = FSMConfig(acquire_frames=3)
    fsm = AtlasFSM(sensors, hardware, config)

    # Step 1: detection (count=1)
    fsm.step()
    assert fsm.state == AtlasFSM.SEARCH
    # Step 2: detection (count=2)
    fsm.step()
    assert fsm.state == AtlasFSM.SEARCH
    # Step 3: blank → reset (count=0)
    fsm.step()
    assert fsm.state == AtlasFSM.SEARCH
    # Step 4: detection (count=1) — must NOT have transitioned
    fsm.step()
    assert fsm.state == AtlasFSM.SEARCH
    # Step 5: detection (count=2) — still not at acquire_frames=3
    fsm.step()
    assert fsm.state == AtlasFSM.SEARCH


def test_search_single_frame_acquire():
    """acquire_frames=1 means a single detection step triggers ACQUIRE.

    Uses a not-yet-initialised filter so the FSM stays in ACQUIRE after entry.
    """
    class NotYetInitFilter:
        def reset(self): pass
        def is_initialised(self): return False

    detection = [[1.0, 2.0, 0.5]]
    fsm, _, _ = _make_fsm(
        detections=detection, acquire_frames=1,
        track_filter=NotYetInitFilter()
    )
    fsm.step()
    assert fsm.state == AtlasFSM.ACQUIRE


# ---------------------------------------------------------------------------
# _do_acquire — entry actions and ACQUIRE→TRACK transition
# ---------------------------------------------------------------------------

class CapturingSearchRadar(StubSearchRadar):
    """StubSearchRadar that records set_target() calls."""

    def __init__(self, detections):
        super().__init__(detections)
        self.set_target_calls = []

    def set_target(self, node):
        self.set_target_calls.append(node)


class CapturingFCR(StubFCR):
    """StubFCR that records set_target() calls."""

    def __init__(self, position):
        super().__init__(position)
        self.set_target_calls = []

    def set_target(self, node):
        self.set_target_calls.append(node)


class CapturingTrackFilter(StubTrackFilter):
    """StubTrackFilter that records reset() calls."""

    def __init__(self, position, velocity, initialised=True):
        super().__init__(position, velocity)
        self.reset_calls = 0
        self._initialised = initialised

    def reset(self):
        self.reset_calls += 1

    def is_initialised(self):
        return self._initialised


def _make_fsm_with_capturing(acquire_frames=1, initialised=True):
    """Build an FSM with capturing stubs for verifying entry actions."""
    detection = [[1.0, 2.0, 0.5]]
    pan_motor = StubMotor()
    tilt_motor = StubMotor()
    search_radar = CapturingSearchRadar(detection)
    fcr = CapturingFCR(position=[1.0, 1.0, 1.0])
    track_filter = CapturingTrackFilter(
        position=[1.0, 1.0, 1.0], velocity=[0.0, 0.0, 0.0], initialised=initialised
    )
    sensors = SensorSuite(
        search_radar=search_radar,
        fcr=fcr,
        track_filter=track_filter,
        ballistic_predictor=None,
    )
    hardware = TurretHardware(
        pan_motor=pan_motor,
        tilt_motor=tilt_motor,
        turret_position=TURRET_POS,
        timestep_ms=32,
    )
    config = FSMConfig(acquire_frames=acquire_frames)
    fsm = AtlasFSM(sensors, hardware, config)
    return fsm, search_radar, fcr, track_filter


def test_acquire_calls_set_target_on_fcr():
    """On entering ACQUIRE, fcr.set_target() must be called exactly once.

    Uses initialised=False so the FSM stays in ACQUIRE (does not immediately
    fall through to TRACK), allowing the state assertion to be meaningful.
    """
    fsm, search_radar, fcr, track_filter = _make_fsm_with_capturing(
        acquire_frames=1, initialised=False
    )
    fsm.step()  # detection → ACQUIRE, entry actions fire; filter not initialised → stays
    assert fsm.state == AtlasFSM.ACQUIRE
    assert len(fcr.set_target_calls) == 1


def test_acquire_calls_set_target_on_search_radar():
    """On entering ACQUIRE, search_radar.set_target() must be called exactly once."""
    fsm, search_radar, fcr, track_filter = _make_fsm_with_capturing(acquire_frames=1)
    fsm.step()  # detection → ACQUIRE, entry actions fire
    assert len(search_radar.set_target_calls) == 1


def test_acquire_calls_track_filter_reset():
    """On entering ACQUIRE, track_filter.reset() must be called exactly once."""
    fsm, search_radar, fcr, track_filter = _make_fsm_with_capturing(acquire_frames=1)
    fsm.step()  # detection → ACQUIRE, entry actions fire
    assert track_filter.reset_calls == 1


def test_acquire_entry_actions_fire_exactly_once():
    """Entry actions (set_target / reset) must NOT repeat on subsequent steps in ACQUIRE."""
    # initialised=False so FSM stays in ACQUIRE for multiple steps
    fsm, search_radar, fcr, track_filter = _make_fsm_with_capturing(
        acquire_frames=1, initialised=False
    )
    fsm.step()  # SEARCH → ACQUIRE (entry actions fire)
    fsm.step()  # still ACQUIRE (entry actions must NOT fire again)
    fsm.step()  # still ACQUIRE
    assert len(fcr.set_target_calls) == 1
    assert len(search_radar.set_target_calls) == 1
    assert track_filter.reset_calls == 1


def test_acquire_transitions_to_track_when_filter_initialised():
    """ACQUIRE must transition to TRACK once track_filter.is_initialised() is True."""
    fsm, search_radar, fcr, track_filter = _make_fsm_with_capturing(
        acquire_frames=1, initialised=True
    )
    # Step 1: SEARCH → ACQUIRE (entry actions); track_filter is already initialised
    # so the same step or the next should move us to TRACK
    fsm.step()
    # After entry, filter is initialised, so should be in TRACK
    assert fsm.state == AtlasFSM.TRACK


def test_acquire_stays_in_acquire_while_filter_not_initialised():
    """ACQUIRE must remain in ACQUIRE while track_filter.is_initialised() is False."""
    fsm, search_radar, fcr, track_filter = _make_fsm_with_capturing(
        acquire_frames=1, initialised=False
    )
    fsm.step()  # SEARCH → ACQUIRE
    assert fsm.state == AtlasFSM.ACQUIRE
    fsm.step()  # still ACQUIRE
    assert fsm.state == AtlasFSM.ACQUIRE


def test_acquire_transitions_to_track_after_filter_initialises():
    """ACQUIRE must move to TRACK on the step when is_initialised() first returns True."""

    class LateInitFilter(StubTrackFilter):
        """is_initialised() returns False for the first N calls, then True."""

        def __init__(self, delay):
            super().__init__(position=[1.0, 1.0, 1.0], velocity=[0.0, 0.0, 0.0])
            self._delay = delay
            self._calls = 0
            self.reset_calls = 0

        def reset(self):
            self.reset_calls += 1

        def is_initialised(self):
            self._calls += 1
            return self._calls > self._delay

    late_filter = LateInitFilter(delay=2)
    detection = [[1.0, 2.0, 0.5]]
    pan_motor = StubMotor()
    tilt_motor = StubMotor()
    sensors = SensorSuite(
        search_radar=CapturingSearchRadar(detection),
        fcr=CapturingFCR(position=[1.0, 1.0, 1.0]),
        track_filter=late_filter,
        ballistic_predictor=None,
    )
    hardware = TurretHardware(
        pan_motor=pan_motor,
        tilt_motor=tilt_motor,
        turret_position=TURRET_POS,
        timestep_ms=32,
    )
    config = FSMConfig(acquire_frames=1)
    fsm = AtlasFSM(sensors, hardware, config)

    fsm.step()   # SEARCH → ACQUIRE; is_initialised call 1 → False
    assert fsm.state == AtlasFSM.ACQUIRE
    fsm.step()   # ACQUIRE; is_initialised call 2 → False
    assert fsm.state == AtlasFSM.ACQUIRE
    fsm.step()   # ACQUIRE; is_initialised call 3 → True → transition to TRACK
    assert fsm.state == AtlasFSM.TRACK


# ---------------------------------------------------------------------------
# _compute_aim_angles
# ---------------------------------------------------------------------------

def test_compute_aim_angles_straight_ahead():
    """Target directly in front (dy>0, dz=0, dx=0) → pan=0, tilt=0."""
    fsm, _, _ = _make_fsm()
    pan, tilt = fsm._compute_aim_angles([0.0, 5.0, 0.0])
    assert pan == pytest.approx(0.0)
    assert tilt == pytest.approx(0.0)


def test_compute_aim_angles_target_to_the_right():
    """Target 45° to the right: dx=1, dy=1, dz=0 → pan=pi/4, tilt=0."""
    fsm, _, _ = _make_fsm()
    pan, tilt = fsm._compute_aim_angles([1.0, 1.0, 0.0])
    assert pan == pytest.approx(math.pi / 4)
    assert tilt == pytest.approx(0.0)


def test_compute_aim_angles_target_above():
    """Target directly above and ahead: dx=0, dy=1, dz=1 → pan=0, tilt=pi/4."""
    fsm, _, _ = _make_fsm()
    pan, tilt = fsm._compute_aim_angles([0.0, 1.0, 1.0])
    assert pan == pytest.approx(0.0)
    assert tilt == pytest.approx(math.pi / 4)


def test_compute_aim_angles_combined():
    """Combined angle: dx=1, dy=0, dz=0 → pan=pi/2, tilt=0."""
    fsm, _, _ = _make_fsm()
    pan, tilt = fsm._compute_aim_angles([1.0, 0.0, 0.0])
    assert pan == pytest.approx(math.pi / 2)
    assert tilt == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# _target_in_range
# ---------------------------------------------------------------------------

def test_target_in_range_within_bounds():
    """A nearby, above-ground target must be in range."""
    fsm, _, _ = _make_fsm()
    # rel_position = [dx, dy, dz]; dz > ground_threshold (0.1) and
    # distance < max_range (10.0)
    assert fsm._target_in_range([1.0, 1.0, 1.0]) is True


def test_target_in_range_too_far():
    """A target beyond max_range must not be in range."""
    fsm, _, _ = _make_fsm()
    # Distance = sqrt(10^2 + 10^2 + 10^2) ≈ 17.3 > max_range=10.0
    assert fsm._target_in_range([10.0, 10.0, 10.0]) is False


def test_target_in_range_below_ground_threshold():
    """A target below ground_threshold (dz=0.0) must not be in range."""
    fsm, _, _ = _make_fsm()
    # dz=0.0 ≤ ground_threshold=0.1
    assert fsm._target_in_range([0.5, 0.5, 0.0]) is False


def test_target_in_range_exactly_at_max_range():
    """A target exactly at max_range=10.0 on one axis must be in range (≤ check)."""
    fsm, _, _ = _make_fsm()
    # Distance = 10.0, dz = 1.0 > ground_threshold
    assert fsm._target_in_range([0.0, 0.0, 10.0]) is True
