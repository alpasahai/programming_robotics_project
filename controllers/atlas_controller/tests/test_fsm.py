"""Tests for AtlasFSM — finite state machine state handlers."""
import math
import pytest
from stubs import StubFCR, StubSearchRadar, StubTrackFilter, StubMotor
from search_radar import Detection
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
        detections: list of Detection objects returned by StubSearchRadar.
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

    Uses initialised=False so the FSM stays in ACQUIRE (rather than immediately
    falling through to TRACK on the next step), making it easy to assert
    state == ACQUIRE.

    Step sequence (acquire_frames=3):
      Steps 1-3: SEARCH handler counts 3 consecutive detections → transitions
                 to ACQUIRE on step 3 (no ACQUIRE handler runs yet).
      Step 4:    ACQUIRE handler runs for the first time; filter not initialised
                 → remains in ACQUIRE.
    """
    detection = [Detection(track_id=0, position=[1.0, 2.0, 0.5])]
    track_filter = CapturingTrackFilter(
        position=[1.0, 1.0, 1.0], velocity=[0.0, 0.0, 0.0], initialised=False
    )

    fsm, _, _ = _make_fsm(
        detections=detection, acquire_frames=3,
        track_filter=track_filter,
    )
    for _ in range(3):
        fsm.step()
    # After 3 steps the transition has fired but _do_acquire has not yet run.
    # State must already be ACQUIRE.
    assert fsm.state == AtlasFSM.ACQUIRE


def test_search_requires_consecutive_detections():
    """A no-detection frame resets the counter; acquire_frames must start over."""

    # Use a custom search radar that alternates detections and blanks
    class AlternatingRadar:
        def __init__(self):
            self._calls = 0
            self.set_target_called = False

        def get_detections(self):
            self._calls += 1
            # Two detections, one blank, two more detections (total 5 calls)
            if self._calls in (1, 2, 4, 5):
                return [Detection(track_id=0, position=[1.0, 2.0, 0.5])]
            return []

        def set_target(self, track_id):
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

    Uses a not-yet-initialised filter so the FSM stays in ACQUIRE after the
    ACQUIRE handler runs on the following step.

    Step sequence:
      Step 1: SEARCH handler detects target → transitions to ACQUIRE (no ACQUIRE
              handler runs yet).
      Step 2: ACQUIRE handler runs; filter not initialised → remains in ACQUIRE.
    """
    track_filter = CapturingTrackFilter(
        position=[1.0, 1.0, 1.0], velocity=[0.0, 0.0, 0.0], initialised=False
    )
    detection = [Detection(track_id=0, position=[1.0, 2.0, 0.5])]
    fsm, _, _ = _make_fsm(
        detections=detection, acquire_frames=1,
        track_filter=track_filter,
    )
    fsm.step()   # SEARCH → ACQUIRE (transition only; ACQUIRE handler not yet run)
    assert fsm.state == AtlasFSM.ACQUIRE
    fsm.step()   # ACQUIRE handler runs; filter not initialised → stays in ACQUIRE
    assert fsm.state == AtlasFSM.ACQUIRE


def test_search_selects_first_detection_track_id_as_target():
    """When transitioning to ACQUIRE, fsm._target must be the track_id int from detections[0].

    Per ADR-0006 the FSM holds only an integer track_id, never a position list
    or a Webots node.
    """
    track_filter = CapturingTrackFilter(
        position=[1.0, 1.0, 1.0], velocity=[0.0, 0.0, 0.0], initialised=False
    )
    # Two detections; FSM must pick track_id=0 (the first one)
    detections = [
        Detection(track_id=0, position=[1.0, 2.0, 0.5]),
        Detection(track_id=1, position=[3.0, 4.0, 1.0]),
    ]
    fsm, _, _ = _make_fsm(detections=detections, acquire_frames=1, track_filter=track_filter)
    fsm.step()  # SEARCH → ACQUIRE
    assert fsm._target == 0
    assert isinstance(fsm._target, int)


# ---------------------------------------------------------------------------
# _do_acquire — entry actions and ACQUIRE→TRACK transition
# ---------------------------------------------------------------------------

class CapturingSearchRadar(StubSearchRadar):
    """StubSearchRadar that records set_target() calls."""

    def __init__(self, detections):
        super().__init__(detections)
        self.set_target_calls = []

    def set_target(self, track_id):
        self.set_target_calls.append(track_id)


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
    """Build an FSM with capturing stubs for verifying ACQUIRE entry actions.

    Args:
        acquire_frames: consecutive detection steps needed to leave SEARCH.
            Default 1 so the very first step triggers the SEARCH→ACQUIRE
            transition, minimising setup noise in ACQUIRE-focused tests.
        initialised: passed to CapturingTrackFilter.  When False the FSM
            stays in ACQUIRE indefinitely (filter never initialises), letting
            callers assert ACQUIRE-internal behaviour without the FSM
            advancing to TRACK.  When True the FSM moves to TRACK on the
            first step that runs the ACQUIRE handler.

    Returns:
        (fsm, search_radar, fcr, track_filter) — the FSM and each capturing
        stub, all pre-wired together and ready for fsm.step() calls.
    """
    detection = [Detection(track_id=0, position=[1.0, 2.0, 0.5])]
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


def test_acquire_calls_set_target_on_search_radar_with_track_id():
    """On entering ACQUIRE, search_radar.set_target() must be called with the integer track_id.

    Per ADR-0006 the FSM passes an integer track_id (not a node or position) to
    SearchRadar.set_target(). The SearchRadar resolves the node internally.

    Step sequence (acquire_frames=1, initialised=False):
      Step 1: SEARCH → ACQUIRE transition (no ACQUIRE handler yet).
      Step 2: ACQUIRE handler runs; entry actions fire; filter not init → stays.
    """
    fsm, search_radar, fcr, track_filter = _make_fsm_with_capturing(
        acquire_frames=1, initialised=False
    )
    fsm.step()  # SEARCH → ACQUIRE (transition only)
    fsm.step()  # ACQUIRE handler: entry actions fire; filter not initialised → stays
    assert fsm.state == AtlasFSM.ACQUIRE
    assert len(search_radar.set_target_calls) == 1
    assert search_radar.set_target_calls[0] == 0   # track_id of the first detection
    assert isinstance(search_radar.set_target_calls[0], int)


def test_acquire_does_not_call_set_target_on_fcr():
    """On entering ACQUIRE, fcr.set_target() must NOT be called.

    Per ADR-0006 FCR is single-target and cued at construction; the FSM does
    not re-target it.

    Step sequence (acquire_frames=1, initialised=False):
      Step 1: SEARCH → ACQUIRE transition (no ACQUIRE handler yet).
      Step 2: ACQUIRE handler runs; entry actions fire; filter not init → stays.
    """
    fsm, search_radar, fcr, track_filter = _make_fsm_with_capturing(
        acquire_frames=1, initialised=False
    )
    fsm.step()  # SEARCH → ACQUIRE (transition only)
    fsm.step()  # ACQUIRE handler: entry actions fire
    assert len(fcr.set_target_calls) == 0


def test_acquire_calls_set_target_on_search_radar():
    """On entering ACQUIRE, search_radar.set_target() must be called exactly once.

    Step sequence (acquire_frames=1, initialised=True):
      Step 1: SEARCH → ACQUIRE transition (no ACQUIRE handler yet).
      Step 2: ACQUIRE handler runs; entry actions fire; filter initialised → TRACK.
    """
    fsm, search_radar, fcr, track_filter = _make_fsm_with_capturing(acquire_frames=1)
    fsm.step()  # SEARCH → ACQUIRE (transition only)
    fsm.step()  # ACQUIRE handler: entry actions fire; filter initialised → TRACK
    assert len(search_radar.set_target_calls) == 1


def test_acquire_calls_track_filter_reset():
    """On entering ACQUIRE, track_filter.reset() must be called exactly once.

    Step sequence (acquire_frames=1, initialised=True):
      Step 1: SEARCH → ACQUIRE transition (no ACQUIRE handler yet).
      Step 2: ACQUIRE handler runs; entry actions fire; filter initialised → TRACK.
    """
    fsm, search_radar, fcr, track_filter = _make_fsm_with_capturing(acquire_frames=1)
    fsm.step()  # SEARCH → ACQUIRE (transition only)
    fsm.step()  # ACQUIRE handler: entry actions fire; filter initialised → TRACK
    assert track_filter.reset_calls == 1


def test_acquire_entry_actions_fire_exactly_once():
    """Entry actions (set_target / reset) must NOT repeat on subsequent steps in ACQUIRE.

    Step sequence (acquire_frames=1, initialised=False):
      Step 1: SEARCH → ACQUIRE transition (no ACQUIRE handler yet).
      Step 2: ACQUIRE handler runs for the first time; entry actions fire.
      Step 3: ACQUIRE handler runs again; entry actions must NOT fire again.
      Step 4: ACQUIRE handler runs again; entry actions must NOT fire again.
    """
    # initialised=False so FSM stays in ACQUIRE for multiple steps
    fsm, search_radar, fcr, track_filter = _make_fsm_with_capturing(
        acquire_frames=1, initialised=False
    )
    fsm.step()  # SEARCH → ACQUIRE (transition only; no ACQUIRE handler yet)
    fsm.step()  # ACQUIRE handler runs; entry actions fire
    fsm.step()  # still ACQUIRE; entry actions must NOT fire again
    fsm.step()  # still ACQUIRE; entry actions must NOT fire again
    assert len(search_radar.set_target_calls) == 1
    assert len(fcr.set_target_calls) == 0
    assert track_filter.reset_calls == 1


def test_acquire_transitions_to_track_when_filter_initialised():
    """ACQUIRE must transition to TRACK once track_filter.is_initialised() is True.

    Step sequence (acquire_frames=1, initialised=True):
      Step 1: SEARCH → ACQUIRE transition (no ACQUIRE handler yet).
      Step 2: ACQUIRE handler runs; entry actions fire; filter already initialised
              → transitions to TRACK immediately.
    """
    fsm, search_radar, fcr, track_filter = _make_fsm_with_capturing(
        acquire_frames=1, initialised=True
    )
    fsm.step()   # SEARCH → ACQUIRE (transition only)
    fsm.step()   # ACQUIRE handler: entry actions fire; filter initialised → TRACK
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
    detection = [Detection(track_id=0, position=[1.0, 2.0, 0.5])]
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

    # Step sequence (acquire_frames=1, LateInitFilter delay=2):
    #   Step 1: SEARCH → ACQUIRE transition; is_initialised not called yet.
    #   Step 2: ACQUIRE handler; entry actions fire; is_initialised call 1 → False.
    #   Step 3: ACQUIRE handler; is_initialised call 2 → False.
    #   Step 4: ACQUIRE handler; is_initialised call 3 → True → TRACK.
    fsm.step()   # SEARCH → ACQUIRE (transition only; no ACQUIRE handler yet)
    assert fsm.state == AtlasFSM.ACQUIRE
    fsm.step()   # ACQUIRE handler; is_initialised call 1 → False
    assert fsm.state == AtlasFSM.ACQUIRE
    fsm.step()   # ACQUIRE handler; is_initialised call 2 → False
    assert fsm.state == AtlasFSM.ACQUIRE
    fsm.step()   # ACQUIRE handler; is_initialised call 3 → True → TRACK
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


# ---------------------------------------------------------------------------
# _do_track — helpers
# ---------------------------------------------------------------------------

def _make_fsm_in_track(
    position=None,
    min_track_frames=5,
):
    """Build an AtlasFSM already in TRACK state with a fixed filter position.

    The FSM state and _track_frames counter are set directly — this avoids
    stepping through SEARCH and ACQUIRE in every TRACK/PREDICT test, and is
    consistent with how the existing test suite manipulates mid-pipeline state.

    Args:
        position:         Relative [dx, dy, dz] that StubTrackFilter reports.
                          Defaults to [1.0, 2.0, 3.0].
        min_track_frames: FSMConfig.min_track_frames — kept small for fast tests.

    Returns:
        (fsm, pan_motor, tilt_motor) tuple.
    """
    if position is None:
        position = [1.0, 2.0, 3.0]

    track_filter = StubTrackFilter(position=position, velocity=[0.0, 0.0, 0.0])
    pan_motor = StubMotor()
    tilt_motor = StubMotor()
    sensors = SensorSuite(
        search_radar=StubSearchRadar([]),
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
    config = FSMConfig(min_track_frames=min_track_frames)
    fsm = AtlasFSM(sensors, hardware, config)
    fsm.state = AtlasFSM.TRACK
    return fsm, pan_motor, tilt_motor


# ---------------------------------------------------------------------------
# _do_track — motor aiming
# ---------------------------------------------------------------------------

def test_track_aims_pan_motor_at_filtered_position():
    """TRACK must command the pan motor to the angle computed from the filter position.

    Position [1.0, 2.0, 3.0] → pan = atan2(1.0, 2.0).
    """
    position = [1.0, 2.0, 3.0]
    fsm, pan, _ = _make_fsm_in_track(position=position)
    fsm.step()
    expected_pan = math.atan2(position[0], position[1])
    assert pan.position == pytest.approx(expected_pan)


def test_track_aims_tilt_motor_at_filtered_position():
    """TRACK must command the tilt motor to the angle computed from the filter position.

    Position [1.0, 2.0, 3.0] → tilt = atan2(3.0, sqrt(1.0² + 2.0²)).
    """
    position = [1.0, 2.0, 3.0]
    fsm, _, tilt = _make_fsm_in_track(position=position)
    fsm.step()
    expected_tilt = math.atan2(position[2], math.sqrt(position[0]**2 + position[1]**2))
    assert tilt.position == pytest.approx(expected_tilt)


def test_track_increments_frame_counter_each_step():
    """_track_frames must increment by 1 on every TRACK step."""
    fsm, _, _ = _make_fsm_in_track(min_track_frames=100)
    for expected in range(1, 4):
        fsm.step()
        assert fsm._track_frames == expected


def test_track_stays_in_track_before_min_frames():
    """TRACK must remain in TRACK while frame count < min_track_frames."""
    min_track_frames = 3
    fsm, _, _ = _make_fsm_in_track(min_track_frames=min_track_frames)
    for _ in range(min_track_frames - 1):
        fsm.step()
    assert fsm.state == AtlasFSM.TRACK


def test_track_transitions_to_predict_at_min_frames():
    """TRACK must transition to PREDICT on the step that reaches min_track_frames.

    The transition fires when _track_frames reaches min_track_frames; the PREDICT
    handler does NOT run on that same step (single-dispatch per step).
    """
    min_track_frames = 3
    fsm, _, _ = _make_fsm_in_track(min_track_frames=min_track_frames)
    for _ in range(min_track_frames):
        fsm.step()
    assert fsm.state == AtlasFSM.PREDICT


# ---------------------------------------------------------------------------
# _do_predict — helpers
# ---------------------------------------------------------------------------

class StubPredictor:
    """Minimal ballistic predictor stub that returns a fixed intercept.

    Args:
        intercept: The [dx, dy, dz] value returned by get_intercept(), regardless
                   of the lookahead_steps argument.
    """

    def __init__(self, intercept):
        self._intercept = intercept

    def get_intercept(self, lookahead_steps):
        return list(self._intercept)


def _make_fsm_in_predict(intercept, max_range=10.0, ground_threshold=0.1):
    """Build an AtlasFSM in PREDICT state with a controllable StubPredictor.

    Args:
        intercept:        The [dx, dy, dz] intercept the predictor will return.
        max_range:        FSMConfig.max_range (metres).
        ground_threshold: FSMConfig.ground_threshold (metres).

    Returns:
        (fsm, pan_motor, tilt_motor) tuple. Motors are included for structural
        symmetry with _make_fsm_in_track; PREDICT does not command them.
    """
    predictor = StubPredictor(intercept)
    track_filter = StubTrackFilter(position=[1.0, 1.0, 1.0], velocity=[0.0, 0.0, 0.0])
    pan_motor = StubMotor()
    tilt_motor = StubMotor()
    sensors = SensorSuite(
        search_radar=StubSearchRadar([]),
        fcr=StubFCR(position=[1.0, 1.0, 1.0]),
        track_filter=track_filter,
        ballistic_predictor=predictor,
    )
    hardware = TurretHardware(
        pan_motor=pan_motor,
        tilt_motor=tilt_motor,
        turret_position=TURRET_POS,
        timestep_ms=32,
    )
    config = FSMConfig(max_range=max_range, ground_threshold=ground_threshold)
    fsm = AtlasFSM(sensors, hardware, config)
    fsm.state = AtlasFSM.PREDICT
    return fsm, pan_motor, tilt_motor


# ---------------------------------------------------------------------------
# _do_predict — transitions and intercept storage
# ---------------------------------------------------------------------------

def test_predict_valid_intercept_transitions_to_aiming():
    """PREDICT with a valid (in-range, above-ground) intercept must transition to AIMING."""
    # [0.0, 2.0, 1.0]: distance=sqrt(5)≈2.24 < max_range=10.0; dz=1.0 > ground_threshold=0.1
    fsm, _, _ = _make_fsm_in_predict(intercept=[0.0, 2.0, 1.0])
    fsm.step()
    assert fsm.state == AtlasFSM.AIMING


def test_predict_valid_intercept_stores_intercept():
    """PREDICT with a valid intercept must store it on self._intercept."""
    intercept = [0.0, 2.0, 1.0]
    fsm, _, _ = _make_fsm_in_predict(intercept=intercept)
    fsm.step()
    assert fsm._intercept == intercept


def test_predict_out_of_range_intercept_transitions_to_track():
    """PREDICT with an out-of-range intercept must transition back to TRACK.

    Distance = sqrt(8² + 8² + 8²) ≈ 13.86 > max_range=10.0 → invalid.
    """
    fsm, _, _ = _make_fsm_in_predict(intercept=[8.0, 8.0, 8.0], max_range=10.0)
    fsm.step()
    assert fsm.state == AtlasFSM.TRACK


def test_predict_below_ground_intercept_transitions_to_track():
    """PREDICT with a below-ground intercept must transition back to TRACK.

    dz = 0.0 ≤ ground_threshold=0.1 → invalid regardless of distance.
    """
    fsm, _, _ = _make_fsm_in_predict(
        intercept=[1.0, 1.0, 0.0], ground_threshold=0.1
    )
    fsm.step()
    assert fsm.state == AtlasFSM.TRACK


def test_predict_invalid_intercept_resets_track_bookkeeping():
    """PREDICT→TRACK (invalid intercept) must reset _track_frames to 0 AND clear _intercept.

    Guards the _track_frames reset (existing) and the _intercept clear (fix 1) so
    that a PREDICT→TRACK transition never leaves stale intercept data for AIMING to see.
    """
    # Seed a stale intercept to confirm it is cleared on re-entry to TRACK.
    fsm, _, _ = _make_fsm_in_predict(intercept=[8.0, 8.0, 8.0], max_range=10.0)
    fsm._intercept = [0.0, 2.0, 1.0]   # stale value from a previous PREDICT cycle
    fsm.step()
    assert fsm.state == AtlasFSM.TRACK
    assert fsm._track_frames == 0
    assert fsm._intercept is None


# ---------------------------------------------------------------------------
# _do_aim — helpers
# ---------------------------------------------------------------------------

def _make_fsm_in_aiming(
    intercept=None,
    aim_error_threshold=0.05,
    track_position=None,
):
    """Build an AtlasFSM already in AIMING state with a fixed intercept.

    The FSM state is set directly (no stepping through earlier states) to keep
    AIMING tests fast and isolated. The intercept is placed on self._intercept.
    The last-commanded motor angles (_commanded_pan, _commanded_tilt) are left
    at the post-__init__ default (0.0, 0.0) so the first step always has a
    non-zero error unless the intercept happens to be dead-ahead.

    Args:
        intercept:           Relative [dx, dy, dz] stored as self._intercept.
                             Defaults to [1.0, 2.0, 1.0].
        aim_error_threshold: FSMConfig.aim_error_threshold (radians).
        track_position:      Position StubTrackFilter returns (for ENGAGING tests
                             that reuse this helper). Defaults to [1.0, 2.0, 1.0].

    Returns:
        (fsm, pan_motor, tilt_motor) tuple.
    """
    if intercept is None:
        intercept = [1.0, 2.0, 1.0]
    if track_position is None:
        track_position = [1.0, 2.0, 1.0]

    track_filter = StubTrackFilter(position=track_position, velocity=[0.0, 0.0, 0.0])
    pan_motor = StubMotor()
    tilt_motor = StubMotor()
    sensors = SensorSuite(
        search_radar=StubSearchRadar([]),
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
    config = FSMConfig(aim_error_threshold=aim_error_threshold)
    fsm = AtlasFSM(sensors, hardware, config)
    fsm.state = AtlasFSM.AIMING
    fsm._intercept = list(intercept)
    return fsm, pan_motor, tilt_motor


# ---------------------------------------------------------------------------
# _do_aim — motor commands
# ---------------------------------------------------------------------------

def test_aim_commands_pan_motor_at_intercept():
    """AIMING must command the pan motor to the pan angle for the intercept.

    Intercept [1.0, 2.0, 1.0] → pan = atan2(1.0, 2.0).
    """
    intercept = [1.0, 2.0, 1.0]
    fsm, pan, _ = _make_fsm_in_aiming(intercept=intercept)
    fsm.step()
    expected_pan = math.atan2(intercept[0], intercept[1])
    assert pan.position == pytest.approx(expected_pan)


def test_aim_commands_tilt_motor_at_intercept():
    """AIMING must command the tilt motor to the tilt angle for the intercept.

    Intercept [1.0, 2.0, 1.0] → tilt = atan2(1.0, sqrt(1.0² + 2.0²)).
    """
    intercept = [1.0, 2.0, 1.0]
    fsm, _, tilt = _make_fsm_in_aiming(intercept=intercept)
    fsm.step()
    expected_tilt = math.atan2(intercept[2], math.sqrt(intercept[0]**2 + intercept[1]**2))
    assert tilt.position == pytest.approx(expected_tilt)


# ---------------------------------------------------------------------------
# _do_aim — convergence and AIMING→ENGAGING transition
# ---------------------------------------------------------------------------

def test_aim_stays_in_aiming_before_convergence():
    """AIMING must remain in AIMING on the first step (aim error is nonzero on entry).

    The FSM starts with _commanded_pan = _commanded_tilt = 0.0. On the first step
    the aim error is measured BEFORE commanding the motors (against the previous
    commanded angles, which are 0.0), so the error is nonzero. The motors are then
    commanded to the intercept angles and those angles are recorded. On the NEXT step
    the error will be zero (desired == newly commanded) and the FSM transitions to
    ENGAGING. Hence the first step stays in AIMING.

    Intercept [1.0, 2.0, 1.0] → nonzero pan, so initial error is nonzero.
    """
    intercept = [1.0, 2.0, 1.0]
    fsm, _, _ = _make_fsm_in_aiming(
        intercept=intercept,
        aim_error_threshold=0.05,
    )
    fsm.step()  # commands motors; error measured vs prior commanded → nonzero → stays
    assert fsm.state == AtlasFSM.AIMING


def test_aim_transitions_to_engaging_after_convergence():
    """AIMING must transition to ENGAGING once aim error falls below threshold.

    On the second AIMING step the commanded angles equal the desired angles
    (error == 0), which is below any positive threshold → transition fires.
    """
    intercept = [1.0, 2.0, 1.0]
    fsm, _, _ = _make_fsm_in_aiming(
        intercept=intercept,
        aim_error_threshold=0.05,
    )
    fsm.step()  # step 1: commands motors, error nonzero → stays in AIMING
    fsm.step()  # step 2: error == 0 < threshold → transitions to ENGAGING
    assert fsm.state == AtlasFSM.ENGAGING


def test_aim_already_aligned_transitions_immediately():
    """AIMING with a very large threshold must transition to ENGAGING on the first step.

    With aim_error_threshold=10.0 (>> any realistic angular error) the first-step
    error is below the threshold even before the motors have slewed, so AIMING
    transitions to ENGAGING immediately.
    """
    # The initial commanded angles are (0.0, 0.0). The intercept angles for
    # [1.0, 2.0, 1.0] are (atan2(1,2) ≈ 0.46, atan2(1, sqrt(5)) ≈ 0.42), giving
    # an error of ~0.63 rad — well below threshold=10.0.
    intercept = [1.0, 2.0, 1.0]
    fsm, _, _ = _make_fsm_in_aiming(
        intercept=intercept,
        aim_error_threshold=10.0,
    )
    fsm.step()
    assert fsm.state == AtlasFSM.ENGAGING


# ---------------------------------------------------------------------------
# _do_engage — helpers
# ---------------------------------------------------------------------------

def _make_fsm_in_engaging(
    intercept=None,
    track_position=None,
    max_range=10.0,
    ground_threshold=0.1,
):
    """Build an AtlasFSM already in ENGAGING state.

    Reuses _make_fsm_in_aiming internals and then advances to ENGAGING.

    Args:
        intercept:        Relative [dx, dy, dz] stored as self._intercept.
                          Defaults to [1.0, 2.0, 1.0] (in-range, above-ground).
        track_position:   Position that StubTrackFilter.get_position() returns.
                          Defaults to match the intercept (target still in range).
        max_range:        FSMConfig.max_range (metres).
        ground_threshold: FSMConfig.ground_threshold (metres).

    Returns:
        (fsm, pan_motor, tilt_motor) tuple.
    """
    if intercept is None:
        intercept = [1.0, 2.0, 1.0]
    if track_position is None:
        track_position = list(intercept)

    track_filter = StubTrackFilter(position=track_position, velocity=[0.0, 0.0, 0.0])
    pan_motor = StubMotor()
    tilt_motor = StubMotor()
    sensors = SensorSuite(
        search_radar=StubSearchRadar([]),
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
    config = FSMConfig(max_range=max_range, ground_threshold=ground_threshold)
    fsm = AtlasFSM(sensors, hardware, config)
    fsm.state = AtlasFSM.ENGAGING
    fsm._intercept = list(intercept)
    return fsm, pan_motor, tilt_motor


# ---------------------------------------------------------------------------
# _do_engage — laser and motor behaviour
# ---------------------------------------------------------------------------

def test_engage_sets_laser_active_true():
    """ENGAGING must set self.laser_active = True on every step."""
    fsm, _, _ = _make_fsm_in_engaging()
    assert fsm.laser_active is False   # pre-condition: starts False
    fsm.step()
    assert fsm.laser_active is True


def test_engage_holds_aim_at_intercept_pan():
    """ENGAGING must keep commanding the pan motor at the intercept pan angle."""
    intercept = [1.0, 2.0, 1.0]
    fsm, pan, _ = _make_fsm_in_engaging(intercept=intercept)
    fsm.step()
    expected_pan = math.atan2(intercept[0], intercept[1])
    assert pan.position == pytest.approx(expected_pan)


def test_engage_holds_aim_at_intercept_tilt():
    """ENGAGING must keep commanding the tilt motor at the intercept tilt angle."""
    intercept = [1.0, 2.0, 1.0]
    fsm, _, tilt = _make_fsm_in_engaging(intercept=intercept)
    fsm.step()
    expected_tilt = math.atan2(intercept[2], math.sqrt(intercept[0]**2 + intercept[1]**2))
    assert tilt.position == pytest.approx(expected_tilt)


def test_engage_stays_in_engaging_while_target_in_range():
    """ENGAGING must remain in ENGAGING while the target is within range and above ground."""
    # track_position in range (distance=sqrt(3)≈1.73 < 10.0, dz=1.0 > 0.1)
    fsm, _, _ = _make_fsm_in_engaging(track_position=[1.0, 1.0, 1.0])
    fsm.step()
    assert fsm.state == AtlasFSM.ENGAGING


def test_engage_transitions_to_reset_when_target_out_of_range():
    """ENGAGING must transition to RESET when the target goes out of range.

    Track position [9.0, 9.0, 9.0]: distance ≈ 15.6 > max_range=10.0 → out of range.
    """
    fsm, _, _ = _make_fsm_in_engaging(track_position=[9.0, 9.0, 9.0], max_range=10.0)
    fsm.step()
    assert fsm.state == AtlasFSM.RESET


def test_engage_transitions_to_reset_when_target_hits_ground():
    """ENGAGING must transition to RESET when the target hits the ground.

    Track position dz=0.0 ≤ ground_threshold=0.1 → target has landed.
    """
    fsm, _, _ = _make_fsm_in_engaging(
        track_position=[1.0, 1.0, 0.0],
        ground_threshold=0.1,
    )
    fsm.step()
    assert fsm.state == AtlasFSM.RESET


# ---------------------------------------------------------------------------
# _do_reset — state clearing and RESET→SEARCH transition
# ---------------------------------------------------------------------------

def _make_fsm_in_reset(intercept=None):
    """Build an AtlasFSM in RESET state with engagement bookkeeping populated.

    Sets laser_active=True, _intercept, _track_frames, _detection_count, and
    _target to non-default values so the test can verify RESET clears them all.

    Args:
        intercept: Relative [dx, dy, dz] pre-loaded into self._intercept.
                   Defaults to [1.0, 2.0, 1.0].

    Returns:
        (fsm, pan_motor, tilt_motor) tuple.
    """
    if intercept is None:
        intercept = [1.0, 2.0, 1.0]

    track_filter = StubTrackFilter(position=[1.0, 2.0, 1.0], velocity=[0.0, 0.0, 0.0])
    pan_motor = StubMotor()
    tilt_motor = StubMotor()
    sensors = SensorSuite(
        search_radar=StubSearchRadar([]),
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
    config = FSMConfig()
    fsm = AtlasFSM(sensors, hardware, config)

    # Populate all bookkeeping that RESET must clear
    fsm.state = AtlasFSM.RESET
    fsm.laser_active = True
    fsm._intercept = list(intercept)
    fsm._track_frames = 7
    fsm._detection_count = 5
    fsm._target = 0   # integer track_id (ADR-0006)
    fsm._acquire_entry_done = True

    return fsm, pan_motor, tilt_motor


def test_reset_transitions_to_search():
    """RESET must transition to SEARCH on its first (and only) step."""
    fsm, _, _ = _make_fsm_in_reset()
    fsm.step()
    assert fsm.state == AtlasFSM.SEARCH


def test_reset_clears_laser_active():
    """RESET must set laser_active to False before transitioning to SEARCH."""
    fsm, _, _ = _make_fsm_in_reset()
    fsm.step()
    assert fsm.laser_active is False


def test_reset_clears_intercept():
    """RESET must set _intercept to None."""
    fsm, _, _ = _make_fsm_in_reset()
    fsm.step()
    assert fsm._intercept is None


def test_reset_clears_target():
    """RESET must set _target to None."""
    fsm, _, _ = _make_fsm_in_reset()
    fsm.step()
    assert fsm._target is None


def test_reset_clears_acquire_entry_done():
    """RESET must set _acquire_entry_done to False so re-entry actions fire on next ACQUIRE."""
    fsm, _, _ = _make_fsm_in_reset()
    fsm.step()
    assert fsm._acquire_entry_done is False


def test_reset_detection_count_is_zero_after_reset():
    """_detection_count must be 0 after RESET→SEARCH (reset on SEARCH entry via _transition)."""
    fsm, _, _ = _make_fsm_in_reset()
    fsm.step()
    assert fsm._detection_count == 0


def test_reset_track_frames_is_zero_after_reset():
    """_track_frames must be 0 after RESET (reset on TRACK entry via _transition)."""
    fsm, _, _ = _make_fsm_in_reset()
    # _track_frames is reset when entering TRACK, not SEARCH; verify it is reset
    # on the RESET handler itself (not deferred to _transition) so that after
    # RESET→SEARCH the counter is clean for the next TRACK engagement.
    fsm.step()
    assert fsm._track_frames == 0


def test_laser_active_is_false_on_init():
    """laser_active must be False on FSM construction (before any ENGAGING step)."""
    fsm, _, _ = _make_fsm()
    assert fsm.laser_active is False
