"""Tests for AtlasFSM — finite state machine state handlers.

SEARCH and ACQUIRE are now driven by Search Radar *cues* arriving over a radio
link (SearchRadarLink), not by an in-process SearchRadar. A cue is a world-frame
position [x, y, z]; the FSM converts it to a turret-relative bearing using its
known turret world position.
"""
import math
import pytest
from stubs import StubFCR, StubCueLink, StubTrackFilter, StubMotor, StubGroundHitLink
from fsm import AtlasFSM, SensorSuite, TurretHardware, FSMConfig


# ---------------------------------------------------------------------------
# Helpers / fixtures
# ---------------------------------------------------------------------------

TURRET_POS = [0.0, 0.0, 0.0]


def _make_fsm(
    cue=None,
    acquire_frames=3,
    search_speed=0.1,
    search_pan_limit=1.0,
    track_filter=None,
    cue_link=None,
    fcr=None,
    turret_position=None,
):
    """Build an AtlasFSM wired to stubs.

    Args:
        cue: world-frame [x, y, z] cue returned by StubCueLink.get_cue().
            Defaults to None (no cue).
        acquire_frames: FSMConfig.acquire_frames — kept small for fast tests.
        search_speed: radians per step during pan sweep.
        search_pan_limit: pan sweep extent in radians.
        track_filter: inject a custom track_filter stub; defaults to
            StubTrackFilter with is_initialised() == True.
        cue_link: inject a custom cue link; defaults to StubCueLink(cue).
        fcr: inject a custom FCR stub; defaults to StubFCR(locked=True).
        turret_position: world-frame turret origin; defaults to TURRET_POS.

    Returns:
        (fsm, pan_motor, tilt_motor) tuple for inspection.
    """
    if track_filter is None:
        track_filter = StubTrackFilter(position=[1.0, 1.0, 1.0], velocity=[0.0, 0.0, 0.0])
    if cue_link is None:
        cue_link = StubCueLink(cue=cue)
    if fcr is None:
        fcr = StubFCR(position=[1.0, 1.0, 1.0])
    if turret_position is None:
        turret_position = TURRET_POS

    pan_motor = StubMotor()
    tilt_motor = StubMotor()
    sensors = SensorSuite(
        cue_link=cue_link,
        fcr=fcr,
        track_filter=track_filter,
        ballistic_predictor=None,
        ground_hit_link=StubGroundHitLink(),
    )
    hardware = TurretHardware(
        pan_motor=pan_motor,
        tilt_motor=tilt_motor,
        turret_position=turret_position,
        timestep_ms=32,
    )
    config = FSMConfig(
        acquire_frames=acquire_frames,
        search_speed=search_speed,
        search_pan_limit=search_pan_limit,
    )
    fsm = AtlasFSM(sensors, hardware, config)
    return fsm, pan_motor, tilt_motor


class CapturingTrackFilter(StubTrackFilter):
    """StubTrackFilter that records reset() calls and has a configurable init flag."""

    def __init__(self, position, velocity, initialised=True):
        super().__init__(position, velocity)
        self.reset_calls = 0
        self._initialised = initialised

    def reset(self):
        self.reset_calls += 1

    def is_initialised(self):
        return self._initialised


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
    for _ in range(6):
        fsm.step()
    assert pan.position == pytest.approx(-1.0)
    fsm.step()  # reverse back → -0.5
    assert pan.position == pytest.approx(-0.5)


# ---------------------------------------------------------------------------
# _do_search — cue counting and SEARCH→ACQUIRE transition
# ---------------------------------------------------------------------------

def test_search_no_cue_stays_in_search():
    """With no cue the FSM must remain in SEARCH indefinitely."""
    fsm, _, _ = _make_fsm(cue=None, acquire_frames=3)
    for _ in range(10):
        fsm.step()
    assert fsm.state == AtlasFSM.SEARCH


def test_search_consecutive_cues_transition_out_of_search():
    """After acquire_frames consecutive cue steps, the FSM must have left SEARCH.

    Uses initialised=False so the FSM stays in ACQUIRE (rather than falling
    through to TRACK) on the next step, making it easy to assert ACQUIRE.

    Step sequence (acquire_frames=3):
      Steps 1-3: SEARCH handler counts 3 consecutive cues → transitions to
                 ACQUIRE on step 3 (no ACQUIRE handler runs yet).
    """
    track_filter = CapturingTrackFilter(
        position=[1.0, 1.0, 1.0], velocity=[0.0, 0.0, 0.0], initialised=False
    )
    fsm, _, _ = _make_fsm(
        cue=[1.0, 2.0, 0.5], acquire_frames=3, track_filter=track_filter
    )
    for _ in range(3):
        fsm.step()
    assert fsm.state == AtlasFSM.ACQUIRE


def test_search_requires_consecutive_cues():
    """A no-cue frame resets the counter; acquire_frames must start over."""

    class AlternatingCueLink:
        """get_cue() returns a cue on calls 1, 2, 4, 5 and None on call 3."""

        def __init__(self):
            self._calls = 0

        def update(self):
            pass

        def get_cue(self):
            self._calls += 1
            if self._calls in (1, 2, 4, 5):
                return [1.0, 2.0, 0.5]
            return None

    fsm, _, _ = _make_fsm(cue_link=AlternatingCueLink(), acquire_frames=3)

    fsm.step()  # cue (count=1)
    assert fsm.state == AtlasFSM.SEARCH
    fsm.step()  # cue (count=2)
    assert fsm.state == AtlasFSM.SEARCH
    fsm.step()  # blank → reset (count=0)
    assert fsm.state == AtlasFSM.SEARCH
    fsm.step()  # cue (count=1) — must NOT have transitioned
    assert fsm.state == AtlasFSM.SEARCH
    fsm.step()  # cue (count=2) — still not at acquire_frames=3
    assert fsm.state == AtlasFSM.SEARCH


def test_search_single_frame_acquire():
    """acquire_frames=1 means a single cue step triggers ACQUIRE."""
    track_filter = CapturingTrackFilter(
        position=[1.0, 1.0, 1.0], velocity=[0.0, 0.0, 0.0], initialised=False
    )
    fsm, _, _ = _make_fsm(
        cue=[1.0, 2.0, 0.5], acquire_frames=1, track_filter=track_filter
    )
    fsm.step()  # SEARCH → ACQUIRE (transition only; ACQUIRE handler not yet run)
    assert fsm.state == AtlasFSM.ACQUIRE
    fsm.step()  # ACQUIRE handler runs; filter not initialised → stays
    assert fsm.state == AtlasFSM.ACQUIRE


def test_search_stores_cue_world_position_as_target():
    """On transitioning to ACQUIRE, fsm._target must be the world-position cue.

    The cue is a world-frame [x, y, z] list — no longer an integer track_id.
    """
    track_filter = CapturingTrackFilter(
        position=[1.0, 1.0, 1.0], velocity=[0.0, 0.0, 0.0], initialised=False
    )
    cue = [1.0, 2.0, 0.5]
    fsm, _, _ = _make_fsm(cue=cue, acquire_frames=1, track_filter=track_filter)
    fsm.step()  # SEARCH → ACQUIRE
    assert fsm._target == cue


# ---------------------------------------------------------------------------
# _do_acquire — entry actions, motor aiming, and ACQUIRE→TRACK transition
# ---------------------------------------------------------------------------

def test_acquire_resets_track_filter_on_entry():
    """On entering ACQUIRE, track_filter.reset() must be called exactly once."""
    track_filter = CapturingTrackFilter(
        position=[1.0, 1.0, 1.0], velocity=[0.0, 0.0, 0.0], initialised=False
    )
    fsm, _, _ = _make_fsm(
        cue=[1.0, 2.0, 0.5], acquire_frames=1, track_filter=track_filter
    )
    fsm.step()  # SEARCH → ACQUIRE (transition only)
    fsm.step()  # ACQUIRE handler: entry actions fire; filter not init → stays
    fsm.step()  # ACQUIRE handler again: reset must NOT fire again
    assert track_filter.reset_calls == 1


def test_acquire_aims_pan_motor_at_cue_bearing():
    """ACQUIRE must command the pan motor toward the cue's turret-relative bearing.

    Cue [1.0, 2.0, 0.5] with turret at origin → relative [1.0, 2.0, 0.5],
    pan = atan2(1.0, 2.0).
    """
    track_filter = CapturingTrackFilter(
        position=[1.0, 1.0, 1.0], velocity=[0.0, 0.0, 0.0], initialised=False
    )
    cue = [1.0, 2.0, 0.5]
    fsm, pan, _ = _make_fsm(cue=cue, acquire_frames=1, track_filter=track_filter)
    fsm.step()  # SEARCH → ACQUIRE
    fsm.step()  # ACQUIRE handler: aim at cue
    assert pan.position == pytest.approx(math.atan2(1.0, 2.0))


def test_acquire_aims_tilt_motor_at_cue_bearing():
    """ACQUIRE must command the tilt motor toward the cue's turret-relative bearing.

    Cue [1.0, 2.0, 0.5] → tilt = atan2(0.5, sqrt(1.0² + 2.0²)).
    """
    track_filter = CapturingTrackFilter(
        position=[1.0, 1.0, 1.0], velocity=[0.0, 0.0, 0.0], initialised=False
    )
    cue = [1.0, 2.0, 0.5]
    fsm, _, tilt = _make_fsm(cue=cue, acquire_frames=1, track_filter=track_filter)
    fsm.step()  # SEARCH → ACQUIRE
    fsm.step()  # ACQUIRE handler: aim at cue
    expected_tilt = math.atan2(0.5, math.sqrt(1.0 ** 2 + 2.0 ** 2))
    assert tilt.position == pytest.approx(expected_tilt)


def test_acquire_aims_relative_to_turret_world_position():
    """ACQUIRE must subtract the turret world position from the world-frame cue.

    Turret at [1.0, 1.0, 0.0], cue at [2.0, 3.0, 0.5] → relative [1.0, 2.0, 0.5],
    pan = atan2(1.0, 2.0).
    """
    track_filter = CapturingTrackFilter(
        position=[1.0, 1.0, 1.0], velocity=[0.0, 0.0, 0.0], initialised=False
    )
    fsm, pan, _ = _make_fsm(
        cue=[2.0, 3.0, 0.5],
        acquire_frames=1,
        track_filter=track_filter,
        turret_position=[1.0, 1.0, 0.0],
    )
    fsm.step()  # SEARCH → ACQUIRE
    fsm.step()  # ACQUIRE handler: aim at cue
    assert pan.position == pytest.approx(math.atan2(1.0, 2.0))


def test_acquire_does_not_error_when_cue_briefly_none():
    """ACQUIRE must not raise when get_cue() briefly returns None.

    The cue link goes silent for one step; ACQUIRE holds its last aim and
    keeps running without error.
    """
    cue_link = StubCueLink(cue=[1.0, 2.0, 0.5])
    track_filter = CapturingTrackFilter(
        position=[1.0, 1.0, 1.0], velocity=[0.0, 0.0, 0.0], initialised=False
    )
    fsm, pan, _ = _make_fsm(
        acquire_frames=1, cue_link=cue_link, track_filter=track_filter
    )
    fsm.step()  # SEARCH → ACQUIRE
    fsm.step()  # ACQUIRE handler: aim at cue
    aimed = pan.position
    cue_link.cue = None
    fsm.step()  # ACQUIRE handler: cue is None → no error, aim held
    assert fsm.state == AtlasFSM.ACQUIRE
    assert pan.position == pytest.approx(aimed)


def test_acquire_transitions_to_track_when_locked_and_initialised():
    """ACQUIRE→TRACK fires only when fcr.is_locked() AND track_filter.is_initialised()."""
    track_filter = CapturingTrackFilter(
        position=[1.0, 1.0, 1.0], velocity=[0.0, 0.0, 0.0], initialised=True
    )
    fcr = StubFCR(position=[1.0, 1.0, 1.0], locked=True)
    fsm, _, _ = _make_fsm(
        cue=[1.0, 2.0, 0.5], acquire_frames=1, track_filter=track_filter, fcr=fcr
    )
    fsm.step()  # SEARCH → ACQUIRE
    fsm.step()  # ACQUIRE handler: locked AND initialised → TRACK
    assert fsm.state == AtlasFSM.TRACK


def test_acquire_stays_when_filter_not_initialised():
    """ACQUIRE must remain in ACQUIRE while the filter is not initialised."""
    track_filter = CapturingTrackFilter(
        position=[1.0, 1.0, 1.0], velocity=[0.0, 0.0, 0.0], initialised=False
    )
    fcr = StubFCR(position=[1.0, 1.0, 1.0], locked=True)
    fsm, _, _ = _make_fsm(
        cue=[1.0, 2.0, 0.5], acquire_frames=1, track_filter=track_filter, fcr=fcr
    )
    fsm.step()  # SEARCH → ACQUIRE
    fsm.step()  # ACQUIRE handler: locked but not initialised → stays
    assert fsm.state == AtlasFSM.ACQUIRE


def test_acquire_stays_when_fcr_not_locked():
    """ACQUIRE must remain in ACQUIRE while the FCR is not locked."""
    track_filter = CapturingTrackFilter(
        position=[1.0, 1.0, 1.0], velocity=[0.0, 0.0, 0.0], initialised=True
    )
    fcr = StubFCR(position=[1.0, 1.0, 1.0], locked=False)
    fsm, _, _ = _make_fsm(
        cue=[1.0, 2.0, 0.5], acquire_frames=1, track_filter=track_filter, fcr=fcr
    )
    fsm.step()  # SEARCH → ACQUIRE
    fsm.step()  # ACQUIRE handler: initialised but not locked → stays
    assert fsm.state == AtlasFSM.ACQUIRE


# ---------------------------------------------------------------------------
# commanded_aim property
# ---------------------------------------------------------------------------

def test_commanded_aim_reports_last_commanded_angles():
    """commanded_aim must report the (pan, tilt) last sent to the motors.

    After an ACQUIRE step aiming at cue [1.0, 2.0, 0.5], commanded_aim must
    match the pan/tilt computed from that cue.
    """
    track_filter = CapturingTrackFilter(
        position=[1.0, 1.0, 1.0], velocity=[0.0, 0.0, 0.0], initialised=False
    )
    fsm, _, _ = _make_fsm(
        cue=[1.0, 2.0, 0.5], acquire_frames=1, track_filter=track_filter
    )
    fsm.step()  # SEARCH → ACQUIRE
    fsm.step()  # ACQUIRE handler: aim at cue
    pan, tilt = fsm.commanded_aim
    assert pan == pytest.approx(math.atan2(1.0, 2.0))
    assert tilt == pytest.approx(math.atan2(0.5, math.sqrt(1.0 ** 2 + 2.0 ** 2)))


def test_commanded_aim_initial_value_is_zero():
    """commanded_aim must be (0.0, 0.0) before any motor command is issued."""
    fsm, _, _ = _make_fsm()
    assert fsm.commanded_aim == (0.0, 0.0)


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
# _target_within_range (range-only; no ground reasoning lives in the FSM)
# ---------------------------------------------------------------------------

def test_target_within_range_within_bounds():
    """A nearby target must be in range."""
    fsm, _, _ = _make_fsm()
    assert fsm._target_within_range([1.0, 1.0, 1.0]) is True


def test_target_within_range_too_far():
    """A target beyond max_range must not be in range."""
    fsm, _, _ = _make_fsm()
    assert fsm._target_within_range([10.0, 10.0, 10.0]) is False


def test_target_within_range_ignores_ground():
    """A target at the floor (dz=0.0) but within max_range IS in range:
    the FSM no longer treats low Z as out-of-range — ground hits come only
    from the attacker's emitted cue."""
    fsm, _, _ = _make_fsm()
    assert fsm._target_within_range([0.5, 0.5, 0.0]) is True


def test_target_within_range_exactly_at_max_range():
    """A target exactly at max_range=10.0 on one axis must be in range (≤ check)."""
    fsm, _, _ = _make_fsm()
    assert fsm._target_within_range([0.0, 0.0, 10.0]) is True


# ---------------------------------------------------------------------------
# _do_track — helpers
# ---------------------------------------------------------------------------

def _make_fsm_in_track(
    position=None,
    min_track_frames=5,
):
    """Build an AtlasFSM already in TRACK state with a fixed filter position.

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
        cue_link=StubCueLink(),
        fcr=StubFCR(position=[1.0, 1.0, 1.0]),
        track_filter=track_filter,
        ballistic_predictor=None,
        ground_hit_link=StubGroundHitLink(),
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
    """TRACK must transition to PREDICT on the step that reaches min_track_frames."""
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
        (fsm, pan_motor, tilt_motor) tuple.
    """
    predictor = StubPredictor(intercept)
    track_filter = StubTrackFilter(position=[1.0, 1.0, 1.0], velocity=[0.0, 0.0, 0.0])
    pan_motor = StubMotor()
    tilt_motor = StubMotor()
    sensors = SensorSuite(
        cue_link=StubCueLink(),
        fcr=StubFCR(position=[1.0, 1.0, 1.0]),
        track_filter=track_filter,
        ballistic_predictor=predictor,
        ground_hit_link=StubGroundHitLink(),
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
    """PREDICT with an out-of-range intercept must transition back to TRACK."""
    fsm, _, _ = _make_fsm_in_predict(intercept=[8.0, 8.0, 8.0], max_range=10.0)
    fsm.step()
    assert fsm.state == AtlasFSM.TRACK


def test_predict_invalid_intercept_resets_track_bookkeeping():
    """PREDICT→TRACK (invalid intercept) must reset _track_frames to 0 AND clear _intercept."""
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

    Args:
        intercept:           Relative [dx, dy, dz] stored as self._intercept.
                             Defaults to [1.0, 2.0, 1.0].
        aim_error_threshold: FSMConfig.aim_error_threshold (radians).
        track_position:      Position StubTrackFilter returns. Defaults to
                             [1.0, 2.0, 1.0].

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
        cue_link=StubCueLink(),
        fcr=StubFCR(position=[1.0, 1.0, 1.0]),
        track_filter=track_filter,
        ballistic_predictor=None,
        ground_hit_link=StubGroundHitLink(),
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
    """AIMING must command the pan motor to the pan angle for the intercept."""
    intercept = [1.0, 2.0, 1.0]
    fsm, pan, _ = _make_fsm_in_aiming(intercept=intercept)
    fsm.step()
    expected_pan = math.atan2(intercept[0], intercept[1])
    assert pan.position == pytest.approx(expected_pan)


def test_aim_commands_tilt_motor_at_intercept():
    """AIMING must command the tilt motor to the tilt angle for the intercept."""
    intercept = [1.0, 2.0, 1.0]
    fsm, _, tilt = _make_fsm_in_aiming(intercept=intercept)
    fsm.step()
    expected_tilt = math.atan2(intercept[2], math.sqrt(intercept[0]**2 + intercept[1]**2))
    assert tilt.position == pytest.approx(expected_tilt)


# ---------------------------------------------------------------------------
# _do_aim — convergence and AIMING→ENGAGING transition
# ---------------------------------------------------------------------------

def test_aim_stays_in_aiming_before_convergence():
    """AIMING must remain in AIMING on the first step (aim error is nonzero on entry)."""
    intercept = [1.0, 2.0, 1.0]
    fsm, _, _ = _make_fsm_in_aiming(
        intercept=intercept,
        aim_error_threshold=0.05,
    )
    fsm.step()  # commands motors; error measured vs prior commanded → nonzero → stays
    assert fsm.state == AtlasFSM.AIMING


def test_aim_transitions_to_engaging_after_convergence():
    """AIMING must transition to ENGAGING once aim error falls below threshold."""
    intercept = [1.0, 2.0, 1.0]
    fsm, _, _ = _make_fsm_in_aiming(
        intercept=intercept,
        aim_error_threshold=0.05,
    )
    fsm.step()  # step 1: commands motors, error nonzero → stays in AIMING
    fsm.step()  # step 2: error == 0 < threshold → transitions to ENGAGING
    assert fsm.state == AtlasFSM.ENGAGING


def test_aim_already_aligned_transitions_immediately():
    """AIMING with a very large threshold must transition to ENGAGING on the first step."""
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
    ground_hit=False,
):
    """Build an AtlasFSM already in ENGAGING state.

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
        cue_link=StubCueLink(),
        fcr=StubFCR(position=[1.0, 1.0, 1.0]),
        track_filter=track_filter,
        ballistic_predictor=None,
        ground_hit_link=StubGroundHitLink(hit=ground_hit),
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
    fsm, _, _ = _make_fsm_in_engaging(track_position=[1.0, 1.0, 1.0])
    fsm.step()
    assert fsm.state == AtlasFSM.ENGAGING


def test_engage_transitions_to_reset_when_target_out_of_range():
    """ENGAGING must transition to RESET when the target goes out of range."""
    fsm, _, _ = _make_fsm_in_engaging(track_position=[9.0, 9.0, 9.0], max_range=10.0)
    fsm.step()
    assert fsm.state == AtlasFSM.RESET


def test_engage_transitions_to_reset_on_ground_hit_cue():
    """ENGAGING must transition to RESET when the attacker's ground-hit cue fires,
    even though the target is still in range and above ground."""
    fsm, _, _ = _make_fsm_in_engaging(
        track_position=[1.0, 2.0, 1.0],  # in range, above ground
        ground_hit=True,
    )
    fsm.step()
    assert fsm.state == AtlasFSM.RESET


def test_engage_stays_engaged_for_low_target_without_cue():
    """ENGAGING must NOT exit on a low/descending target by itself: ground hits
    come only from the emitted cue, never from the track Z estimate."""
    fsm, _, _ = _make_fsm_in_engaging(
        track_position=[1.0, 1.0, 0.0],  # at the floor, but no cue
        ground_hit=False,
    )
    fsm.step()
    assert fsm.state == AtlasFSM.ENGAGING


# ---------------------------------------------------------------------------
# _do_reset — state clearing and RESET→SEARCH transition
# ---------------------------------------------------------------------------

def _make_fsm_in_reset(intercept=None):
    """Build an AtlasFSM in RESET state with engagement bookkeeping populated.

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
        cue_link=StubCueLink(),
        fcr=StubFCR(position=[1.0, 1.0, 1.0]),
        track_filter=track_filter,
        ballistic_predictor=None,
        ground_hit_link=StubGroundHitLink(),
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
    fsm._target = [1.0, 2.0, 0.5]   # world-position cue
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
    """_track_frames must be 0 after RESET (reset on the RESET handler itself)."""
    fsm, _, _ = _make_fsm_in_reset()
    fsm.step()
    assert fsm._track_frames == 0


def test_laser_active_is_false_on_init():
    """laser_active must be False on FSM construction (before any ENGAGING step)."""
    fsm, _, _ = _make_fsm()
    assert fsm.laser_active is False
