"""Tests for AtlasFSM — 5-state finite state machine.

States: IDLE → AIM → TRACK_PREDICT → ENGAGING → RESET → IDLE.

IDLE and AIM are driven by Search Radar *cues* arriving over a radio link
(SearchRadarLink). A cue is a world-frame position [x, y, z]; the FSM converts
it to a turret-relative bearing using its known turret world position.
"""
import math
import pytest
from stubs import StubFCR, StubCueLink, StubTrackFilter, StubMotor, StubGroundHitLink
from fsm import AtlasFSM, SensorSuite, TurretHardware, FSMConfig


# ---------------------------------------------------------------------------
# Helpers / fixtures
# ---------------------------------------------------------------------------

TURRET_POS = [0.0, 0.0, 0.0]


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


class CapturingTrackFilter(StubTrackFilter):
    """StubTrackFilter that records reset() calls."""

    def __init__(self, position, velocity):
        super().__init__(position, velocity)
        self.reset_calls = 0

    def reset(self):
        self.reset_calls += 1


def _build_fsm(
    *,
    cue=None,
    cue_link=None,
    fcr=None,
    track_filter=None,
    ballistic_predictor=None,
    ground_hit_link=None,
    turret_position=None,
    config=None,
):
    """Build an AtlasFSM wired to stubs, returning (fsm, pan_motor, tilt_motor)."""
    if track_filter is None:
        track_filter = StubTrackFilter(position=[1.0, 1.0, 1.0], velocity=[0.0, 0.0, 0.0])
    if cue_link is None:
        cue_link = StubCueLink(cue=cue)
    if fcr is None:
        fcr = StubFCR(position=[1.0, 1.0, 1.0])
    if ground_hit_link is None:
        ground_hit_link = StubGroundHitLink()
    if turret_position is None:
        turret_position = TURRET_POS

    pan_motor = StubMotor()
    tilt_motor = StubMotor()
    sensors = SensorSuite(
        cue_link=cue_link,
        fcr=fcr,
        track_filter=track_filter,
        ballistic_predictor=ballistic_predictor,
        ground_hit_link=ground_hit_link,
    )
    hardware = TurretHardware(
        pan_motor=pan_motor,
        tilt_motor=tilt_motor,
        turret_position=turret_position,
        timestep_ms=32,
    )
    fsm = AtlasFSM(sensors, hardware, config if config is not None else FSMConfig())
    return fsm, pan_motor, tilt_motor


# ---------------------------------------------------------------------------
# Initial state
# ---------------------------------------------------------------------------

def test_initial_state_is_idle():
    """FSM must start in the IDLE state."""
    fsm, _, _ = _build_fsm()
    assert fsm.state == AtlasFSM.IDLE


# ---------------------------------------------------------------------------
# IDLE — fixed beam direction (no pan sweep)
# ---------------------------------------------------------------------------

def test_idle_holds_fixed_pan_direction():
    """IDLE must command the pan motor to the configured idle_pan and hold it."""
    config = FSMConfig(idle_pan=0.5, idle_tilt=0.2)
    fsm, pan, tilt = _build_fsm(config=config)
    for _ in range(5):
        fsm.step()
    assert pan.position == pytest.approx(0.5)
    assert tilt.position == pytest.approx(0.2)


def test_idle_does_not_sweep():
    """IDLE must not move the pan motor between steps (no sweep)."""
    config = FSMConfig(idle_pan=0.3, idle_tilt=0.0)
    fsm, pan, _ = _build_fsm(config=config)
    fsm.step()
    first = pan.position
    fsm.step()
    assert pan.position == pytest.approx(first)


# ---------------------------------------------------------------------------
# IDLE — cue counting and IDLE→AIM transition
# ---------------------------------------------------------------------------

def test_idle_no_cue_stays_in_idle():
    """With no cue the FSM must remain in IDLE indefinitely."""
    fsm, _, _ = _build_fsm(cue=None, config=FSMConfig(acquire_frames=3))
    for _ in range(10):
        fsm.step()
    assert fsm.state == AtlasFSM.IDLE


def test_idle_consecutive_cues_transition_to_aim():
    """After acquire_frames consecutive cue steps, the FSM must enter AIM."""
    fsm, _, _ = _build_fsm(cue=[1.0, 2.0, 0.5], config=FSMConfig(acquire_frames=3))
    for _ in range(3):
        fsm.step()
    assert fsm.state == AtlasFSM.AIM


def test_idle_requires_consecutive_cues():
    """A no-cue frame resets the counter; acquire_frames must start over."""

    class AlternatingCueLink:
        def __init__(self):
            self._calls = 0

        def update(self):
            pass

        def get_cue(self):
            self._calls += 1
            if self._calls in (1, 2, 4, 5):
                return [1.0, 2.0, 0.5]
            return None

    fsm, _, _ = _build_fsm(cue_link=AlternatingCueLink(), config=FSMConfig(acquire_frames=3))
    for _ in range(5):
        fsm.step()
        assert fsm.state == AtlasFSM.IDLE  # never reaches 3 consecutive


def test_idle_stores_cue_world_position_as_target():
    """On transitioning to AIM, fsm._target must be the world-position cue."""
    cue = [1.0, 2.0, 0.5]
    fsm, _, _ = _build_fsm(cue=cue, config=FSMConfig(acquire_frames=1))
    fsm.step()
    assert fsm._target == cue


def test_idle_ground_hit_transitions_to_reset():
    """A ground-hit cue during IDLE must send the FSM to RESET."""
    fsm, _, _ = _build_fsm(ground_hit_link=StubGroundHitLink(hit=True))
    fsm.step()
    assert fsm.state == AtlasFSM.RESET


# ---------------------------------------------------------------------------
# AIM — entry reset, motor aiming, AIM→TRACK_PREDICT transition
# ---------------------------------------------------------------------------

def _build_fsm_in_aim(cue=None, fcr=None, track_filter=None, ground_hit_link=None):
    """Build an AtlasFSM placed directly in AIM with the given target cue."""
    if cue is None:
        cue = [1.0, 2.0, 0.5]
    if track_filter is None:
        track_filter = CapturingTrackFilter(position=[1.0, 1.0, 1.0], velocity=[0.0, 0.0, 0.0])
    fsm, pan, tilt = _build_fsm(
        cue=cue, fcr=fcr, track_filter=track_filter, ground_hit_link=ground_hit_link
    )
    fsm.state = AtlasFSM.AIM
    fsm._target = cue
    return fsm, pan, tilt, track_filter


def test_aim_resets_track_filter_once_on_entry():
    """AIM must call track_filter.reset() exactly once on entry."""
    fcr = StubFCR(position=[1.0, 1.0, 1.0], locked=False)  # stay in AIM
    fsm, _, _, track_filter = _build_fsm_in_aim(fcr=fcr)
    fsm.step()
    fsm.step()
    fsm.step()
    assert track_filter.reset_calls == 1


def test_aim_aims_pan_motor_at_cue_bearing():
    """AIM must command the pan motor toward the cue's turret-relative bearing.

    Cue [1.0, 2.0, 0.5] with turret at origin → pan = atan2(1.0, 2.0).
    """
    fcr = StubFCR(position=[1.0, 1.0, 1.0], locked=False)
    fsm, pan, _, _ = _build_fsm_in_aim(cue=[1.0, 2.0, 0.5], fcr=fcr)
    fsm.step()
    assert pan.position == pytest.approx(math.atan2(1.0, 2.0))


def test_aim_aims_tilt_motor_at_cue_bearing():
    """AIM must command the tilt motor toward the cue's turret-relative bearing.

    Cue [1.0, 2.0, 0.5] → tilt = atan2(0.5, sqrt(1² + 2²)).
    """
    fcr = StubFCR(position=[1.0, 1.0, 1.0], locked=False)
    fsm, _, tilt, _ = _build_fsm_in_aim(cue=[1.0, 2.0, 0.5], fcr=fcr)
    fsm.step()
    assert tilt.position == pytest.approx(math.atan2(0.5, math.sqrt(1.0 ** 2 + 2.0 ** 2)))


def test_aim_does_not_error_when_cue_briefly_none():
    """AIM must not raise (and must hold aim) when get_cue() briefly returns None."""
    cue_link = StubCueLink(cue=[1.0, 2.0, 0.5])
    fcr = StubFCR(position=[1.0, 1.0, 1.0], locked=False)
    track_filter = CapturingTrackFilter(position=[1.0, 1.0, 1.0], velocity=[0.0, 0.0, 0.0])
    fsm, pan, _ = _build_fsm(cue_link=cue_link, fcr=fcr, track_filter=track_filter)
    fsm.state = AtlasFSM.AIM
    fsm._target = [1.0, 2.0, 0.5]
    fsm.step()
    aimed = pan.position
    cue_link.cue = None
    fsm.step()
    assert fsm.state == AtlasFSM.AIM
    assert pan.position == pytest.approx(aimed)


def test_aim_transitions_to_track_predict_when_fcr_locked():
    """AIM→TRACK_PREDICT fires when fcr.is_locked() (the FCR finds the target)."""
    fcr = StubFCR(position=[1.0, 1.0, 1.0], locked=True)
    fsm, _, _, _ = _build_fsm_in_aim(fcr=fcr)
    fsm.step()
    assert fsm.state == AtlasFSM.TRACK_PREDICT


def test_aim_stays_while_fcr_not_locked():
    """AIM must remain in AIM while the FCR is not locked."""
    fcr = StubFCR(position=[1.0, 1.0, 1.0], locked=False)
    fsm, _, _, _ = _build_fsm_in_aim(fcr=fcr)
    fsm.step()
    assert fsm.state == AtlasFSM.AIM


def test_aim_ground_hit_transitions_to_reset():
    """A ground-hit cue during AIM must send the FSM to RESET."""
    fsm, _, _, _ = _build_fsm_in_aim(ground_hit_link=StubGroundHitLink(hit=True))
    fsm.step()
    assert fsm.state == AtlasFSM.RESET


# ---------------------------------------------------------------------------
# TRACK_PREDICT — fusion, aiming, convergence → ENGAGING
# ---------------------------------------------------------------------------

def _build_fsm_in_track_predict(
    *,
    position=None,
    intercept=None,
    lookahead_steps=1,
    converge_frames=1,
    track_error_threshold=0.3,
    max_range=10.0,
    cue=None,
    ground_hit_link=None,
):
    """Build an AtlasFSM placed directly in TRACK_PREDICT.

    With lookahead_steps=1 the rolling history (maxlen=2) first yields a
    prediction error on the 3rd step. Defaults make intercept == filtered
    position so the error is 0 (below threshold) and within range.
    """
    if position is None:
        position = [2.0, 2.0, 2.0]
    if intercept is None:
        intercept = list(position)

    track_filter = StubTrackFilter(position=position, velocity=[0.0, 0.0, 0.0])
    predictor = StubPredictor(intercept)
    config = FSMConfig(
        lookahead_steps=lookahead_steps,
        converge_frames=converge_frames,
        track_error_threshold=track_error_threshold,
        max_range=max_range,
    )
    fsm, pan, tilt = _build_fsm(
        cue=cue,
        track_filter=track_filter,
        ballistic_predictor=predictor,
        ground_hit_link=ground_hit_link,
        config=config,
    )
    fsm.state = AtlasFSM.TRACK_PREDICT
    return fsm, pan, tilt, track_filter


def test_track_predict_aims_at_filtered_position():
    """TRACK_PREDICT must command the motors at the filtered position bearing."""
    position = [1.0, 2.0, 3.0]
    fsm, pan, tilt, _ = _build_fsm_in_track_predict(position=position, intercept=position)
    fsm.step()
    assert pan.position == pytest.approx(math.atan2(position[0], position[1]))
    assert tilt.position == pytest.approx(
        math.atan2(position[2], math.sqrt(position[0] ** 2 + position[1] ** 2))
    )


def test_track_predict_fuses_search_cue():
    """TRACK_PREDICT must fuse a present Search Radar cue via update_search()."""

    class RecordingFilter(StubTrackFilter):
        def __init__(self):
            super().__init__(position=[2.0, 2.0, 2.0], velocity=[0.0, 0.0, 0.0])
            self.search_updates = []

        def update_search(self, m):
            self.search_updates.append(list(m))

    rf = RecordingFilter()
    predictor = StubPredictor([2.0, 2.0, 2.0])
    config = FSMConfig(lookahead_steps=1, converge_frames=1)
    fsm, _, _ = _build_fsm(
        cue=[3.0, 4.0, 5.0],
        track_filter=rf,
        ballistic_predictor=predictor,
        turret_position=[1.0, 1.0, 1.0],
        config=config,
    )
    fsm.state = AtlasFSM.TRACK_PREDICT
    fsm.step()
    # World cue [3,4,5] minus turret [1,1,1] → relative [2,3,4].
    assert rf.search_updates == [[2.0, 3.0, 4.0]]


def test_track_predict_converges_to_engaging():
    """A stable, in-range prediction must transition to ENGAGING once the
    prediction error stays below threshold for converge_frames steps.

    lookahead_steps=1 → error first available on step 3; converge_frames=1 →
    transition on step 3.
    """
    fsm, _, _, _ = _build_fsm_in_track_predict(lookahead_steps=1, converge_frames=1)
    fsm.step()  # step 1 — warming up
    assert fsm.state == AtlasFSM.TRACK_PREDICT
    fsm.step()  # step 2 — warming up
    assert fsm.state == AtlasFSM.TRACK_PREDICT
    fsm.step()  # step 3 — error 0 < threshold → ENGAGING
    assert fsm.state == AtlasFSM.ENGAGING


def test_track_predict_stores_intercept_on_engaging():
    """The intercept fired on must be stored when transitioning to ENGAGING."""
    intercept = [2.0, 2.0, 2.0]
    fsm, _, _, _ = _build_fsm_in_track_predict(
        position=intercept, intercept=intercept, lookahead_steps=1, converge_frames=1
    )
    for _ in range(3):
        fsm.step()
    assert fsm.state == AtlasFSM.ENGAGING
    assert fsm._intercept == intercept


def test_track_predict_requires_converge_frames():
    """With converge_frames=2 the FSM needs two consecutive sub-threshold steps."""
    fsm, _, _, _ = _build_fsm_in_track_predict(lookahead_steps=1, converge_frames=2)
    fsm.step()  # 1 — warming
    fsm.step()  # 2 — warming
    fsm.step()  # 3 — first sub-threshold (count 1)
    assert fsm.state == AtlasFSM.TRACK_PREDICT
    fsm.step()  # 4 — second sub-threshold (count 2) → ENGAGING
    assert fsm.state == AtlasFSM.ENGAGING


def test_track_predict_out_of_range_does_not_engage():
    """An out-of-range intercept must never trigger ENGAGING, even if stable."""
    far = [8.0, 8.0, 8.0]  # |.| ≈ 13.9 > max_range 10
    fsm, _, _, _ = _build_fsm_in_track_predict(
        position=far, intercept=far, lookahead_steps=1, converge_frames=1, max_range=10.0
    )
    for _ in range(6):
        fsm.step()
    assert fsm.state == AtlasFSM.TRACK_PREDICT


def test_track_predict_high_error_does_not_engage():
    """A prediction whose intercept drifts far from the estimate must not engage."""
    fsm, _, _, _ = _build_fsm_in_track_predict(
        position=[2.0, 2.0, 2.0],
        intercept=[2.0, 2.0, 5.0],  # error 3.0 m > threshold 0.3
        lookahead_steps=1,
        converge_frames=1,
        track_error_threshold=0.3,
    )
    for _ in range(6):
        fsm.step()
    assert fsm.state == AtlasFSM.TRACK_PREDICT


def test_track_predict_ground_hit_transitions_to_reset():
    """A ground-hit cue during TRACK_PREDICT must send the FSM to RESET."""
    fsm, _, _, _ = _build_fsm_in_track_predict(ground_hit_link=StubGroundHitLink(hit=True))
    fsm.step()
    assert fsm.state == AtlasFSM.RESET


# ---------------------------------------------------------------------------
# ENGAGING — holds aim, exit conditions
# ---------------------------------------------------------------------------

def _build_fsm_in_engaging(
    *, intercept=None, track_position=None, max_range=10.0, ground_hit=False
):
    """Build an AtlasFSM placed directly in ENGAGING with a stored intercept."""
    if intercept is None:
        intercept = [1.0, 2.0, 1.0]
    if track_position is None:
        track_position = list(intercept)

    track_filter = StubTrackFilter(position=track_position, velocity=[0.0, 0.0, 0.0])
    config = FSMConfig(max_range=max_range)
    fsm, pan, tilt = _build_fsm(
        track_filter=track_filter,
        ground_hit_link=StubGroundHitLink(hit=ground_hit),
        config=config,
    )
    fsm.state = AtlasFSM.ENGAGING
    fsm._intercept = list(intercept)
    return fsm, pan, tilt


def test_engage_holds_aim_at_intercept():
    """ENGAGING must keep commanding the motors at the stored intercept bearing."""
    intercept = [1.0, 2.0, 1.0]
    fsm, pan, tilt = _build_fsm_in_engaging(intercept=intercept)
    fsm.step()
    assert pan.position == pytest.approx(math.atan2(intercept[0], intercept[1]))
    assert tilt.position == pytest.approx(
        math.atan2(intercept[2], math.sqrt(intercept[0] ** 2 + intercept[1] ** 2))
    )


def test_engage_stays_while_target_in_range():
    """ENGAGING must remain while the target is within range and no cue fires."""
    fsm, _, _ = _build_fsm_in_engaging(track_position=[1.0, 1.0, 1.0])
    fsm.step()
    assert fsm.state == AtlasFSM.ENGAGING


def test_engage_transitions_to_reset_when_out_of_range():
    """ENGAGING must transition to RESET when the target goes out of range."""
    fsm, _, _ = _build_fsm_in_engaging(track_position=[9.0, 9.0, 9.0], max_range=10.0)
    fsm.step()
    assert fsm.state == AtlasFSM.RESET


def test_engage_transitions_to_reset_on_ground_hit_cue():
    """ENGAGING must transition to RESET on the ground-hit cue even while in range."""
    fsm, _, _ = _build_fsm_in_engaging(track_position=[1.0, 2.0, 1.0], ground_hit=True)
    fsm.step()
    assert fsm.state == AtlasFSM.RESET


def test_engage_stays_for_low_target_without_cue():
    """ENGAGING must NOT exit on a low target by itself: ground hits come only
    from the emitted cue, never from the track Z estimate."""
    fsm, _, _ = _build_fsm_in_engaging(track_position=[1.0, 1.0, 0.0], ground_hit=False)
    fsm.step()
    assert fsm.state == AtlasFSM.ENGAGING


# ---------------------------------------------------------------------------
# RESET — wipes state, returns to IDLE
# ---------------------------------------------------------------------------

def _build_fsm_in_reset():
    """Build an AtlasFSM in RESET with engagement bookkeeping populated."""
    track_filter = CapturingTrackFilter(position=[1.0, 2.0, 1.0], velocity=[0.0, 0.0, 0.0])
    fsm, pan, tilt = _build_fsm(track_filter=track_filter)
    fsm.state = AtlasFSM.RESET
    fsm._intercept = [1.0, 2.0, 1.0]
    fsm._detection_count = 5
    fsm._converge_count = 4
    fsm._target = [1.0, 2.0, 0.5]
    fsm._aim_entry_done = True
    fsm._pred_history.append([1.0, 2.0, 1.0])
    return fsm, track_filter


def test_reset_transitions_to_idle():
    """RESET must transition to IDLE on its first (and only) step."""
    fsm, _ = _build_fsm_in_reset()
    fsm.step()
    assert fsm.state == AtlasFSM.IDLE


def test_reset_wipes_track_filter():
    """RESET must wipe the Kalman filter via track_filter.reset()."""
    fsm, track_filter = _build_fsm_in_reset()
    fsm.step()
    assert track_filter.reset_calls == 1


def test_reset_clears_intercept_and_target():
    """RESET must clear _intercept and _target."""
    fsm, _ = _build_fsm_in_reset()
    fsm.step()
    assert fsm._intercept is None
    assert fsm._target is None


def test_reset_clears_counters_and_history():
    """RESET must clear convergence/detection counters and prediction history."""
    fsm, _ = _build_fsm_in_reset()
    fsm.step()
    assert fsm._converge_count == 0
    assert fsm._detection_count == 0
    assert len(fsm._pred_history) == 0


def test_reset_clears_aim_entry_guard():
    """RESET must clear _aim_entry_done so the next AIM re-runs entry actions."""
    fsm, _ = _build_fsm_in_reset()
    fsm.step()
    assert fsm._aim_entry_done is False


def test_reset_clears_search_radar_cue():
    """RESET must clear the stored Search Radar cue.

    The cue link has no expiry, so a cue from the previous engagement would
    otherwise still be returned by get_cue() and immediately re-trigger
    IDLE → AIM without any fresh cue arriving.
    """
    cue_link = StubCueLink(cue=[1.0, 2.0, 0.5])
    track_filter = StubTrackFilter(position=[1.0, 2.0, 1.0], velocity=[0.0, 0.0, 0.0])
    fsm, _, _ = _build_fsm(
        cue_link=cue_link,
        track_filter=track_filter,
        config=FSMConfig(acquire_frames=1),
    )
    fsm.state = AtlasFSM.RESET
    fsm.step()  # RESET → IDLE, must wipe the stale cue
    assert cue_link.get_cue() is None
    assert fsm.state == AtlasFSM.IDLE


def test_idle_after_reset_waits_for_fresh_cue():
    """After RESET → IDLE, a stale cue must not re-trigger AIM; only fresh cues count.

    Reproduces the reported bug: with the cue persisting and acquire_frames=1,
    the FSM jumped IDLE → AIM on the very next step despite no new cue.
    """
    cue_link = StubCueLink(cue=[1.0, 2.0, 0.5])
    track_filter = StubTrackFilter(position=[1.0, 2.0, 1.0], velocity=[0.0, 0.0, 0.0])
    fsm, _, _ = _build_fsm(
        cue_link=cue_link,
        track_filter=track_filter,
        config=FSMConfig(acquire_frames=1),
    )
    fsm.state = AtlasFSM.RESET
    fsm.step()  # RESET → IDLE (clears the stale cue)
    assert fsm.state == AtlasFSM.IDLE
    fsm.step()  # IDLE step: cue is gone → must stay in IDLE
    assert fsm.state == AtlasFSM.IDLE
    # A genuinely fresh cue then drives the next acquisition.
    cue_link.cue = [3.0, 4.0, 1.0]
    fsm.step()
    assert fsm.state == AtlasFSM.AIM


# ---------------------------------------------------------------------------
# commanded_aim property
# ---------------------------------------------------------------------------

def test_commanded_aim_initial_value_is_zero():
    """commanded_aim must be (0.0, 0.0) before any motor command is issued."""
    fsm, _, _ = _build_fsm()
    assert fsm.commanded_aim == (0.0, 0.0)


def test_commanded_aim_reports_last_commanded_angles():
    """commanded_aim must report the (pan, tilt) angles last commanded to the motors.

    The FCR no longer reads commanded_aim (it gates on the real position sensors),
    but the property is retained for telemetry/introspection, so it must reflect
    the FSM's last commanded target.
    """
    config = FSMConfig(idle_pan=0.4, idle_tilt=0.1)
    fsm, _, _ = _build_fsm(config=config)
    fsm.step()
    assert fsm.commanded_aim == pytest.approx((0.4, 0.1))


# ---------------------------------------------------------------------------
# _compute_aim_angles
# ---------------------------------------------------------------------------

def test_compute_aim_angles_straight_ahead():
    """Target directly in front (dy>0, dz=0, dx=0) → pan=0, tilt=0."""
    fsm, _, _ = _build_fsm()
    pan, tilt = fsm._compute_aim_angles([0.0, 5.0, 0.0])
    assert pan == pytest.approx(0.0)
    assert tilt == pytest.approx(0.0)


def test_compute_aim_angles_target_to_the_right():
    """Target 45° to the right: dx=1, dy=1, dz=0 → pan=pi/4, tilt=0."""
    fsm, _, _ = _build_fsm()
    pan, tilt = fsm._compute_aim_angles([1.0, 1.0, 0.0])
    assert pan == pytest.approx(math.pi / 4)
    assert tilt == pytest.approx(0.0)


def test_compute_aim_angles_target_above():
    """Target directly above and ahead: dx=0, dy=1, dz=1 → pan=0, tilt=pi/4."""
    fsm, _, _ = _build_fsm()
    pan, tilt = fsm._compute_aim_angles([0.0, 1.0, 1.0])
    assert pan == pytest.approx(0.0)
    assert tilt == pytest.approx(math.pi / 4)


# ---------------------------------------------------------------------------
# _target_within_range (range-only; no ground reasoning lives in the FSM)
# ---------------------------------------------------------------------------

def test_target_within_range_within_bounds():
    fsm, _, _ = _build_fsm()
    assert fsm._target_within_range([1.0, 1.0, 1.0]) is True


def test_target_within_range_too_far():
    fsm, _, _ = _build_fsm()
    assert fsm._target_within_range([10.0, 10.0, 10.0]) is False


def test_target_within_range_ignores_ground():
    """A target at the floor (dz=0.0) but within max_range IS in range."""
    fsm, _, _ = _build_fsm()
    assert fsm._target_within_range([0.5, 0.5, 0.0]) is True


def test_target_within_range_exactly_at_max_range():
    """A target exactly at max_range=10.0 on one axis must be in range (≤ check)."""
    fsm, _, _ = _build_fsm()
    assert fsm._target_within_range([0.0, 0.0, 10.0]) is True


# ---------------------------------------------------------------------------
# Full state walk-through
# ---------------------------------------------------------------------------

def test_full_cycle_idle_to_idle():
    """Drive one FSM through IDLE → AIM → TRACK_PREDICT → ENGAGING → RESET → IDLE."""
    cue_link = StubCueLink(cue=[2.0, 2.0, 2.0])
    fcr = StubFCR(position=[2.0, 2.0, 2.0], locked=False)
    track_filter = StubTrackFilter(position=[2.0, 2.0, 2.0], velocity=[0.0, 0.0, 0.0])
    predictor = StubPredictor([2.0, 2.0, 2.0])
    ground = StubGroundHitLink(hit=False)
    config = FSMConfig(acquire_frames=1, lookahead_steps=1, converge_frames=1)
    fsm, _, _ = _build_fsm(
        cue_link=cue_link,
        fcr=fcr,
        track_filter=track_filter,
        ballistic_predictor=predictor,
        ground_hit_link=ground,
        config=config,
    )

    # IDLE → AIM (one cue, acquire_frames=1)
    fsm.step()
    assert fsm.state == AtlasFSM.AIM

    # AIM → TRACK_PREDICT once the FCR locks
    fsm.step()  # not locked yet
    assert fsm.state == AtlasFSM.AIM
    fcr._locked = True
    fsm.step()
    assert fsm.state == AtlasFSM.TRACK_PREDICT

    # TRACK_PREDICT → ENGAGING after convergence (3 steps with lookahead 1)
    fsm.step()
    fsm.step()
    fsm.step()
    assert fsm.state == AtlasFSM.ENGAGING

    # ENGAGING → RESET on ground-hit cue
    ground.hit = True
    fsm.step()
    assert fsm.state == AtlasFSM.RESET

    # RESET → IDLE
    fsm.step()
    assert fsm.state == AtlasFSM.IDLE
