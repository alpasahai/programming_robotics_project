"""Tests for TrackFilter — Kalman filter fusing FCR measurements.

Verifies the Kalman matrix structure (F, H, B, R), predict() ballistic
propagation, and the is_initialised()/reset() state lifecycle. All matrix
assertions use numpy.
"""
import numpy as np
import pytest
from track_filter import TrackFilter


# ---------------------------------------------------------------------------
# Shared construction helpers
# ---------------------------------------------------------------------------

TIMESTEP_MS = 32          # 32 ms — typical Webots timestep
DT = TIMESTEP_MS / 1000.0 # 0.032 s

R_FCR = 0.1    # low — FCR is precise
R_SEARCH = 5.0 # high — SearchRadar is coarse
Q_SCALE = 0.5

GRAVITY = -9.81  # m/s² applied on Z


def make_filter() -> TrackFilter:
    """Return a freshly constructed TrackFilter with standard test parameters."""
    return TrackFilter(
        timestep_ms=TIMESTEP_MS,
        R_fcr=R_FCR,
        R_search=R_SEARCH,
        Q=Q_SCALE,
    )


# ---------------------------------------------------------------------------
# Slice 1: is_initialised() starts False
# ---------------------------------------------------------------------------

def test_is_initialised_false_on_construction():
    """is_initialised() must return False immediately after construction.

    The FSM relies on this to refuse get_position()/get_velocity() before
    the first measurement arrives.
    """
    tf = make_filter()
    assert tf.is_initialised() is False


# ---------------------------------------------------------------------------
# Slice 2: is_initialised() True after update_fcr()
# ---------------------------------------------------------------------------

def test_is_initialised_true_after_update_fcr():
    """update_fcr() must flip is_initialised() to True."""
    tf = make_filter()
    tf.update_fcr([1.0, 2.0, 3.0])
    assert tf.is_initialised() is True


def test_is_initialised_true_after_update_search():
    """update_search() must flip is_initialised() to True."""
    tf = make_filter()
    tf.update_search([1.0, 2.0, 3.0])
    assert tf.is_initialised() is True


# ---------------------------------------------------------------------------
# Slice 4: reset() clears the initialised flag
# ---------------------------------------------------------------------------

def test_reset_clears_initialised_flag():
    """reset() must return is_initialised() to False regardless of prior updates."""
    tf = make_filter()
    tf.update_fcr([1.0, 2.0, 3.0])
    assert tf.is_initialised() is True
    tf.reset()
    assert tf.is_initialised() is False


# ---------------------------------------------------------------------------
# Slice 5: get_position() / get_velocity() return 3-element lists
# ---------------------------------------------------------------------------

def test_get_position_returns_list_of_length_3():
    """get_position() must return a list of exactly 3 floats."""
    tf = make_filter()
    result = tf.get_position()
    assert isinstance(result, list)
    assert len(result) == 3


def test_get_velocity_returns_list_of_length_3():
    """get_velocity() must return a list of exactly 3 floats."""
    tf = make_filter()
    result = tf.get_velocity()
    assert isinstance(result, list)
    assert len(result) == 3


# ---------------------------------------------------------------------------
# Slice 6: Kalman matrix structure — F (constant velocity) and H (3x6)
# ---------------------------------------------------------------------------

def test_F_matrix_encodes_constant_velocity_model():
    """kf.F must be the 6x6 constant-velocity transition matrix for dt=0.032 s.

    Expected structure (positions integrate velocity over dt):

        [ 1  0  0  dt  0   0  ]
        [ 0  1  0   0  dt  0  ]
        [ 0  0  1   0   0  dt ]
        [ 0  0  0   1   0   0 ]
        [ 0  0  0   0   1   0 ]
        [ 0  0  0   0   0   1 ]
    """
    tf = make_filter()
    expected_F = np.array([
        [1, 0, 0, DT,  0,  0],
        [0, 1, 0,  0, DT,  0],
        [0, 0, 1,  0,  0, DT],
        [0, 0, 0,  1,  0,  0],
        [0, 0, 0,  0,  1,  0],
        [0, 0, 0,  0,  0,  1],
    ], dtype=float)
    np.testing.assert_array_almost_equal(tf.kf.F, expected_F)


def test_H_matrix_selects_position_from_state():
    """kf.H must be the 3x6 matrix that maps state to [x, y, z].

    Expected:
        [ 1  0  0  0  0  0 ]
        [ 0  1  0  0  0  0 ]
        [ 0  0  1  0  0  0 ]
    """
    tf = make_filter()
    expected_H = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
    ], dtype=float)
    np.testing.assert_array_almost_equal(tf.kf.H, expected_H)


# ---------------------------------------------------------------------------
# Slice 7: B matrix — gravity control input shape and values
# ---------------------------------------------------------------------------

def test_B_matrix_maps_gravity_to_velocity():
    """kf.B must route the 3-element acceleration control input [ax,ay,az]
    into the velocity components of the 6-element state.

    Only the velocity rows are non-zero — gravity enters velocity directly,
    and position integrates velocity through F, not B. Specifically:

        B = [ 0   0   0  ]   <- x position unaffected by acceleration directly
            [ 0   0   0  ]   <- y position unaffected
            [ 0   0   0  ]   <- z position unaffected (gravity enters via vz)
            [ dt  0   0  ]   <- vx += ax*dt
            [ 0   dt  0  ]   <- vy += ay*dt
            [ 0   0   dt ]   <- vz += az*dt

    u = [0, 0, -9.81] so only vz is affected: vz += -9.81 * dt per step.
    """
    tf = make_filter()
    expected_B = np.array([
        [0,  0,  0 ],
        [0,  0,  0 ],
        [0,  0,  0 ],
        [DT, 0,  0 ],
        [0,  DT, 0 ],
        [0,  0,  DT],
    ], dtype=float)
    np.testing.assert_array_almost_equal(tf.kf.B, expected_B)


# ---------------------------------------------------------------------------
# Slice 7b: Q matrix — scaled identity
# ---------------------------------------------------------------------------

def test_Q_matrix_is_scaled_identity():
    """kf.Q must equal Q_scale * I₆.

    The process noise matrix is an isotropic scaled identity, capturing
    model mismatch uniformly across all state dimensions.
    """
    tf = make_filter()
    expected_Q = Q_SCALE * np.eye(6)
    np.testing.assert_array_almost_equal(tf.kf.Q, expected_Q)


# ---------------------------------------------------------------------------
# Slice 8: R_fcr vs R_search produce different diagonal scaling
# ---------------------------------------------------------------------------

def test_update_fcr_applies_low_R():
    """After update_fcr(), kf.R diagonal must equal R_fcr (low noise)."""
    tf = make_filter()
    tf.update_fcr([1.0, 2.0, 3.0])
    expected_R = np.eye(3) * R_FCR
    np.testing.assert_array_almost_equal(tf.kf.R, expected_R)


def test_update_search_applies_high_R():
    """After update_search(), kf.R diagonal must equal R_search (high noise)."""
    tf = make_filter()
    tf.update_search([1.0, 2.0, 3.0])
    expected_R = np.eye(3) * R_SEARCH
    np.testing.assert_array_almost_equal(tf.kf.R, expected_R)


def test_update_search_moves_estimate_toward_measurement():
    """update_search() must pull the position estimate toward the measurement."""
    tf = make_filter()
    tf.update_search([4.0, 5.0, 6.0])
    pos = tf.get_position()
    # From a zeroed state with large initial covariance, a single measurement
    # pulls the estimate most of the way toward it.
    assert pos[0] > 0.0 and pos[1] > 0.0 and pos[2] > 0.0


# ---------------------------------------------------------------------------
# Slice 9: predict() propagates position by v*dt and gravity affects vz
# ---------------------------------------------------------------------------

def test_predict_propagates_position_by_velocity():
    """predict() must advance x, y, z by vx*dt, vy*dt, vz*dt.

    Gravity only affects vz on subsequent steps; first predict() from
    zero velocity leaves position at zero.
    """
    tf = make_filter()
    # Inject a known state: position (2,3,4), velocity (1,0,0)
    tf.kf.x = np.array([[2.0], [3.0], [4.0], [1.0], [0.0], [0.0]])
    tf.predict()
    pos = tf.get_position()
    # x advances by vx*dt, y and z unchanged (vy=vz=0 before gravity step)
    assert pos[0] == pytest.approx(2.0 + 1.0 * DT, abs=1e-9)
    assert pos[1] == pytest.approx(3.0, abs=1e-9)
    assert pos[2] == pytest.approx(4.0, abs=1e-9)


def test_predict_applies_gravity_to_vz():
    """predict() must add gravity*dt to vz each timestep.

    Starting from vz=0, after one predict(), vz must equal GRAVITY*dt.
    """
    tf = make_filter()
    tf.kf.x = np.zeros((6, 1))
    tf.predict()
    vel = tf.get_velocity()
    expected_vz = GRAVITY * DT
    assert pytest.approx(vel[2], abs=1e-9) == expected_vz


def test_predict_parabolic_z_over_multiple_steps():
    """Over N steps, z must follow the parabolic ballistic trajectory.

    Starting from z=10, vz=0, after N steps:
        vz_n = GRAVITY * n * dt
        z_n  = 10 + 0*t - 0.5*g*t²  (discrete approximation)
    """
    N = 10
    tf = make_filter()
    z0 = 10.0
    tf.kf.x = np.array([[0.0], [0.0], [z0], [0.0], [0.0], [0.0]])

    for _ in range(N):
        tf.predict()

    t = N * DT
    expected_z = z0 + 0.5 * GRAVITY * t**2  # continuum approximation
    pos = tf.get_position()
    # Allow 1% relative tolerance. The discrete Euler integrator accumulates
    # O(g·dt²) error per step, giving a total O(g·N·dt²) lag relative to the
    # continuous parabola. Over 10 steps at dt=0.032 s this is ~0.003 m, well
    # within 1% of |expected_z|.
    assert abs(pos[2] - expected_z) < abs(expected_z) * 0.01 + 1e-6

