"""Track Filter — Kalman filter estimating current target state.

This is the only module that imports filterpy. The KalmanFilter attributes
kf.F, kf.H, kf.R, kf.Q, and kf.B are kept public so tests can verify the
matrix design directly.

See ADR-0002 for the 6-state constant-velocity + gravity model.
See ADR-0003 for the continuous dual-sensor fusion design.
See ADR-0005 for the filterpy library choice.
"""

import numpy as np
from filterpy.kalman import KalmanFilter


_GRAVITY = -9.81  # m/s², applied on Z axis via control input

# Large initial diagonal covariance — signals high uncertainty before first measurement.
_INITIAL_COVARIANCE = 1000.0


class TrackFilter:
    """Kalman filter that fuses FCR and SearchRadar measurements into a
    clean current-state estimate of target position and velocity.

    State vector: [x, y, z, vx, vy, vz] — relative to turret, Z-up ENU.
    Process model: constant velocity + gravity (az = -9.81 m/s²) as control input.
    Measurement model: direct position observation [x, y, z] from either sensor.

    Both sensors update the same filter every timestep, weighted by their
    noise covariances. FCR dominates (low R_fcr); SearchRadar contributes
    a weaker correction (high R_search). This is genuine continuous fusion —
    neither sensor is gated by FSM state.

    The filterpy KalmanFilter instance is exposed as ``self.kf`` so that
    the matrix design (F, H, B, R, Q) can be inspected directly in tests.

    This class is the seam for future filter upgrades (e.g. EKF, UKF, IMM).
    Swapping this class does not affect FCR, SearchRadar, or the FSM.

    See ADR-0002 for Kalman filter rationale.
    See ADR-0003 for continuous fusion rationale.
    See ADR-0005 for filterpy library choice.
    """

    def __init__(
        self,
        timestep_ms: int,
        R_fcr: float,
        R_search: float,
        Q: float,
    ) -> None:
        """Construct and configure the Kalman filter matrices.

        Args:
            timestep_ms: Simulation timestep in milliseconds. Determines dt
                         used to build the state transition matrix F and the
                         control matrix B.
            R_fcr:       FCR measurement noise variance (scalar, m²). Applied
                         equally to each axis as R_fcr * I₃. Low value —
                         FCR is precise.
            R_search:    SearchRadar measurement noise variance (scalar, m²).
                         Applied as R_search * I₃. High value — SearchRadar
                         is coarse.
            Q:           Process noise scale factor. Scales the identity
                         process-noise matrix to account for model mismatch
                         (wind, drag, launch variability).
        """
        dt = timestep_ms / 1000.0
        self._R_fcr = R_fcr
        self._R_search = R_search

        # dim_x=6: [x, y, z, vx, vy, vz]
        # dim_z=3: [x, y, z] measurements
        # dim_u=3: [ax, ay, az] control (gravity)
        self.kf = KalmanFilter(dim_x=6, dim_z=3, dim_u=3)

        # --- State transition matrix F: constant-velocity kinematics ---
        # x_k+1 = x_k + vx*dt, etc.
        self.kf.F = np.array([
            [1, 0, 0, dt,  0,  0],
            [0, 1, 0,  0, dt,  0],
            [0, 0, 1,  0,  0, dt],
            [0, 0, 0,  1,  0,  0],
            [0, 0, 0,  0,  1,  0],
            [0, 0, 0,  0,  0,  1],
        ], dtype=float)

        # --- Measurement matrix H: observe position only ---
        # z_k = [x, y, z] = H * state
        self.kf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
        ], dtype=float)

        # --- Control matrix B: acceleration [ax,ay,az] integrates into velocity ---
        # velocity_k+1 += B * u, where u = [0, 0, -9.81]
        self.kf.B = np.array([
            [0,  0,  0 ],
            [0,  0,  0 ],
            [0,  0,  0 ],
            [dt, 0,  0 ],
            [0,  dt, 0 ],
            [0,  0,  dt],
        ], dtype=float)

        # --- Measurement noise R: set to FCR default; overridden per update ---
        self.kf.R = np.eye(3) * R_fcr

        # --- Process noise Q: scaled identity ---
        self.kf.Q = np.eye(6) * Q

        # Initialise state, covariance, and the initialised flag.
        self._reset_state()

    def predict(self) -> None:
        """Propagate Kalman state one timestep forward using the ballistic model.

        Applies gravity as a control input u = [0, 0, -9.81] m/s² through
        the B matrix, decoupling gravity from the state vector. Call once
        per timestep, before any update_fcr() calls.
        """
        u = np.array([[0.0], [0.0], [_GRAVITY]])
        self.kf.predict(u=u)

    def update_fcr(self, measurement: list[float]) -> None:
        """Fuse a FireControlRadar measurement into the filter.

        Sets kf.R to R_fcr * I₃ (low noise — FCR is precise) before
        calling the Kalman update step, then marks the filter as initialised.

        Args:
            measurement: Turret-relative [dx, dy, dz] from FireControlRadar,
                         in metres, Z-up ENU frame.
        """
        self.kf.R = np.eye(3) * self._R_fcr
        self.kf.update(np.array(measurement, dtype=float).reshape(3, 1))
        self._initialised = True

    def update_search(self, measurement: list[float]) -> None:
        """Fuse a Search Radar measurement into the filter.

        Sets kf.R to R_search * I₃ (high noise — the Search Radar is coarse)
        before calling the Kalman update step, then marks the filter as
        initialised. Mirrors update_fcr() but with the weaker, less-trusted
        noise covariance, so a Search Radar fix nudges the estimate while the
        precise FCR measurement dominates.

        Driven by the FSM's TRACK_PREDICT state (genuine dual-sensor fusion);
        the FCR path runs continuously from the main loop. See ADR-0003.

        Args:
            measurement: Turret-relative [dx, dy, dz] from a Search Radar cue,
                         in metres, Z-up ENU frame.
        """
        self.kf.R = np.eye(3) * self._R_search
        self.kf.update(np.array(measurement, dtype=float).reshape(3, 1))
        self._initialised = True

    def _reset_state(self) -> None:
        """Zero the Kalman state, reset covariance to initial uncertainty, and
        clear the initialised flag.

        Called from __init__ and reset() to guarantee both code paths set
        identical starting conditions. Using _INITIAL_COVARIANCE here ensures
        the two call sites cannot diverge.
        """
        self.kf.x = np.zeros((6, 1))
        self.kf.P = np.eye(6) * _INITIAL_COVARIANCE
        self._initialised = False

    def reset(self) -> None:
        """Reinitialise Kalman state and covariance to defaults.

        Called on AIM entry (new target) and on RESET (engagement complete).
        Clears stale history from any previous engagement and resets
        is_initialised() to False so the FSM waits for a fresh measurement.
        """
        self._reset_state()

    def get_position(self) -> list[float]:
        """Return current filtered position estimate [x, y, z] in metres.

        Returns:
            3-element list [x, y, z] in turret-relative Z-up ENU frame.
        """
        return [float(self.kf.x[0, 0]), float(self.kf.x[1, 0]), float(self.kf.x[2, 0])]

    def get_velocity(self) -> list[float]:
        """Return current filtered velocity estimate [vx, vy, vz] in m/s.

        Returns:
            3-element list [vx, vy, vz] in turret-relative Z-up ENU frame.
        """
        return [float(self.kf.x[3, 0]), float(self.kf.x[4, 0]), float(self.kf.x[5, 0])]

    def is_initialised(self) -> bool:
        """Return True after at least one update_fcr() call.

        The FSM checks this before trusting get_position() or get_velocity().
        Returns False on construction and after reset().
        """
        return self._initialised
