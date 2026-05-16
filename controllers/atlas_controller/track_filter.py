"""Track Filter — Kalman filter estimating current target state."""
from __future__ import annotations


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
        """
        Args:
            timestep_ms: Simulation timestep in milliseconds. Used to build
                         the state transition matrix F.
            R_fcr:       FCR measurement noise variance (scalar, applied
                         equally to each axis). Low value — FCR is precise.
            R_search:    SearchRadar measurement noise variance (scalar).
                         High value — SearchRadar is coarse.
            Q:           Process noise scale factor. Accounts for model
                         mismatch (wind, launch variability, drag).
        """
        raise NotImplementedError

    def predict(self) -> None:
        """Propagate Kalman state one timestep forward using the ballistic model.

        Applies gravity as a control input (u = -9.81 m/s² on Z axis).
        Call once per timestep, before any update_fcr() or update_search() calls.
        """
        raise NotImplementedError

    def update_fcr(self, measurement: list[float]) -> None:
        """Fuse a FireControlRadar measurement into the filter.

        Applies R_fcr as the measurement noise covariance matrix before
        calling the Kalman update step.

        Args:
            measurement: Relative [dx, dy, dz] from FireControlRadar.
        """
        raise NotImplementedError

    def update_search(self, measurement: list[float]) -> None:
        """Fuse a SearchRadar measurement into the filter.

        Applies R_search as the measurement noise covariance matrix before
        calling the Kalman update step.

        Args:
            measurement: Relative [dx, dy, dz] from SearchRadar.
        """
        raise NotImplementedError

    def reset(self) -> None:
        """Reinitialise Kalman state and covariance to defaults.

        Called at the ACQUIRE transition when a new target is locked onto.
        Clears stale history from any previous engagement.
        """
        raise NotImplementedError

    def get_position(self) -> list[float]:
        """Return current filtered position estimate [x, y, z]."""
        raise NotImplementedError

    def get_velocity(self) -> list[float]:
        """Return current filtered velocity estimate [vx, vy, vz]."""
        raise NotImplementedError

    def is_initialised(self) -> bool:
        """Return True after at least one update_fcr() or update_search() call.

        The FSM checks this before trusting get_position() or get_velocity().
        """
        raise NotImplementedError
