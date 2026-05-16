"""Ballistic Trajectory Predictor — predicts future projectile intercept point."""
from __future__ import annotations


class BallisticTrajectoryPredictor:
    """Predicts the future position of the tracked projectile using closed-form
    ballistic equations applied to the current TrackFilter state estimate.

    Uses the filtered [x, y, z, vx, vy, vz] from TrackFilter and applies:
        x_pred = x + vx * T
        y_pred = y + vy * T
        z_pred = z + vz * T - 0.5 * g * T²
    where T = lookahead_steps * timestep_s.

    Does not run the Kalman predict step — the TrackFilter's velocity estimate
    already encodes ballistic dynamics. This keeps prediction stateless and
    avoids mutating the live filter.

    Distinct from the future AttackPredictor, which will reason about launch
    origin and attack frequency across multiple engagements.

    See ADR-0001 for fixed-lookahead rationale.
    See ADR-0002 for prediction approach.
    """

    GRAVITY: float = 9.81  # m/s²

    def __init__(self, track_filter, timestep_ms: int) -> None:
        """
        Args:
            track_filter: TrackFilter instance. Provides get_position() and
                          get_velocity() each call to get_intercept().
            timestep_ms:  Simulation timestep in milliseconds. Used to convert
                          lookahead_steps to seconds.
        """
        raise NotImplementedError

    def get_intercept(self, lookahead_steps: int) -> list[float]:
        """Predict projectile position lookahead_steps ahead of now.

        Args:
            lookahead_steps: Number of simulation timesteps to look ahead.

        Returns:
            Predicted relative [x, y, z] position at the intercept time.
        """
        raise NotImplementedError
