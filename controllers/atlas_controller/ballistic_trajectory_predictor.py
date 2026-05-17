"""Ballistic Trajectory Predictor — predicts future projectile intercept point."""


class BallisticTrajectoryPredictor:
    """Predicts the future position of the tracked projectile using closed-form
    ballistic equations applied to the current TrackFilter state estimate.

    Uses the filtered [x, y, z, vx, vy, vz] from TrackFilter and applies:
        x_pred = x + vx * T
        y_pred = y + vy * T
        z_pred = z + vz * T - 0.5 * g * T²
    where T = lookahead_steps * timestep_ms / 1000.0.

    Does not run the Kalman predict step — the TrackFilter's velocity estimate
    already encodes ballistic dynamics. This keeps prediction stateless and
    avoids mutating the live filter.

    Coordinate system: Z-up ENU, position relative to the turret.

    Distinct from the future AttackPredictor, which will reason about launch
    origin and attack frequency across multiple engagements.

    See ADR-0001 for fixed-lookahead rationale.
    See ADR-0002 for prediction approach.
    """

    GRAVITY: float = 9.81  # m/s²

    def __init__(self, track_filter, timestep_ms: int) -> None:
        """Store the filter and timestep; no computation performed here.

        Args:
            track_filter: TrackFilter instance (or compatible stub). Must expose
                          get_position() -> list[float] and
                          get_velocity() -> list[float]. Queried fresh on every
                          call to get_intercept() — never mutated by this class.
            timestep_ms:  Simulation timestep in milliseconds. Used to convert
                          lookahead_steps to seconds: T = steps * ms / 1000.
        """
        self._track_filter = track_filter
        self._timestep_ms = timestep_ms

    def get_intercept(self, lookahead_steps: int) -> list[float]:
        """Predict projectile position lookahead_steps simulation steps ahead.

        Reads the current filter state on each call; does not mutate the filter.

        Args:
            lookahead_steps: Number of simulation timesteps to look ahead.
                             0 returns the current filtered position.

        Returns:
            Predicted relative [x, y, z] position (metres, Z-up ENU frame)
            at the intercept time T = lookahead_steps * timestep_ms / 1000.0.
        """
        x, y, z = self._track_filter.get_position()
        vx, vy, vz = self._track_filter.get_velocity()

        T = lookahead_steps * self._timestep_ms / 1000.0

        x_pred = x + vx * T
        y_pred = y + vy * T
        z_pred = z + vz * T - 0.5 * self.GRAVITY * T ** 2

        return [x_pred, y_pred, z_pred]
