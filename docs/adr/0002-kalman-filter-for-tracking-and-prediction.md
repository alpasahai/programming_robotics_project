# ADR-0002: Kalman Filter for Both Tracking and Prediction

**Status:** Accepted, revised by ADR-0009  
**Supersedes:** ADR-0002-parabolic-prediction-before-kalman.md

## Decision

A linear Kalman filter (via filterpy) is used for both target state estimation
(TrackFilter) and intercept prediction (BallisticTrajectoryPredictor). Parabolic
curve fitting (numpy polyfit) is not used.

## Context

The original plan deferred the Kalman filter in favour of a parabolic fit on raw
FCR position history. Two reasons forced the change:

1. **Filtering requirement** — the tracking path needs a state estimator that
   smooths noisy FCR measurements and estimates velocity. A parabola fit on raw
   position history treats measurement noise as signal.

2. **Prediction quality** — polyfit on noisy positions fits noise as signal.
   Residuals grow quadratically with noise variance. The Kalman filter's
   state estimate is already noise-filtered, making intercept prediction via
   closed-form ballistic equations significantly more accurate.

## Design

**TrackFilter** uses a 6-state Kalman filter `[x, y, z, vx, vy, vz]`:
- State transition F encodes constant velocity; gravity enters as a control
  input `u = [0, 0, -9.81]` via the B matrix each predict step.
- One measurement update path: `update_fcr()`. ADR-0009 removed continuous
  Search Radar measurement fusion; Search Radar now supplies acquisition cues,
  not TrackFilter measurements.

**BallisticTrajectoryPredictor** does not run the Kalman predict step. It reads
`get_position()` and `get_velocity()` from TrackFilter and applies closed-form
ballistic equations:
```
x_pred = x + vx * T
y_pred = y + vy * T
z_pred = z + vz * T - 0.5 * g * T²
```
where `T = lookahead_steps * timestep_s`. This avoids mutating the live filter
and keeps prediction stateless.

## Upgrade path

Replace TrackFilter internals with an EKF or UKF if drag modelling is needed.
The FSM, FCR, SearchRadar, and BallisticTrajectoryPredictor are unaffected.
See ADR-0005 for the filterpy library choice.
