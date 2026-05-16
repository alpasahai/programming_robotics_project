# ADR-0002: Parabolic Curve Fitting for Prediction, Kalman Filter Deferred

**Status:** Superseded by [ADR-0002-kalman-filter-for-tracking-and-prediction.md](0002-kalman-filter-for-tracking-and-prediction.md)

## Decision

The Prediction Module uses least-squares parabolic curve fitting on FCR position history to estimate the intercept point. A Kalman filter is deferred to a future iteration.

## Context

Two approaches were considered for ballistic intercept prediction:

1. **Parabolic curve fit** — fit a parabola to N observed positions from the FCR history buffer, extrapolate to the intercept point. Naturally smooths noise across all samples. Simpler to implement and explain.
2. **Kalman filter** — use the ballistic motion model as the predict step, fuse FCR and Search Radar measurements via noise-weighted update step. Optimal under Gaussian noise, handles two sensors with different covariances naturally.

The Kalman filter is genuinely justified here (two sensors with known different noise characteristics, known ballistic motion model), but adds implementation complexity.

## Reasoning

Parabolic fitting is implemented first to validate the prediction architecture end-to-end before introducing filter complexity. The two approaches share the same ballistic motion model — the Kalman predict step is the same parabolic equation, just used differently.

## Upgrade path

Replace the least-squares parabola fit in the Prediction Module with a Kalman filter. State vector: `[x, y, z, vx, vy, vz]`. Predict step: ballistic equations (same as current parabolic model). Update step: fuse FCR measurement (low R) and Search Radar cue (high R) each tick. The FSM and Track Report interface do not change.
