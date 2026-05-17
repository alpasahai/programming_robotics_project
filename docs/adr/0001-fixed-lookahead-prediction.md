# ADR-0001: Fixed Lookahead for Intercept Prediction

**Status:** Accepted

## Decision

The Prediction Module uses a fixed lookahead window (`LOOKAHEAD_STEPS` timesteps) to estimate the intercept point, rather than solving for the true intercept time.

## Context

A true intercept time calculation would couple the ballistic prediction with the turret's motor slew rate — solving for the time T where "where the projectile is at T" matches "where the turret can physically point by T". This requires knowing the motor's maximum angular velocity and modelling the rotation time as a function of the required angle delta.

## Reasoning

- The Webots `RotationalMotor` slew rate is not explicitly configured in the current world file, making it hard to model accurately.
- Fixed lookahead is sufficient to demonstrate predictive (non-reactive) tracking behaviour, which is what the assignment assesses.
- The constant is named and isolated (`LOOKAHEAD_STEPS`) so it is easy to tune during testing.

## Upgrade path

To implement true intercept time: measure the motor's effective slew rate empirically in Webots, then solve iteratively for T such that `predict(T) == turret_angle_at(T)`. Replace the fixed `t = LOOKAHEAD_STEPS * dt` in `_compute_intercept` with the solved T.
