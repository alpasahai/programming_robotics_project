# ATLAS Domain Glossary

## BallisticTrajectoryPredictor

The subsystem responsible for estimating the future position of a tracked projectile using ballistic equations. Operates during the `PREDICT` FSM state. Takes the position history from the Track Filter and outputs an Intercept Point. Distinct from the future `AttackPredictor`.

## AttackPredictor

Planned future subsystem. Analyses patterns across multiple engagements to estimate launch origin and attack frequency. Operates at a higher level than the `BallisticTrajectoryPredictor` — it reasons about threats, not individual projectile trajectories.

## Search Radar

Wide-beam acquisition radar external to ATLAS. Simulated using Webots world coordinates with added Gaussian noise to produce coarse, imprecise target cues. Not part of ATLAS — it represents a higher-level external system that cues the fire-control system. Not under test.

## Fire-Control Radar (FCR)

ATLAS's own narrow-beam on-board radar sensor. Provides precise target measurements to the Track Filter. Currently simulated with low-noise world position data as a placeholder — will be replaced with a real Webots sensor node in a future iteration. Distinct from the Search Radar: the FCR is part of ATLAS, the Search Radar is external to it.

## ATLAS (Autonomous Tracking and Laser Aiming System)

The cue-driven fire-control system being built. Receives coarse cues from the Search Radar, uses its own FCR to lock on precisely, fuses inputs via the Track Filter, and commands the turret via the FSM.

## Track Filter

The subsystem that receives raw FCR measurements and smooths them into a clean current-state estimate of position and velocity. Owns the position history buffer. Currently implemented as a finite-difference velocity estimator over a fixed-size deque; the Kalman filter upgrade replaces only this class. Sits between the FCR and the Prediction Module. Does not know the turret position — it receives relative coordinates from the FCR.

## Track Report

The data package the Track Filter produces each timestep: filtered relative position `[dx, dy, dz]` from the turret and estimated velocity `[vx, vy, vz]`.

## Intercept Point

The predicted future position of the projectile that the turret should aim at. Computed by the `BallisticTrajectoryPredictor` using a fixed lookahead window.

## FSM (Finite State Machine)

The central control module governing all turret behaviour. States: `SEARCH`, `ACQUIRE`, `TRACK`, `PREDICT`, `AIMING`, `ENGAGING`, `RESET`.
