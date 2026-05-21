# ATLAS Domain Glossary

## BallisticTrajectoryPredictor

The subsystem responsible for estimating the future position of a tracked projectile using ballistic equations. Operates during the `PREDICT` FSM state. Takes the position history from the Track Filter and outputs an Intercept Point. Distinct from the future `AttackPredictor`.

## AttackPredictor

Planned future subsystem. Analyses patterns across multiple engagements to estimate launch origin and attack frequency. Operates at a higher level than the `BallisticTrajectoryPredictor` — it reasons about threats, not individual projectile trajectories.

## Search Radar

Wide-beam acquisition radar external to ATLAS. Implemented as a dedicated Webots controller process with its own world pose, vertical-FOV gate, rotating scan beam, and track buffer. It selects one target internally and emits that target's world-frame cue over an `Emitter` radio link. `track_id`s and node handles stay inside the Search Radar process; ATLAS receives only the latest cue position via `SearchRadarLink`. See ADR-0007 and ADR-0009.

## Fire-Control Radar (FCR)

ATLAS's own narrow-beam on-board radar sensor, embodied by the turret. It is simulated, but no longer omniscient: each update is gated by range and by an FOV cone around the turret's commanded aim. When the Search Radar cue slews the turret onto the projectile, the FCR locks and reports low-noise turret-relative measurements to the Track Filter. It has no independent boresight or `set_target()` interface. See ADR-0008.

## ATLAS (Autonomous Tracking and Laser Aiming System)

The cue-driven fire-control system being built. Receives coarse world-frame cues from the Search Radar, slews the turret until its own FCR locks, filters FCR measurements via the Track Filter, and commands the turret via the FSM.

## Track Filter

The subsystem that receives raw FCR measurements and smooths them into a clean current-state estimate of position and velocity. Owns the position history buffer. Currently implemented as a finite-difference velocity estimator over a fixed-size deque; the Kalman filter upgrade replaces only this class. Sits between the FCR and the Prediction Module. Does not know the turret position — it receives relative coordinates from the FCR.

## Track Report

The data package the Track Filter produces each timestep: filtered relative position `[dx, dy, dz]` from the turret and estimated velocity `[vx, vy, vz]`.

## Intercept Point

The predicted future position of the projectile that the turret should aim at. Computed by the `BallisticTrajectoryPredictor` using a fixed lookahead window.

## FSM (Finite State Machine)

The central control module governing all turret behaviour. States: `SEARCH`, `ACQUIRE`, `TRACK`, `PREDICT`, `AIMING`, `ENGAGING`, `RESET`.

## Turret Weapon

The turret destroys the incoming ball by firing a physical projectile (the Bullet) along its aim direction, rather than with an instant-hit laser. Travel time is intentional — it is what makes the `BallisticTrajectoryPredictor` and the Intercept Point meaningful. The cosmetic `AtlasLaser` node is retained purely as an aiming line and plays no role in hit detection. See `docs/superpowers/specs/2026-05-18-turret-weapon-design.md`.

## Bullet

The turret's fired projectile, defined by `AtlasBullet.proto`. A self-reporting `Robot` carrying a `bumper`-type `TouchSensor` and the `atlas_bullet_controller`; it detects its own collisions and publishes a hit via its `customData` field. Spawned dynamically by the turret supervisor at fire time and removed when the shot resolves. Distinct from the incoming ball (`Projectile`), which the turret aims *at*.

## Fire Command

The one-shot event the FSM emits on entering the `ENGAGING` state, carrying the intercept/aim for the shot. The FSM only produces the event; `atlas_controller` consumes it to spawn and launch the Bullet. Keeps node spawning out of the FSM so the FSM stays pure and unit testable.

