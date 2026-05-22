# ATLAS Domain Glossary

## BallisticTrajectoryPredictor

The subsystem responsible for estimating the future position of a tracked projectile using ballistic equations. Operates during the `TRACK_PREDICT` FSM state. Takes the position history from the Track Filter and outputs an Intercept Point. Distinct from the future `AttackPredictor`.

## AttackPredictor

Online learning subsystem on the ATLAS side. Observes each launch radar-only
(first Search Radar acquisition after a resolution cue, segmented by `LaunchLatch`),
discovers the launch sectors by online angular clustering (`SectorMap`), and learns
`P(next launch sector | recent context)` with an online logistic regression
(`SGDClassifier.partial_fit`). Predicts the **next launch sector** and exposes a
ready-aim bearing the FSM `IDLE` state uses to pre-slew the turret. Re-adapts when
the attacker's pattern drifts. Reasons about the *threat stream*, distinct from
the `BallisticTrajectoryPredictor` (individual projectile trajectories). No
dataset and no serialized model — it learns live.

## SectorMap

Online angular clustering subsystem used by `AttackPredictor`. Receives observed
launch bearings (pan angles in radians, normalised to `[-π, π]`) and discovers
launch sectors incrementally — no preset count or fixed boundaries. Clustering
tolerance is derived from radar noise. Cluster count is bounded by a capacity cap.
Each sector is identified by the mean bearing of its cluster. New observations
either reinforce an existing sector (within tolerance) or create a new one until
the cap is reached, at which point the closest sector absorbs the observation.

## LaunchLatch

Pure state machine that segments the continuous radar-cue stream into exactly one
launch observation per engagement. Waits for the previous engagement's cue to be
cleared (FSM RESET clears the cue; the latch watches for an empty cue) and then
accepts the first fresh, non-empty cue that arrives afterward as the launch
observation. This prevents training on a stale in-flight cue from the just-resolved
projectile — which is still tracked for one step after resolution — and ensures
each observation corresponds to a genuine new launch near its actual origin.

## Search Radar

Wide-beam acquisition radar external to ATLAS. Implemented as a dedicated Webots controller process with its own world pose, vertical-FOV gate, rotating scan beam, and track buffer. It selects one target internally and emits that target's world-frame cue over an `Emitter` radio link. `track_id`s and node handles stay inside the Search Radar process; ATLAS receives only the latest cue position via `SearchRadarLink`. See ADR-0007 and ADR-0009.

## Fire-Control Radar (FCR)

ATLAS's own narrow-beam on-board radar sensor, embodied by the turret. It is simulated, but no longer omniscient: each update is gated by range and by an FOV cone around the turret's commanded aim. When the Search Radar cue slews the turret onto the projectile, the FCR locks and reports low-noise turret-relative measurements to the Track Filter. It has no independent boresight or `set_target()` interface. See ADR-0008.

## Attacker

The offense side of the turret-defense game. A device-less supervisor `Robot` (`Attacker.proto`, DEF `ATTACKER`) running `attacker_controller`. It owns the incoming `Projectile`: it spawns the ball, registers each ground hit, despawns the ball, and spawns a fresh one after a delay. Today it drives a single recycled ball on a fixed cadence; the attack-pattern schedule and the ML launch log will live here later. The lifecycle logic lives in the unit-tested `Projectile` class, not in the controller loop. See ADR-0011 and `docs/superpowers/specs/2026-05-18-attack-pattern-and-ml-detection-design.md`.

## Projectile (incoming ball)

The target the turret defends against, defined by `Projectile.proto` (DEF `PROJECTILE`). A passive `Solid` with physics and no controller of its own — the `Attacker` drives it via the supervisor API. On a ground hit it is *recycled*, not deleted: the attacker parks it out of sensor range ("despawn"), then teleports it back to the spawn point and relaunches it ("spawn"). Recycling a single node keeps the `getFromDef("PROJECTILE")` handles that `search_radar_controller` and `atlas_controller` hold valid. Distinct from the `Bullet`, which the turret fires *at* it. See ADR-0011.

## Ground Hit

The event where the incoming `Projectile` touches the floor. Detected from the ball's real physics **contact points** (`getContactPoints`, world frame), not a height heuristic: a contact near the floor (`z <= floor_contact_z_m`) is a ground hit, a contact up in the air is a `Bullet` hit. Webots' contact `node_id` identifies the ball itself, not the other body, so contact **height** is the discriminator between the two outcomes. A ground contact only counts once the ball has cleared the floor since launch (an "airborne latch"), so the floor contact present at spawn is not mistaken for a landing. The `Attacker` registers each hit — incrementing a count and firing a callback — then despawns and respawns the ball. Registration is the seam where scoring / the future referee will hook in. See ADR-0011.

## ATLAS (Autonomous Tracking and Laser Aiming System)

The cue-driven fire-control system being built. Receives coarse world-frame cues from the Search Radar, slews the turret until its own FCR locks, filters FCR measurements via the Track Filter, and commands the turret via the FSM.

## Track Filter

The subsystem that receives raw FCR measurements and smooths them into a clean current-state estimate of position and velocity. Owns the position history buffer. Currently implemented as a finite-difference velocity estimator over a fixed-size deque; the Kalman filter upgrade replaces only this class. Sits between the FCR and the Prediction Module. Does not know the turret position — it receives relative coordinates from the FCR.

## Track Report

The data package the Track Filter produces each timestep: filtered relative position `[dx, dy, dz]` from the turret and estimated velocity `[vx, vy, vz]`.

## Intercept Point

The predicted future position of the projectile that the turret should aim at. Computed by the `BallisticTrajectoryPredictor` using a fixed lookahead window.

## FSM (Finite State Machine)

The central control module governing all turret behaviour. Five states: `IDLE` (hold a fixed beam direction, wait for a Search Radar cue), `AIM` (slew toward the cue until the FCR locks), `TRACK_PREDICT` (fuse both sensors, follow the filtered estimate, predict the intercept, and fire once the predicted-vs-observed error converges), `ENGAGING` (hold aim at the fired intercept), and `RESET` (wipe the filter and return to `IDLE`). Readiness to fire is the `ENGAGING` state itself — there is no separate flag. See ADR-0012.

## Turret Weapon

The turret destroys the incoming ball by firing a physical projectile (the Bullet) along its aim direction, rather than with an instant-hit laser. Travel time is intentional — it is what makes the `BallisticTrajectoryPredictor` and the Intercept Point meaningful. The cosmetic `AtlasLaser` node is retained purely as an aiming line and plays no role in hit detection. See `docs/superpowers/specs/2026-05-18-turret-weapon-design.md`.

## Bullet

The turret's fired projectile, defined by `AtlasBullet.proto` (DEF `ATLAS_BULLET`).
A passive `Solid` — no controller, sensor, or radio — that the `atlas_controller`
supervisor *recycles*, mirroring the incoming `Projectile`: one pre-placed node is
parked out of range, teleported to the muzzle and launched (`setVelocity`) toward
the intercept when the FSM enters `ENGAGING`, then parked again when the
engagement resolves (the bullet struck the ball, the ball landed, or a flight-time
timeout). It carries no hit sensor: the incoming `Projectile` is the single source
of hit truth (it classifies a mid-air contact as a bullet hit by world-Z), and the
attacker relays that as a bullet-hit pulse on channel 3. Distinct from the
incoming ball (`Projectile`), which the turret aims *at*. See ADR-0013.

## Fire Command

The one-shot event the FSM emits on entering the `ENGAGING` state, carrying the intercept/aim for the shot. The FSM only produces the event; `atlas_controller` consumes it to spawn and launch the Bullet. Keeps node spawning out of the FSM so the FSM stays pure and unit testable.

