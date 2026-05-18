# Turret weapon — a fired, self-reporting projectile

**Status:** Approved design — ready for implementation planning.
**Date:** 2026-05-18
**Supersedes:** Part 2 of `2026-05-18-attack-pattern-and-ml-detection-design.md`
(the concept capture). Parts 1, 3, 4, 5 of that document remain future work.

## Purpose

Make the ATLAS turret destroy the incoming ball by **firing a physical
projectile** along its aim direction, instead of relying on an instant-hit
laser. The projectile detects its own collisions with a real Webots sensor and
reports the hit back to the turret.

## Scope

In scope:

- `AtlasBullet.proto` — the turret's projectile, a self-reporting `Robot`.
- `atlas_bullet_controller` — a minimal controller that reads the bullet's
  collision sensor.
- A one-shot **fire event** in the FSM, emitted on entering `ENGAGING`.
- Bullet spawn, launch, and lifecycle management in `atlas_controller`.

Out of scope (future work, owned by other specs):

- Scoring and the referee supervisor (Part 3 of the concept-capture spec).
- The attacker and the incoming-ball launch pattern (Part 1).
- The incoming-ball recycling rework currently in flight — this design does
  **not** depend on or reference the current ball reset logic.
- A bullet pool / multiple bullets alive at once.

## Why a fired projectile (not an instant laser)

The system already has a `BallisticTrajectoryPredictor` and an FSM `PREDICT`
state that computes an intercept point `lookahead_steps` ahead. That prediction
pipeline only earns its place if the weapon has **travel time**: an instant
laser would aim at the ball's current position and make the predictor dead
code. A fired projectile must be *led*, so the predictor and intercept logic
become the core of the game.

## Component design

### 1. `AtlasBullet.proto` — a self-reporting projectile

A `Robot` node — not a plain `Solid` — because it carries a sensor and a
controller:

- A small sphere `Shape` with `Physics` and a spherical `boundingObject`,
  visually distinct from the red incoming ball.
- A `bumper`-type `TouchSensor` whose `boundingObject` matches the bullet. A
  `bumper` sensor returns `1` whenever its bounding object intersects another
  bounding object, `0` otherwise.
- `controller "atlas_bullet_controller"`, `supervisor FALSE`.

The proto is a **template only** — it is never pre-placed in the world. The
turret supervisor imports an instance at fire time.

### 2. `atlas_bullet_controller` — minimal collision reporter

A new controller under `controllers/atlas_bullet_controller/`. One job: each
timestep, read the `TouchSensor`; on the first contact, call
`robot.setCustomData("HIT")` and keep it set for the rest of the bullet's life.

Webots auto-starts this controller when the supervisor imports the bullet node,
and stops it when the node is removed.

### 3. FSM fire event

The FSM emits a one-shot **fire command** on entering `ENGAGING`:

- On the `→ ENGAGING` transition, the FSM records the intercept/aim and sets a
  `fire_command` attribute.
- The controller consumes the `fire_command` and clears it; the FSM does not
  re-fire until it cycles through `RESET` and reaches `ENGAGING` again (one shot
  per engagement).
- The FSM **spawns nothing** — it only produces the event. Node spawning is
  Webots-specific and stays in the controller, keeping the FSM pure and unit
  testable. The existing cosmetic `laser_active` flag and `AtlasLaser` node are
  left untouched.

### 4. Bullet spawn and launch — `atlas_controller`

`atlas_controller` is already a supervisor. When it consumes a `fire_command`:

- Computes a launch direction: the unit vector from the turret muzzle toward
  the intercept point.
- Spawns the bullet via `importMFNodeFromString` into the scene tree root, at
  the turret muzzle position.
- Calls `setVelocity()` on the new node along the launch direction scaled by a
  configurable `MUZZLE_SPEED` constant.

### 5. Bullet lifecycle — supervisor-owned

One bullet alive at a time (single shot per `ENGAGING`). Each step, while a
bullet is live, the supervisor:

- Reads the bullet node's `customData` field. `"HIT"` → resolve the shot.
- Disambiguates *what* was struck by the bullet's world-Z height: at/below the
  ground threshold → it hit the ground; otherwise → it hit the **ball**, logged
  as a hit signal for a future referee to consume.
- Either outcome → `node.remove()` the bullet.
- A safety timeout (a configurable max flight time in timesteps) removes any
  bullet that never reports, so bullets cannot accumulate.

## Known limitations

- **Tunneling.** A `bumper` `TouchSensor` only fires if the physics engine
  registers contact within a single timestep. A sufficiently fast bullet can
  pass through the ball between timesteps without reporting. Mitigated for now
  by capping `MUZZLE_SPEED` and sizing the bullet generously. Not solved.

## Future work

- **Replace `customData` with `Emitter`/`Receiver`** for the bullet → turret
  collision channel. `customData` is the minimal first cut; if time allows, a
  proper radio link is the cleaner design and removes the supervisor's reliance
  on polling a node field.
- A bullet pool for multiple shots in flight.
- Hardening against tunneling (e.g. swept-collision or a supervisor-side
  proximity backstop).
