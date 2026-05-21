# ADR-0011: Attacker Owns the Incoming Projectile, Recycled Not Deleted

**Status:** Accepted

## Decision

The incoming `Projectile` is owned by a separate offense supervisor, the
`Attacker` (`Attacker.proto`, DEF `ATTACKER`, controller `attacker_controller`).
The projectile itself is a passive `Solid` with no controller of its own; the
attacker drives it through the supervisor API.

On a **hit** the attacker **recycles a single node** rather than deleting and
re-importing one:

- **Detect** — query the ball's real contact points each step
  (`getContactPoints`, world frame) and classify by world-Z: a contact near the
  floor is a **ground hit**, a contact in the air is a **bullet hit**. Webots'
  contact `node_id` identifies the ball itself, not the other body, so height is
  the discriminator. An "airborne latch" requires the ball to clear the floor
  once before a floor contact counts, so the launch-time floor contact is not a
  false landing.
- **Register** — increment the relevant hit count and fire a callback (the seam
  for future scoring / the referee).
- **Despawn** — zero the ball's velocity and teleport it far out of every
  sensor's range/FOV.
- **Spawn** — after a respawn delay, teleport the ball back to the spawn point
  and relaunch it. resetPhysics() is done at despawn, never in the same timestep
  as the launch setVelocity() (which Webots would otherwise zero).

The spawn/despawn lifecycle is a two-state machine (`ACTIVE`, `DESPAWNED`) in the
unit-tested `Projectile` class; `attacker_controller` is a thin loop. The bullet
branch is dormant until the turret bullet exists.

## Context

Earlier the projectile's launch/relaunch lived inside `atlas_controller`. The
attack-pattern design doc establishes that incoming projectiles get no
controller of their own — a single attacker supervisor owns them — because
launch timing/locations and the ML launch log are cross-projectile concerns that
belong in one place. This also matches the existing scene topology, where
`atlas_controller` and `search_radar_controller` are already separate
supervisors that manipulate other nodes.

True node deletion (`node.remove()` + `importMFNodeFromString`) was rejected for
the first version: `search_radar_controller` and `atlas_controller` each resolve
`getFromDef("PROJECTILE")` once at startup and hold the handle. Deleting the node
would leave those handles dangling — the Search Radar tracks the node object, so
it would stop detecting entirely and break the sense → track → aim pipeline.
Recycling one node keeps every handle valid.

## Consequences

- The single `DEF PROJECTILE` node persists for the whole run; no dynamic
  spawning. A projectile *pool* and true dynamic spawning remain future work
  (see the attack-pattern design doc).
- "Despawn" is a teleport out of range, not a removal — the node still exists in
  the scene tree while parked.
- Ground hits are observable to other controllers/the future referee via the
  registration count/callback rather than via node lifecycle events.
- `atlas_controller` no longer manages the projectile; it only reads
  `getFromDef("PROJECTILE")` position as ground-truth telemetry.
