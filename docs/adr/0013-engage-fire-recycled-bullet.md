# ADR-0013: Engage-fire — a recycled, supervisor-driven bullet

**Status:** Accepted

## Decision

On entering the FSM `ENGAGING` state the turret fires a small, fast **Bullet** at
the `BallisticTrajectoryPredictor`'s intercept point; the bullet destroys the
incoming `Projectile` on contact.

- The Bullet is a passive `Solid` (`AtlasBullet.proto`, `DEF ATLAS_BULLET`),
  **pre-placed once** and **recycled** by the `atlas_controller` supervisor
  (park → fire → park), mirroring the incoming `Projectile` (ADR-0011). It
  carries no controller, sensor, or radio.
- The FSM emits a one-shot **fire command** (the intercept) on the `→ ENGAGING`
  edge and spawns nothing itself — staying pure and unit-testable.
- Hit detection stays where it already is: the `Projectile` classifies a mid-air
  contact as a bullet hit (`getContactPoints`, by world-Z). The attacker emits a
  **bullet-hit pulse** on channel 3; a new `BulletHitLink` carries it to the FSM,
  which adds a "projectile destroyed → RESET" exit.

## Context

The system already has a `BallisticTrajectoryPredictor` and a `TRACK_PREDICT`
state computing an intercept ahead of the target. That pipeline only earns its
place if the weapon has travel time — an instant laser would aim at the ball's
current position and make the predictor dead code.

## Reasoning

**Recycle, not spawn/delete:** `node.remove()` can leave stale `getFromDef`
handles (cyberbotics/webots#2123), and dynamic import spawns a controller process
per shot and cannot expose new devices cleanly (#6378). Recycling one node keeps
a stable handle and matches the established `Projectile` pattern (ADR-0011).

**Passive Solid, not a self-reporting Robot:** A non-supervisor Robot cannot
`setVelocity`/teleport itself (supervisor-only), so it could not recycle itself.
A `bumper` TouchSensor is boolean-only, readable only by the owning controller,
and needs a radio relay to share — adding latency and a second hit source. So the
bullet stays passive and the supervisor drives it.

**Ball is the single source of hit truth:** The `Projectile` already
disambiguates ground vs. bullet hits by world-Z, radius-independently. Reusing it
(emitting on channel 3, mirroring the ground-hit cue on channel 2, #32) avoids
the double-counting a bullet-side detector would reintroduce.

**FSM emits, controller executes:** Node motion is Webots-specific and
untestable; keeping it in the controller and having the FSM emit a plain
`[dx, dy, dz]` keeps the FSM pure.

## Consequences

- New: `protos/AtlasBullet.proto`, `controllers/atlas_controller/bullet.py`,
  `controllers/atlas_controller/bullet_hit_link.py`. `Attacker.proto` /
  `AtlasTurret.proto` gain a channel-3 emitter/receiver; the world pre-places
  `DEF ATLAS_BULLET` and a non-bouncy `ContactProperties` pair.
- The FSM gains `consume_fire_command()` and a bullet-hit RESET exit (resolving
  the prior `_do_engage` TODO); `SensorSuite` gains `bullet_hit_link`.
- This supersedes the 2026-05-18 turret-weapon spec/plan (self-reporting Robot +
  `customData` polling + dynamic spawn), which predated the FSM restructure
  (ADR-0012) and the ground-hit single-source work (#32).
- Known limitations: **lead/flight-time mismatch** — the fixed-lookahead
  intercept and the bullet's longer flight time must be co-tuned via
  `MUZZLE_SPEED`/`lookahead_steps`; a time-of-flight-aware intercept is future
  work. **Tunnelling** is mitigated (capped speed, generous radius), not solved.
  **One bullet at a time**; a pool is future work.
