# Engage-fire: a recycled, supervisor-driven bullet — design

**Status:** Approved design — ready for implementation planning.
**Date:** 2026-05-22
**Tracking issue:** #6 — feature(atlas): destroy projectiles on engagement.
**Supersedes:** `2026-05-18-turret-weapon-design.md` and its plan
`2026-05-18-turret-weapon.md` (both predate the FSM restructure in PR #35 and
the ground-hit single-source-of-truth work in #32). See "Divergences" below.

## Goal

On the FSM `ENGAGING` state, the FCR turret releases a small, fast **Bullet**
aimed at the `BallisticTrajectoryPredictor`'s intercept point. The Bullet
destroys the incoming `Projectile` on contact, and the engagement resolves back
to `IDLE`.

## How this design was reached

Three expert research passes (Webots radio + node lifecycle; Webots collision
detection; fire-control software architecture) were run and synthesised. Key
Webots facts that shaped the design, with sources:

- A **non-supervisor Robot cannot move itself** — `setVelocity` and
  `translation.setSFVec3f` are supervisor-only. A self-recycling bullet is
  therefore impossible; the `atlas_controller` supervisor must own all bullet
  motion. (Webots Supervisor reference.)
- A `bumper` **TouchSensor can only be read by the owning Robot's controller**,
  returns a bare `1.0/0.0`, and cannot identify *what* it hit. Sharing that
  across processes needs an Emitter/Receiver relay — extra latency and a second,
  conflicting hit source. (Webots TouchSensor reference; device-ownership note.)
- `node.remove()` can leave **stale `getFromDef` handles** cached
  (cyberbotics/webots#2123); dynamic `importMFNodeFromString` **spawns a new
  controller process per shot** and cannot expose newly-imported devices
  cleanly (#6378). Recycling one pre-placed node sidesteps both.
- ODE (Webots' physics) uses **discrete collision only — no CCD**. A fast small
  bullet can tunnel through the ball in one step. The only knobs are bullet
  radius, muzzle speed, and `basicTimeStep`.
- `resetPhysics()` **zeroes velocity**, so it must never run in the same step as
  the launch `setVelocity()` — the existing `Projectile` already documents this.

## Design

### Architecture in one line

The **`Projectile` senses the hit** (already true), the **FSM decides to fire**
(emits a one-shot fire command on `→ ENGAGING`), the **`Bullet` lifecycle class
executes the shot** (recycle: park → fire → park), the **`BulletHitLink` carries
the destroyed signal in**, and `atlas_controller` is **cadence glue**. This is
the project's existing pure-FSM / lifecycle-class / `*Link`-wrapper / thin-glue
decomposition (ADR-0011, ADR-0012, #32), applied unchanged.

### 1. The Bullet is a recycled, passive `Solid` — `protos/AtlasBullet.proto`

A small sphere `Solid` with `Physics`, a spherical `boundingObject`, and
`DEF ATLAS_BULLET`, **pre-placed** in the world parked out of range. No
controller, no sensor, no radio, `supervisor FALSE`. Visually distinct (blue)
from the red `Projectile`.

- **Radius `0.15 m`, mass `0.05 kg`** (small but anti-tunnel-safe — see §6).
- Parked at e.g. `[0, 0, -100]` (below the floor, out of every sensor's range),
  exactly like the despawned `Projectile`.
- Obtained once at startup via `getFromDef("ATLAS_BULLET")`; the handle stays
  valid for the whole run (the reason for recycle, per ADR-0011).

### 2. `Bullet` lifecycle class — `controllers/atlas_controller/bullet.py`

A two-state recycle machine mirroring `Projectile`, over the Webots node
interface (`setVelocity` / `getField("translation")` / `resetPhysics`), so it is
unit-tested with the same stub style as `test_projectile.py` — zero Webots.

```
PARKED    — out of range, waiting to fire
IN_FLIGHT — launched, travelling toward the intercept
```

- `fire(intercept_rel)` — compute the muzzle world position (turret origin +
  `MUZZLE_OFFSET_M` along the unit vector to the intercept), teleport the node
  there, then `setVelocity(unit * MUZZLE_SPEED)`. **No `resetPhysics` this
  step** (it would zero the launch velocity). Enter `IN_FLIGHT`.
- `recycle()` — `setVelocity([0]*6)`, teleport to the park position,
  `resetPhysics()` (safe now: velocity already zeroed this step). Enter `PARKED`.
- `step()` — increment a flight-age counter; expose `age_steps` for the
  controller's safety timeout. The `Bullet` class itself owns **no** hit
  decision — recycling is triggered by the controller from the cues below.

`intercept_rel` is turret-relative `[dx, dy, dz]`, the same frame as the FSM's
`_intercept` and fire command.

### 3. FSM fire command — `controllers/atlas_controller/fsm.py`

One-shot, attribute + consume (matches CONTEXT.md's "Fire Command" entry and the
FSM's existing one-shot conventions). The FSM stays pure — it produces a plain
value and touches no Webots node.

- `self._fire_command: list[float] | None`, initialised `None`.
- Set inside `_transition()` on the `→ ENGAGING` edge to `list(self._intercept)`
  — the single place that edge is taken, so it fires exactly once per
  engagement. `_intercept` is guaranteed set (TRACK_PREDICT sets it before the
  transition).
- `consume_fire_command() -> list[float] | None` — returns the pending command
  and clears it; `None` thereafter until the next `→ ENGAGING`.
- Cleared to `None` on entering `RESET` (so a command never survives unconsumed)
  and in `_do_reset()`.

### 4. Bullet-hit cue — single source of truth on the attacker

The `Projectile` already classifies a mid-air contact as a bullet hit
(`getContactPoints`, Z-height) and fires the dormant `on_bullet_hit` callback.
We make that callback **emit a one-shot pulse**, mirroring the ground-hit cue
exactly (#32):

- `attacker_controller.on_bullet_hit(count)` → `bullet_emitter.send(struct.pack("i", count))`
  on **channel 3**.
- New `ATTACKER_BULLET_EMITTER` (channel 3, `type "radio"`, `range -1`) in
  `protos/Attacker.proto`.
- New `ATLAS_BULLET_HIT_RECEIVER` (channel 3) in `protos/AtlasTurret.proto`.
- New `BulletHitLink` — `controllers/atlas_controller/bullet_hit_link.py` — a
  near-clone of `AttackerGroundHitLink`: drains the queue each `update()`,
  exposes `hit_this_step()` and `count`. Unit-tested with the same 3-method
  receiver stub.

The bullet **never reports its own hit** — this avoids the double-counting #32
deliberately removed, and reuses the ball's radius-independent Z-height
classifier that already works.

### 5. Wiring + lifecycle — `controllers/atlas_controller/atlas_controller.py`

Thin glue. Per step, after `fsm.step()`:

1. **Fire:** `cmd = fsm.consume_fire_command()`; if `cmd is not None` and the
   bullet is `PARKED`, `bullet.fire(cmd)`. (One bullet at a time; a fire command
   while `IN_FLIGHT` is logged and ignored.)
2. **Recycle:** call `bullet.recycle()` when the engagement ends — i.e. when
   `bullet_hit_link.hit_this_step()` (we hit the ball), **or**
   `ground_hit_link.hit_this_step()` (the ball landed; the shot missed), **or**
   `bullet.age_steps >= BULLET_MAX_LIFETIME_STEPS` (safety timeout for a bullet
   that escaped/stuck). This realises the user's "bullet respawns when the
   projectile hits the ground" intent — supervisor-driven, not bullet-driven.
3. `BulletHitLink.update()` is drained in the Sense block alongside the existing
   `ground_hit_link.update()`.

`BulletHitLink` is added to `SensorSuite` so the FSM can read it (§6 below).

### 6. FSM RESET on projectile-destroyed (resolves the existing TODO)

`_do_engage` currently exits to `RESET` only on the ground-hit cue or out-of-
range. Add the bullet-hit cue as a co-equal exit (both mean "engagement over"):

```python
if (self.sensors.ground_hit_link.hit_this_step()
        or self.sensors.bullet_hit_link.hit_this_step()
        or not self._target_within_range(target_position)):
    self._transition(self.RESET)
```

The bullet-hit cue is also honoured as a precedence RESET in IDLE / AIM /
TRACK_PREDICT, exactly as the ground-hit cue already is, so a late pulse can't
strand the FSM. The FSM never learns *which* body was struck — it only sees
"destroyed this step," preserving the FSM-makes-no-physics-decisions rule.

### 7. Anti-tunneling parameters (atlas_controller constants)

ODE is discrete; the governing constraint is `v_max < (r_ball + r_bullet) / Δt`.
With `r_ball = 0.5`, `r_bullet = 0.15`, `Δt = 32 ms`: `v_max ≈ 20 m/s`.

```python
MUZZLE_SPEED = 18.0            # m/s — below the ~20 m/s tunnelling cap (tune in Webots)
MUZZLE_OFFSET_M = 0.6          # m — spawn clear of the turret bounding box
BULLET_PARK_POSITION = [0, 0, -100.0]
BULLET_MAX_LIFETIME_STEPS = 400  # safety: recycle a bullet that never resolves
```

`ContactProperties { bounce 0 bounceVelocity 0 }` in `WorldInfo` so the bullet
does not elastically knock the ball away before it despawns. All four constants
are **iterative tuning** — the orchestrator/human tunes them in Webots, never a
cheap subagent.

## Known limitations / risks

- **Lead/flight-time mismatch (important).** The intercept is a *fixed*
  `lookahead_steps` (0.32 s) ahead, but the bullet's flight time to that point
  is longer. `MUZZLE_SPEED` and `lookahead_steps` must be **co-tuned** so the
  bullet arrives roughly when the ball does. A proper time-of-flight-aware
  intercept (solve for the meeting time) is **future work** — it would make the
  hit robust instead of tuned. Tracked as a follow-up.
- **Tunneling is mitigated, not solved** — capped speed + generous radius. A
  swept/proximity backstop is future work.
- **One bullet at a time** — a bullet pool for rapid fire is future work.

## Divergences from the 2026-05-18 turret-weapon spec (discarded)

1. **Self-reporting bullet `Robot` + `bumper` TouchSensor + `atlas_bullet_controller`
   + `customData` polling** → discarded. The ball is the hit source of truth
   (#32); the bullet is a passive `Solid` with no controller. *(This is the main
   divergence from the feature request's "bullet_controller with receiver +
   emitter" sketch — see "How this design was reached" for the Webots reasons.)*
2. **Dynamic `importMFNodeFromString` spawn + `node.remove()`** → discarded for
   recycle (ADR-0011 handle stability; no per-shot controller process).
3. **Bullet-side Z-height hit disambiguation** → discarded (the `Projectile`
   already does it).
4. **`laser_active` flag** → already gone (ADR-0012: entering ENGAGING *is*
   ready-to-fire). The fire command keys off the ENGAGING entry edge.

## Files

| File | Status | Responsibility |
|---|---|---|
| `controllers/atlas_controller/fsm.py` | modify | Fire command (set on `→ENGAGING`, `consume_fire_command()`); bullet-hit RESET exit. |
| `controllers/atlas_controller/tests/test_fsm.py` | modify | Fire-command + bullet-hit-RESET tests (with a `StubBulletHitLink`). |
| `controllers/atlas_controller/bullet.py` | create | `Bullet` recycle lifecycle class. |
| `controllers/atlas_controller/tests/test_bullet.py` | create | Unit tests for `Bullet` (node stub). |
| `controllers/atlas_controller/bullet_hit_link.py` | create | `BulletHitLink` receiver wrapper (channel 3). |
| `controllers/atlas_controller/tests/test_bullet_hit_link.py` | create | Unit tests (receiver stub). |
| `controllers/atlas_controller/atlas_controller.py` | modify | Wire bullet + ch-3 receiver + `BulletHitLink`; fire + recycle in the loop. |
| `controllers/attacker_controller/attacker_controller.py` | modify | Emit the bullet-hit pulse on channel 3 from `on_bullet_hit`. |
| `protos/AtlasBullet.proto` | create | The recycled `Solid` bullet (currently `# PLACEHOLDER`). |
| `protos/Attacker.proto` | modify | Add `ATTACKER_BULLET_EMITTER` (channel 3). |
| `protos/AtlasTurret.proto` | modify | Add `ATLAS_BULLET_HIT_RECEIVER` (channel 3). |
| `worlds/ATLA_v1.wbt` | modify | Pre-place `DEF ATLAS_BULLET AtlasBullet`; add `ContactProperties`. |
| `controllers/atlas_controller/tests/test_world_proto_layout.py` | modify | Regression assertions for the bullet proto + world wiring. |
| `docs/adr/0013-engage-fire-recycled-bullet.md` | create | Records the decision. |
| `CONTEXT.md` | modify | Rewrite the `Bullet` entry (Solid, recycled, supervisor-driven; not a self-reporting Robot). |
