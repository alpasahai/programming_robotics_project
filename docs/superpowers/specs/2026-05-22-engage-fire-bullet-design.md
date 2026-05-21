# Engage-fire: a recycled, radio-linked bullet — design (WIP)

**Status:** DRAFT — design in progress (research → synthesis → spec).
**Date:** 2026-05-22
**Tracking issue:** #6 — feature(atlas): destroy projectiles on engagement.
**Supersedes (planned):** `2026-05-18-turret-weapon-design.md` and its plan
`2026-05-18-turret-weapon.md`, which predate the FSM restructure (PR #35) and
use dynamic spawn + `customData` polling. This design revisits both.

## Goal

On the FSM `ENGAGING` state, the FCR turret releases a small, fast bullet aimed
at the predicted intercept point of the incoming projectile, and the bullet
destroys the projectile on contact.

## Open design questions (being researched before the spec is finalised)

1. **Bullet lifecycle: recycle vs. dynamic spawn.** The incoming `Projectile`
   is *recycled* (parked out of range, then teleported back) rather than
   deleted, to keep `getFromDef` handles valid (ADR-0011). Should the bullet
   follow the same recycle pattern (parked inside/under the turret) instead of
   the spawn/`remove()` pattern in the 2026-05-18 spec?
2. **Hit-detection ownership.** The `Projectile` already classifies a mid-air
   contact as a bullet hit via its own `getContactPoints`. Should the bullet
   *also* detect the hit (TouchSensor + Emitter), and if so, who is the source
   of truth? (Mirrors the ground-hit "single source of truth" decision, #32.)
3. **Radio topology.** Proposed: bullet has a Receiver (fire command in), an
   Emitter (projectile-hit out), and a Receiver for the projectile-ground-hit
   cue (→ recycle). How many channels, and how does this fit the existing
   two-process radio link (ADR-0009) and ground-hit link (#32)?

These are resolved by the research pass below before any code is written.
