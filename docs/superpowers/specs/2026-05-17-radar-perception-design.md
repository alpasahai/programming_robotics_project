# Radar Perception Architecture Refinement

**Date:** 2026-05-17
**Issue:** #8 — perception from radar simulation needs design refinement
**Status:** Approved design — ready for implementation planning

## Problem

Issue #8 asks a question the current code cannot answer: *where are the Search
Radar and the Fire-Control Radar in space, relative to the robot?*

Today both radars call `projectile.getPosition()` and subtract a single
`turret_position`, so they are mathematically **co-located at the turret
origin**. Neither models a field of view — both "see" every projectile in the
scene regardless of direction or range. The only thing distinguishing them is
noise magnitude (0.2 m vs 0.02 m std).

This directly contradicts `CONTEXT.md`, which describes the Search Radar as
*"external to ATLAS... a higher-level external system"*. In the code it is
physically indistinguishable from the on-board FCR.

This spec refines the **perception layer only**: where each radar sits, how its
field of view is modelled, and how a target measurement is produced and
delivered. The weapon subsystem is deliberately out of scope (see below).

## Scope

**In scope**

- The Search Radar and the FCR as physically distinct, grounded entities.
- Field-of-view modelling for each.
- The cue handoff from Search Radar to FCR.
- The measurement interface feeding the existing `TrackFilter`.
- The coordinate-frame change this forces.
- The ADR and `CONTEXT.md` updates this forces.

**Out of scope** (each is its own future spec)

- The weapon subsystem (kinetic / laser, aiming, firing, hit detection).
- The Kalman-filter upgrade to `TrackFilter`.
- Multi-projectile tracking and engagement.

## Decisions

The design was settled through a brainstorming dialogue. The load-bearing
decisions, with rationale:

1. **Both radars are simulated sensor models, not Webots `Radar` device nodes.**
   The Webots `Radar` node was evaluated and rejected: its `WbRadarTarget`
   output carries `distance`, `received_power`, `speed`, `azimuth` — **no
   elevation**. It is a faithful *2D* radar. A projectile arcing through 3D
   space needs a full 3D fix, and working around the missing elevation (a
   slewing-pedestal trick, or a paired height-finder node) added more
   complexity than it removed. `radarsimpy` was also evaluated and rejected: it
   is an RF-baseband simulator for *designing radars* — the wrong abstraction
   level, it carries a licensing cost, and it does not integrate with Webots,
   so it would not even solve the grounding problem. Simulating the detection
   in Python keeps full 3D control and removes the elevation-handoff problem
   entirely.

2. **"Simulated" does not mean "ungrounded."** Each radar is a real Webots node
   with a true pose and a visible body. Only the *detection* is simulated: the
   controller reads the projectile's true position via the Supervisor,
   transforms it into the radar node's frame, applies an FOV test, adds noise,
   and emits a measurement. The node grounds the radar in space; the Python
   model produces the measurement.

3. **The Search Radar is a separate, physically grounded entity.** It is
   external to ATLAS, so it is its own Webots `Robot` node at a fixed surveyed
   location. *Target architecture:* its own controller process communicating
   with ATLAS over an Emitter → Receiver radio link. *First increment:* the
   `Robot` node carries no controller; the `SearchRadar` model runs in-process
   inside the ATLAS controller, which reads the node's pose directly. The cue
   is a value object crossing a narrow interface, so the radio link is a later
   transport-only swap. See "Implementation increments" below.

4. **The FCR is part of ATLAS and driven by the ATLAS controller.** It is a
   pan/tilt pedestal carrying a radar body. The ATLAS controller owns the node
   and drives it directly — no second radio link.

5. **The Search Radar emits a coarse 3D cue.** Because the radar is simulated,
   modelling elevation costs nothing, and a 3D cue lets the FCR be cued
   straight onto the target with no acquisition tilt-scan. This supersedes the
   earlier 2D-radar framing, which existed only to fit the Webots device.

6. **The FCR is a precise 3D slewing tracker.** Narrow FOV cone on a pan/tilt
   pedestal; cued by the Search Radar; closed-loop tracks once the target is
   inside its cone.

7. **World frame is the common coordinate frame.** Radars emit world-frame
   measurements; each consumer transforms into the frame it needs.

## Architecture

Three physical entities, two controllers:

| Entity | Webots representation | Controller | Role |
|---|---|---|---|
| Search Radar | Own `Robot` node, fixed surveyed location | Its own controller (a Supervisor) | External to ATLAS. Detects the projectile, broadcasts a coarse cue. |
| FCR | Pan/tilt pedestal + radar body, part of ATLAS | Driven by the ATLAS controller | On-board precise 3D tracking sensor. |
| Weapon | — | — | Deferred to the weapons spec. |

```
 [Search Radar Robot]              [ATLAS Robot]
   own controller                    ATLAS controller (Supervisor)
   - rotating scan                   - owns + drives FCR pedestal node
   - simulate coarse 3D detection    - simulate FCR cone detection
   - add coarse noise                - add low noise
        |                            - TrackFilter / predictor / FSM
        | Emitter  --- cue --->  Receiver
        |    {track_id, world_position}
```

### Search Radar — coarse acquisition radar

- Own `Robot` node at a fixed surveyed world location; own controller (a
  Supervisor, since it must read projectile ground truth).
- **Rotating scan beam:** an azimuth beam sweeps continuously. The projectile
  is detected only when the beam passes over it *and* it is within range *and*
  within the vertical FOV gate. Cues are therefore **intermittent**.
- Emits a **coarse 3D** world-frame position (high Gaussian noise) tagged with
  a `track_id`, over the Emitter.

### FCR — precise 3D slewing tracker

- A Webots node — pan/tilt pedestal + radar body — that is part of ATLAS and
  driven by the ATLAS controller.
- Narrow FOV **cone**. Cued by the Search Radar: the controller slews the
  pedestal pan+tilt toward the latest cue.
- Once the projectile is inside the FOV cone, the FCR detects it and switches
  to **closed-loop tracking**, keeping the target centred.
- Emits a **precise 3D** world-frame position with low Gaussian noise.
- If the target leaves the cone (lost track), the FCR falls back to the latest
  Search Radar cue to re-acquire.

## Data flow (per timestep)

1. **Search Radar controller** — advance the scan beam; read projectile truth;
   if the beam covers it (azimuth + range + vertical gate), add coarse noise
   and `Emitter.send()` a cue `{track_id, world_position}`.
2. **ATLAS controller**
   - `Receiver` drains any pending cues.
   - **Drive FCR:** if unlocked, slew the pedestal toward the latest cue;
     simulate FCR detection (transform projectile truth into the FCR frame,
     FOV-cone test); if inside, produce a precise 3D world-frame measurement.
   - **Fuse:** `TrackFilter` is fed the FCR's precise measurements.
   - **Predict → FSM →** (weapon: deferred).

## Coordinate frames

World frame is the common frame. Each radar computes detection in its own node
frame (obtained via `getPose()`), then emits world-frame positions.

This changes one existing assumption: today `TrackFilter` works in
*turret-relative* coordinates. It moves to **world frame**; the weapon performs
the world → turret conversion at aim time. The filter should not need to know
where the weapon sits.

## FSM impact

FOV gating is now real, so the existing FSM states gain genuine meaning:

- `SEARCH` — no cue yet; waiting on the Receiver.
- `ACQUIRE` — a cue arrived; the FCR is slewing toward it, not yet in-cone.
- `TRACK` — the projectile is inside the FCR cone; closed-loop tracking; the
  filter is fed by precise measurements.

No new states are introduced. Transition conditions change from "a measurement
exists" to "a cue was received" / "the FCR cone contains the target".

## Documentation consequences

- **ADR-0004** (FCR currently simulated, with an upgrade path to a real Webots
  `Radar` device) — the upgrade path is now explicitly rejected. Both radars
  stay simulated permanently. A new ADR supersedes ADR-0004.
- **ADR-0006** (sensor membrane / track-id) — the membrane stays disciplinary
  (simulation still reads ground truth). But there are now two processes, so
  `track_id` must be coordinated cross-process: the Search Radar owns ID
  assignment and ships the ID inside the cue. ADR-0006 needs a revision for
  cross-process identity.
- **ADR-0003** — two conflicting files are currently numbered 0003 (continuous
  fusion vs. handoff). This design lands firmly on **handoff**: the Search
  Radar cues (intermittent, coarse); the FCR tracks (continuous, precise). The
  duplicate must be resolved as part of this work.
- **CONTEXT.md** — the Search Radar and FCR glossary entries are rewritten to
  reflect the separate-body architecture, fixing the contradiction issue #8
  flagged.

New ADRs required:

- Radars permanently simulated (supersedes ADR-0004).
- Perception architecture: separate bodies, two controllers, radio cue link.
- Revision of ADR-0006 for cross-process track identity.
- Resolution of the duplicate ADR-0003.

## Implementation increments

The design above is the target architecture. It is delivered incrementally so
each step produces working, testable software and complexity is added against
clean seams rather than all at once.

- **Increment 1 — Physical Search Radar.** A grounded `Robot` node for the
  Search Radar, with FOV gating and a rotating scan beam, modelled in-process.
  Output stays on the existing `Detection` interface (turret-relative), so the
  FSM and `TrackFilter` are untouched. Plan:
  `2026-05-17-search-radar-physical-node.md`.
- **Increment 2 — FCR rework + cue handoff.** The FCR becomes a grounded,
  FOV-gated slewing tracker, cued by the Search Radar; the FSM's ACQUIRE/TRACK
  transitions reflect real FOV gating. Plan:
  `2026-05-17-fcr-rework-and-cue-handoff.md`.
- **Deferred to later increments** (designed as seams, not built yet): the
  Search Radar's own controller process and Emitter → Receiver radio link; the
  migration of `TrackFilter` from turret-relative to world frame. Until the
  frame migration lands, each radar converts its measurements to
  turret-relative internally, keeping `TrackFilter` and the FSM unchanged.

## Open question for the weapons spec

The FCR currently feeds the `TrackFilter`, predictor, and FSM, all of which
live in the ATLAS controller. When the weapon subsystem is designed, decide
whether the "fire-control computer" (filter + predictor + FSM) stays in the
ATLAS controller or becomes its own module. Out of scope here; flagged so the
follow-on spec picks it up.
