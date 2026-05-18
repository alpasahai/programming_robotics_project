# FCR Rework + Cue Handoff — Revised Design (Increment 2)

**Date:** 2026-05-18
**Issue:** #8 — perception from radar simulation needs design refinement
**Status:** Approved design — ready for implementation planning
**Supersedes:** the Increment 2 design in `2026-05-17-radar-perception-design.md`
and the plan `2026-05-17-fcr-rework-and-cue-handoff.md`.

## Why this revision exists

The original Increment 2 design and plan were written on 2026-05-17. Two
changes since then invalidate parts of them:

1. **`search_radar_controller` now exists as a separate Webots controller
   process.** The original plan assumed the `SearchRadar` model ran *in-process*
   inside `atlas_controller`, so the FSM could call `search_radar.get_detections()`
   directly. With the Search Radar running as its own `Robot`/controller, that
   in-process call is impossible — the cue must cross a process boundary. The
   original plan explicitly *deferred* "the Search Radar's own controller process
   + Emitter→Receiver radio link"; that deferral is now void.

2. **PR #18 reframed the FCR.** `AtlasTurret.proto` now carries the comment
   `# The fire-control radar body`: **the ATLAS turret *is* the fire-control
   radar.** There is no separate FCR pedestal. The turret already slews
   physically via its `PAN_MOTOR` / `TILT_MOTOR`, driven by the FSM every step.
   The original plan's Increment-2 FCR — a standalone tracker with its own
   *Python-modelled* boresight that slews at its own rate — is a fiction that
   would drift out of sync with the real motors. It is replaced here.

The target architecture in `2026-05-17-radar-perception-design.md` (separate
bodies, two controllers, radio cue link, FOV gating) still holds. This document
revises only *how* Increment 2 realises it.

## Decisions

1. **The turret is the FCR.** The `FireControlRadar` class is kept as the
   narrow-beam precise sensor, but it has **no internal boresight and no slew
   loop**. Its boresight *is* the turret's aim direction, passed into
   `update(boresight_az, boresight_el)` each step. The FSM already computes
   these as `pan` (= azimuth) and `tilt` (= elevation).

2. **The FCR gates detection by an FOV cone around the turret's aim.** The class
   gains a fixed `fcr_position` (the turret position), an `fov_half_angle`, and
   a `max_range`. Each `update()`: compute the projectile's az/el/range from
   `fcr_position`; the target is detected only when the angular separation
   between the turret aim and the target direction is `≤ fov_half_angle` **and**
   `range ≤ max_range`. When detected, store a noisy turret-relative position
   and report `is_locked()` true; otherwise store `None`.

3. **The cue drives the FSM, not the radar.** The Search Radar cue is a world
   position. It does *not* aim the FCR (the FCR has nothing to aim). It tells
   the FSM where to slew the *turret*. `set_target(node)` and any `cue()` method
   on the FCR are removed.

4. **`ACQUIRE` gets real duration for free.** When a cue arrives, `ACQUIRE`
   commands the turret motors toward the cue bearing. The real motors slew at
   their `maxVelocity`; the FCR cone sweeps onto the target as they move. The
   transition `ACQUIRE → TRACK` is gated on `fcr.is_locked()`. The acquisition
   delay is genuine physical motor travel time — no phantom `slew_rate`
   parameter. Tuning knobs are the motor `maxVelocity` and `fov_half_angle`.

5. **The Search Radar process owns detection *and* target selection.** It emits
   one chosen cue — a single target world position — per step while it holds a
   locked track, and nothing otherwise. The SEARCH-state "pick a target" logic
   moves out of the FSM into the Search Radar process. `track_id` and the
   `Detection` membrane (ADR-0006) live wholly inside that process; no track
   identity crosses the radio link.

6. **`atlas_controller` stops fusing Search Radar measurements.** With the
   in-process `SearchRadar` gone, `track_filter.update_search()` is removed.
   This matches ADR-0003's handoff design: the Search Radar *cues*, it does not
   continuously fuse. Only the FCR feeds the `TrackFilter`. `TrackFilter` is
   modified this increment — but the change is deletion-only, no behavioural
   rewrite.

7. **Shared code lives in a repo-root `lib/`.** `geometry.py` is needed by both
   controllers (the FCR and the Search Radar both do azimuth/elevation math). It
   moves to a top-level `lib/`, made importable by extending `PYTHONPATH` in each
   controller's `runtime.ini` (mirrored into `runtime.ini.example`).

## Architecture

Two Webots controller processes, both Supervisors, communicating over an
`Emitter`/`Receiver` radio link.

```
search_radar_controller (Supervisor)        atlas_controller (Supervisor)
  reads PROJECTILE world position             reads PROJECTILE world position
  SearchRadar: FOV gate + track buffer        SearchRadarLink: drains Receiver
  selects one target track                    FireControlRadar: FOV-cone gate
  Emitter.send(struct("ddd", x,y,z))   ──►     around the turret's aim
                                  radio        FSM / TrackFilter / predictor
```

- **Link payload:** one world-frame target position per step,
  `struct.pack("ddd", x, y, z)`, on a dedicated channel (`type "radio"`,
  `range -1`). Webots delivers an Emitter packet to the Receiver one step later
  — a one-step lag, acceptable at radar rate.
- The `Emitter` is a device node in `SearchRadar.proto`'s `Robot` children; the
  `Receiver` is a device node in `AtlasTurret.proto`'s `Robot` children. They
  pair by matching `channel`.
- No new `FireControlRadar.proto` — the FCR body already exists as
  `AtlasTurret.proto`.

## Components

### `FireControlRadar` (atlas_controller)

The narrow-beam precise sensor. No boresight state, no slew loop.

- Construction: `fcr_position`, `turret_position`, `fov_half_angle`,
  `max_range`, `noise_std`, `rng`.
- `update(boresight_az, boresight_el)` — the turret's current aim. Gates the
  projectile by FOV cone + range; stores a noisy turret-relative position or
  `None`.
- `get_target_position() -> [dx,dy,dz] | None`, `is_locked() -> bool`.
- `set_target` removed.

### `SearchRadar` (moves into search_radar_controller)

Unchanged in behaviour from Increment 1 — FOV/range/beam gating and the
`track_id`-keyed track buffer. It simply moves directory: it is now imported by
`search_radar_controller.py`, not `atlas_controller`.

### `search_radar_controller.py` (rewritten)

A Supervisor. Reads the projectile node, runs `SearchRadar`, selects one live
track (the first, as the FSM did), and `Emitter.send()`s that track's world
position each step it holds a target.

### `SearchRadarLink` (new, atlas_controller)

Wraps the `Receiver`. `update()` drains the queue keeping the last packet;
`get_cue() -> [x,y,z] | None` returns the most recent cue. Unit-tested with a
stub receiver.

### FSM changes

- `SensorSuite.search_radar` → `cue_link` (a `SearchRadarLink`).
- `SEARCH` — waits for a cue from the link; idles or sweeps until one arrives.
- `ACQUIRE` — commands the turret toward the cue bearing each step; the FCR is
  fed the turret aim; transitions to `TRACK` when `fcr.is_locked()` (and the
  filter is initialised).
- The `track_id` / `set_target` path is removed — the FSM consumes a bare
  position.

## Data flow (per timestep)

1. **Search Radar controller** — advance the scan beam; gate the projectile;
   if a track is live, `Emitter.send()` its world position.
2. **ATLAS controller**
   - `cue_link.update()` drains the Receiver.
   - `fcr.update(turret_aim)` — FOV-cone gate around the turret's current aim.
   - `track_filter.predict()`, then `track_filter.update_fcr(...)` only when the
     FCR is locked. `update_search()` is no longer called.
   - `fsm.step()` — `SEARCH`/`ACQUIRE` consume the cue; `ACQUIRE` slews the
     turret; later states unchanged.

## Documentation consequences

- **ADR-0008 (new)** — the FCR is the turret itself: a narrow-beam sensor whose
  boresight is the turret aim, FOV-cone gated, permanently simulated. Supersedes
  ADR-0004 (the real-Webots-`Radar` upgrade path is rejected — no elevation
  output).
- **New ADR** — the two-process radio cue link (Emitter/Receiver, one chosen cue
  per step).
- **ADR-0006** — revised: the sensor membrane now spans a process boundary; the
  `Detection`/`track_id` discipline lives inside the Search Radar process and no
  node handle or track identity crosses the link.
- **ADR-0003** — the two files numbered 0003 are resolved, landing on the
  handoff design (Search Radar cues; FCR tracks).
- **CONTEXT.md** — the Search Radar and FCR entries are rewritten.

## Out of scope

- `TrackFilter` migration to world frame — still deferred; each radar converts
  to turret-relative internally.
- The fired-projectile weapon, attacker, referee, and ML detector from
  `2026-05-18-attack-pattern-and-ml-detection-design.md` — that spec is marked
  future work and is not part of this increment.
- Multi-projectile tracking.
