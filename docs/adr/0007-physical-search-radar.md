# ADR-0007: Physical Search Radar

**Status:** Accepted

## Decision

The Search Radar is implemented as a dedicated Webots node with a world pose, a vertical-FOV gate, and a rotating scan beam. A track buffer to bridge beam sweeps is part of the design for this increment and will be delivered in a follow-on task. The radar model runs in-process inside the ATLAS controller. Giving the radar its own Webots controller and a radio link to the turret is deferred to a future increment.

The `Detection` output remains turret-relative (unchanged from the prior omniscient model) to keep the FSM and `TrackFilter` untouched.

## Context

Previously, both the Search Radar and Fire-Control Radar were co-located at the turret origin with no field-of-view. The Search Radar detected every projectile omnisciently, regardless of geometry or position.

The task "Physical Search Radar" required grounding the radar's pose in the world so that detection became geometric — scoped by the radar's position and field-of-view — rather than omniscient.

## Reasoning

**Pose grounding:** A dedicated Webots node gives the radar a visually clear world pose and makes it obvious where it is and what it can see. This supports visual verification and future multi-radar scenarios.

**Vertical-FOV gate and rotating scan beam:** Detection is now intermittent at the sensor level — the rotating beam sweeps past targets rather than always seeing them. This makes the physics more realistic and prevents aliasing of fast-moving projectiles.

**Track buffer:** A small track buffer will bridge beam sweeps by holding detections from recent timesteps, so targets do not vanish and reappear with each beam pass. This will smooth the output without baking smoothing into the filter itself. The `track_timeout` parameter is accepted at construction but is not yet active — the buffering logic is deferred to a follow-on task.

**In-process model:** Running the radar simulation in the ATLAS controller keeps it in the same Python process and reduces coupling. A future increment (deferred) can extract this into a separate Webots controller with a radio link if needed for architectural isolation or realism.

**Turret-relative output:** Keeping `Detection` turret-relative (not world-frame) avoids cascading changes to the FSM and `TrackFilter`, which expect positions in the turret's local frame. The coordinate transformation happens inside the SearchRadar class; downstream components remain untouched.

## Consequences

- `SearchRadar` now requires construction parameters: `radar_position` (world-frame `[x, y, z]` of the antenna phase centre), `max_range` (detection range gate in metres), `vertical_fov` (full vertical field of view in radians; gating is `|elevation| ≤ vertical_fov / 2`), `beam_width` (half-power beam width in radians), `scan_rate` (beam advance per update in radians), and `track_timeout` (accepted at construction but deferred — not yet active).
- Detections are intermittent at the sensor level — targets are detected only when the rotating beam illuminates them. A track buffer to bridge those gaps is planned; the `track_timeout` parameter is stored but the buffering logic is not yet implemented.
- A new `geometry.py` module of shared world-frame angle helpers was introduced: `azimuth_elevation_range()` (returns an `(azimuth, elevation, range)` triple) and `angle_diff()` (shortest signed angular difference). These compute world-frame geometry before the turret-relative transformation.
- The FSM and `TrackFilter` remain untouched — they still receive turret-relative `Detection` values and have no visibility into world-frame geometry.
- World-frame `Detection` output and the radio link (separate Webots controller for the radar) remain future increments.
