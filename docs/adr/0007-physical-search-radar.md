# ADR-0007: Physical Search Radar

**Status:** Accepted, revised by ADR-0009

## Decision

The Search Radar is implemented as a dedicated Webots node with a world pose, a vertical-FOV gate, and a rotating scan beam. A track buffer to bridge beam sweeps is part of this increment and has been implemented.

This ADR originally kept the radar model in-process inside the ATLAS controller. ADR-0009 revises that part: the Search Radar now runs in its own controller process and emits a selected world-frame cue over a radio link.

## Context

Previously, both the Search Radar and Fire-Control Radar were co-located at the turret origin with no field-of-view. The Search Radar detected every projectile omnisciently, regardless of geometry or position.

The task "Physical Search Radar" required grounding the radar's pose in the world so that detection became geometric — scoped by the radar's position and field-of-view — rather than omniscient.

## Reasoning

**Pose grounding:** A dedicated Webots node gives the radar a visually clear world pose and makes it obvious where it is and what it can see. This supports visual verification and future multi-radar scenarios.

**Vertical-FOV gate and rotating scan beam:** Detection is now intermittent at the sensor level — the rotating beam sweeps past targets rather than always seeing them. This makes the physics more realistic and prevents aliasing of fast-moving projectiles.

**Track buffer:** A small track buffer bridges beam sweeps by holding detections from recent timesteps, so targets do not vanish and reappear with each beam pass. This smooths the output without baking smoothing into the filter itself. The `track_timeout` parameter is accepted at construction and is active: the buffer ages tracks each `update()`, refreshes a re-detected track to age 0, and drops any track once `age > track_timeout`.

**Process model:** The original in-process model reduced coupling during the first physical-radar increment. ADR-0009 supersedes that choice with a separate Search Radar controller process and radio cue link.

**Cue output:** `Detection` and `track_id` state now stay inside the Search Radar process. The value crossing into ATLAS is a selected world-frame cue, not a turret-relative measurement.

## Consequences

- `SearchRadar` now requires construction parameters: `radar_position` (world-frame `[x, y, z]` of the antenna phase centre), `max_range` (detection range gate in metres), `vertical_fov` (full vertical field of view in radians; gating is `|elevation| ≤ vertical_fov / 2`), `beam_width` (half-power beam width in radians), `scan_rate` (beam advance per update in radians), and `track_timeout` (number of consecutive missed updates before a track is dropped).
- Detections are intermittent at the sensor level — targets are detected only when the rotating beam illuminates them. A track buffer bridges those gaps; the `track_timeout` parameter controls how many updates a track survives without re-detection.
- A new `geometry.py` module of shared world-frame angle helpers was introduced: `azimuth_elevation_range()` (returns an `(azimuth, elevation, range)` triple) and `angle_diff()` (shortest signed angular difference). These compute world-frame geometry before the turret-relative transformation.
- ADR-0009 moves the radar model to its own Webots controller process and replaces turret-relative downstream `Detection` consumption with a world-frame radio cue.
