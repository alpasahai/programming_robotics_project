# ADR-0007: Physical Search Radar

**Status:** Accepted

## Decision

The Search Radar is implemented as a dedicated Webots node with a world pose, a vertical-FOV gate, a rotating scan beam, and a track buffer. The radar model runs in-process inside the ATLAS controller. Giving the radar its own Webots controller and a radio link to the turret is deferred to a future increment.

The `Detection` output remains turret-relative (unchanged from the prior omniscient model) to keep the FSM and `TrackFilter` untouched.

## Context

Previously, both the Search Radar and Fire-Control Radar were co-located at the turret origin with no field-of-view. The Search Radar detected every projectile omnisciently, regardless of geometry or position.

The task "Physical Search Radar" required grounding the radar's pose in the world so that detection became geometric — scoped by the radar's position and field-of-view — rather than omniscient.

## Reasoning

**Pose grounding:** A dedicated Webots node gives the radar a visually clear world pose and makes it obvious where it is and what it can see. This supports visual verification and future multi-radar scenarios.

**Vertical-FOV gate and rotating scan beam:** Detection is now intermittent at the sensor level — the rotating beam sweeps past targets rather than always seeing them. This makes the physics more realistic and prevents aliasing of fast-moving projectiles.

**Track buffer:** A small track buffer bridges beam sweeps by holding detections from recent timesteps, so targets do not vanish and reappear with each beam pass. This smooths the output without baking smoothing into the filter itself.

**In-process model:** Running the radar simulation in the ATLAS controller keeps it in the same Python process and reduces coupling. A future increment (deferred) can extract this into a separate Webots controller with a radio link if needed for architectural isolation or realism.

**Turret-relative output:** Keeping `Detection` turret-relative (not world-frame) avoids cascading changes to the FSM and `TrackFilter`, which expect positions in the turret's local frame. The coordinate transformation happens inside the SearchRadar class; downstream components remain untouched.

## Consequences

- `SearchRadar` now requires construction parameters: world pose (`x`, `y`, `z`), vertical FOV gate bounds (`fov_min_elevation`, `fov_max_elevation`), and scan-beam parameters (azimuth sweep rate, speed).
- Detections are intermittent at the sensor level — targets are detected only when the rotating beam illuminates them. This is bridged by the track buffer, which holds recent detections and outputs a smoothed stream.
- A new `geometry.py` module of shared world-frame angle helpers (`azimuth()`, `elevation()`, `euclidean_distance()`) was introduced to compute angles and ranges in the world frame before the turret-relative transformation.
- The FSM and `TrackFilter` remain untouched — they still receive turret-relative `Detection` values and have no visibility into world-frame geometry.
- World-frame `Detection` output and the radio link (separate Webots controller for the radar) remain future increments.
