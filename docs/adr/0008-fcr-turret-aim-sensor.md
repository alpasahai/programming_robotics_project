# ADR-0008: FCR as Turret-Aim Sensor

**Status:** Accepted
**Supersedes:** ADR-0004

## Decision

The Fire-Control Radar is the ATLAS turret itself: a permanently simulated,
narrow-beam sensor whose boresight is the turret's commanded aim. It has no
independent target selection or boresight state. Each update, the controller
passes the turret aim into `FireControlRadar.update(azimuth, elevation)`, and
the FCR detects the projectile only when it lies inside the configured FOV cone
and range limit.

The FCR still reads the projectile node inside the sensor membrane and returns a
low-noise turret-relative measurement. `FireControlRadar.set_target()` is
removed.

## Context

ADR-0004 accepted a simulated FCR as a temporary placeholder and left a future
upgrade path to a real Webots `Radar` or `RangeFinder`. That path does not fit
this project increment: Webots `Radar` does not provide elevation in the form
the ATLAS engagement loop needs, and the turret body is already the natural
physical carrier for the narrow-beam fire-control sensor.

The revised design makes FCR lock depend on real geometry and real turret motor
travel. Search Radar cues slew the turret toward a world-frame position; the FCR
locks only once the turret aim brings the projectile into the narrow cone.

## Consequences

- `FireControlRadar.update()` takes the turret aim every step.
- Detection is gated by angular separation from the turret aim and by maximum
  range.
- `ACQUIRE` has real duration: it waits while the turret slews and only reaches
  `TRACK` after the FCR locks and the track filter is initialised.
- The FCR remains simulated, but the accepted upgrade path is no longer a
  generic Webots `Radar` replacement.
