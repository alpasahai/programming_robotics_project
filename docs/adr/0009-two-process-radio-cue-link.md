# ADR-0009: Two-Process Radio Cue Link

**Status:** Accepted
**Builds on:** ADR-0007

## Decision

The Search Radar runs as its own Webots controller process and hands ATLAS one
chosen world-frame cue per step over an `Emitter`/`Receiver` radio link. The
link uses channel 1, `type "radio"`, and unlimited range (`range -1`).

Target selection lives inside the Search Radar process. The ATLAS controller
receives only the latest cue position through `SearchRadarLink`; it does not
receive `Detection` objects, `track_id`s, or Webots node handles.

## Context

ADR-0007 grounded the Search Radar as a physical Webots node but kept the radar
model in-process inside `atlas_controller`. The revised architecture gives the
Search Radar its own controller. That matches the scene topology and keeps
wide-area acquisition separate from ATLAS fire control.

The radio payload is intentionally small: three packed doubles representing the
selected target's world position. If no target is selected, the Search Radar
sends nothing. `SearchRadarLink` retains the last received cue until a newer
packet arrives.

## Consequences

- There is a one-step delivery lag between Search Radar selection and ATLAS
  consumption.
- `atlas_controller` no longer fuses Search Radar measurements into
  `TrackFilter`; `track_filter.update_search()` is removed.
- `TrackFilter` is now FCR-only.
- `SEARCH` and `ACQUIRE` are cue-driven. `SEARCH` waits for consecutive cue
  frames; `ACQUIRE` slews the turret toward the cue until the turret-aim FCR
  locks.
- The process boundary is part of the sensor membrane: target identity and
  Search Radar track management do not cross into ATLAS.
