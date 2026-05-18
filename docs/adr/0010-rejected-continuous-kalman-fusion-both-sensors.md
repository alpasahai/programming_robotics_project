# ADR-0010: Rejected Continuous Kalman Fusion of Both Sensors

**Status:** Rejected
**Superseded by:** ADR-0003, ADR-0008, ADR-0009

## Decision

Continuous Kalman fusion of Fire-Control Radar and Search Radar measurements is
not the accepted design. The accepted design is a hard acquisition handoff:
Search Radar cues ATLAS, the turret slews, the turret-aim FCR locks, and the
Track Filter consumes FCR measurements only.

## Context

This ADR was formerly numbered `0003`, which created a duplicate with
`0003-search-radar-handoff-not-continuous-fusion.md`. It also contradicted the
current two-process radio-link design: Search Radar no longer sends continuous
measurements to ATLAS, and `atlas_controller` no longer calls
`track_filter.update_search()`.

## Rationale

The continuous-fusion design was attractive for a generic sensor-fusion story,
but it does not match the revised architecture:

- the Search Radar process owns target selection and track identity;
- the radio link carries only one world-frame cue, not a measurement stream;
- the FCR is the turret-aim sensor and is the only source for precision
  tracking after lock.

The handoff design is recorded in ADR-0003. The turret-aim FCR is recorded in
ADR-0008. The two-process cue link is recorded in ADR-0009.
