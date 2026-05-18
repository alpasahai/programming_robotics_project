# ADR-0003: Search Radar as Acquisition Cue Only, Not Continuous Fusion

**Status:** Accepted
**Related rejected alternative:** [ADR-0010](0010-rejected-continuous-kalman-fusion-both-sensors.md)

## Decision

The Search Radar drives `SEARCH` and `ACQUIRE` only. Once the FCR achieves
lock, the FSM transitions to `TRACK` and the Search Radar cue is no longer used
for filtering. There is no continuous blending of Search Radar and FCR data.

## Context

Two sensor fusion approaches were considered:

1. **Hard handoff (chosen)** — Search Radar cues the turret to the target's approximate bearing. Once the FCR locks on, it takes over exclusively. Clean architectural separation between acquisition and tracking.
2. **Continuous fusion** — Search Radar and FCR measurements blended throughout all states, weighted by their noise covariances (e.g. via complementary filter or Kalman filter).

## Reasoning

The hard handoff is simpler to implement and matches the real system architecture: search radars cue fire-control radars, which then track independently. The fusion story is architectural ("two inputs, defined handoff") rather than mathematical, and is easier to explain under assessor questioning.

## Current revision

The revised two-process design strengthens the handoff. Search Radar runs in a
separate controller process and emits only a selected world-frame cue over the
radio link. ATLAS uses that cue to slew the turret. The FCR is the turret-aim
sensor and feeds the Track Filter after lock. Continuous Search Radar
measurement fusion is rejected in ADR-0010.
