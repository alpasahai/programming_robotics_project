# ADR-0003: Search Radar as Acquisition Cue Only, Not Continuous Fusion

**Status:** Superseded by [ADR-0003-continuous-kalman-fusion-both-sensors.md](0003-continuous-kalman-fusion-both-sensors.md)

## Decision

The Search Radar drives SEARCH and ACQUIRE states only. Once the FCR achieves lock, the FSM transitions to TRACK and the Search Radar cue is no longer used. There is no continuous blending of Search Radar and FCR data.

## Context

Two sensor fusion approaches were considered:

1. **Hard handoff (chosen)** — Search Radar cues the turret to the target's approximate bearing. Once the FCR locks on, it takes over exclusively. Clean architectural separation between acquisition and tracking.
2. **Continuous fusion** — Search Radar and FCR measurements blended throughout all states, weighted by their noise covariances (e.g. via complementary filter or Kalman filter).

## Reasoning

The hard handoff is simpler to implement and matches the real system architecture: search radars cue fire-control radars, which then track independently. The fusion story is architectural ("two inputs, defined handoff") rather than mathematical, and is easier to explain under assessor questioning.

## Planned upgrade

Continuous Kalman filter fusion will be added in a future iteration. The Search Radar measurement will enter the Kalman update step with a high noise covariance R, and the FCR with a low R — the filter will naturally weight the FCR more heavily once locked. See ADR-0002 for the Kalman upgrade path on the prediction side. The FSM and Track Report interface will not change.
