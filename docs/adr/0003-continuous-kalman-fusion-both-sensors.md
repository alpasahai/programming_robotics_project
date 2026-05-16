# ADR-0003: Continuous Kalman Fusion of Both Sensors

**Status:** Accepted  
**Supersedes:** ADR-0003-search-radar-handoff-not-continuous-fusion.md

## Decision

Both FireControlRadar (FCR) and SearchRadar feed the TrackFilter Kalman filter
every timestep throughout all FSM states. There is no hard handoff. The filter's
R matrices naturally weight FCR more heavily (low R_fcr) and SearchRadar less
heavily (high R_search).

## Context

The original design used a hard handoff: SearchRadar drove SEARCH/ACQUIRE only,
then FCR took over exclusively from TRACK onwards. Two reasons forced the change:

1. **Sensor fusion requirement** — the assignment's Embedded Intelligence
   specialisation requires sensor fusion. A hard handoff is sensor switching,
   not fusion. Continuous dual-sensor Kalman updates satisfy the requirement
   with genuine mathematics that can be explained under examination.

2. **Multiple projectiles** — with multiple simultaneous projectiles, the FSM
   locks a target identity at ACQUIRE (`set_target()` called on both FCR and
   SearchRadar). After lock-on, both sensors measure the same target — there
   is no reason to gate either sensor by state.

## Design

Every timestep in the main loop:
```python
track_filter.predict()
if fcr.get_target_position():
    track_filter.update_fcr(fcr.get_target_position())
if search_radar.get_target_position():
    track_filter.update_search(search_radar.get_target_position())
```

The FSM never calls update_fcr() or update_search() directly — this happens
in atlas_controller.py before fsm.step(). The FSM only controls set_target()
and track_filter.reset() at the ACQUIRE transition.

## Rationale

Modelled on real fire-control systems (Phalanx Block 1B, Patriot) which fuse
a coarse wide-beam radar with a precise narrow-beam sensor continuously.
The covariance-weighting formula `x_fused = (σ₂²·x₁ + σ₁²·x₂) / (σ₁² + σ₂²)`
is analytically optimal for two independent unbiased estimators — Kalman
implements exactly this.

## Planned upgrade

Add a third measurement source (e.g. Camera with recognitionEnable() on the
tilt motor) as an additional `update_camera()` path with its own R matrix.
The rest of the system is unaffected.
