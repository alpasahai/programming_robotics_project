# Adaptive launch-point estimation (design)

**Date:** 2026-05-22
**Status:** Proposed — spec for the AI / Adaptive Behaviour advanced component (#44 / #34)
**Relationship:** This is the **lower-risk alternative** to
`2026-05-22-attack-plan-and-adaptive-ml-design.md` (the launch-*direction*
prediction feature). Both target the same 10% advanced element; this one is a
smaller, sturdier feature for the deadline. The two live on separate branches /
draft PRs as alternatives.

## Why this exists

The assignment's **Advanced / Contextual Component (10%)** requires *at least one
advanced element*; we take **"AI or Adaptive Behaviour."** The spec says: *"Build
a system that demonstrates intelligence, not just motion."* The Week-4 *Embedded
AI* lecture lists, as concrete AI enhancements: **Perception Improvement**,
**Sensor Averaging (Noise Reduction)** ("a moving-average filter smooths jittery
sensor data"), and **prediction/decision enhancement** (act ahead of an
anticipated event). This feature is squarely those.

**The idea:** the attacker launches from a roughly **static launch point** (with
small per-launch jitter and possibly slow drift). ATLAS is **not told** where that
is, and its only cue — the Search Radar — is **noisy** (≈0.2 m std). So ATLAS
**learns the launch origin online by aggregating the noisy cues across many
launches** (recursive estimation / noise reduction), and uses the sharpened
estimate to **pre-aim the idle turret** and **seed the tracker**, cutting
acquisition latency. The estimate **improves with experience** and **adapts** if
the launch point drifts.

This is a visible **Sense → Think → Act** loop:
- **Sense** — at each launch, ATLAS records the Search Radar bearing/position of
  the new projectile (its first acquisition), using existing cues only.
- **Think** — a recursive estimator fuses that noisy observation into a running
  estimate of the launch point, reducing variance as `~σ/√N`.
- **Act** — ATLAS pre-aims `IDLE` at the estimated launch point (and seeds the
  Kalman tracker there on acquisition), so the FCR locks sooner and the
  prediction converges faster.

### Honest scope of the benefit

The FCR is already precise (0.02 m) **once locked**, and the Kalman filter
de-noises during tracking. So this feature improves **acquisition latency and the
pre-lock estimate**, *not* the terminal aim. The demonstrable claim is:
**"ATLAS learns the launch origin from noisy cues and therefore acquires/fires
sooner, improving with experience and adapting to drift."** That is measurable
(time-to-lock vs launch number) and visible (the idle turret pre-orients).

### Key decisions

- **Learn online from existing cues — no dataset, no new sensor, no new radio
  channel.** ATLAS aggregates the Search Radar cues it already receives.
- **Classical recursive estimation.** An exponentially-weighted running mean per
  axis (equivalently a 1-state-per-axis Kalman update). One meaningful knob: the
  forgetting/adaptation rate `alpha` (the dial between noise-robustness and
  drift-tracking). No arbitrary buffer length or model-capacity constants.
- **The attacker's launch point is its own ground truth and is never sent to
  ATLAS.** ATLAS perceives it through noisy radar only — so a hardcoded constant
  would be both unavailable and (under drift) wrong; the running estimate is what
  makes it work.
- **Reuses the existing acquisition seam.** A launch is observed as the first
  Search Radar acquisition after the previous projectile resolved (the
  ground-hit/bullet-hit cues already segment engagements). No new devices.
- **Plugs into the pure FSM the same way a predictor would** — `IDLE` consumes a
  ready-aim `(pan, tilt)` value; the five-state structure (ADR-0012) is unchanged.

## Part 1 — Attacker side: a noisy, drifting launch point

- The attacker launches from a single nominal launch point, but each launch's
  `spawn_position` is **jittered** by a small zero-mean amount (e.g. ±0.3 m per
  axis), so consecutive observations differ and there is something for ATLAS to
  average out.
- **Optional slow drift (SHOULD):** the nominal point migrates slowly over the
  run (e.g. a few cm per launch along a direction), so the *forgetting* in the
  estimator earns its keep and ATLAS visibly re-converges.
- Implemented via the `Projectile` `select_launch` hook (a callable invoked at the
  top of `spawn()` returning `(spawn_position, launch_velocity)`); the controller
  supplies a jittered/drifting point each launch. The launch velocity stays fixed
  (single corridor toward the turret). Seeded RNG → reproducible demos.
- The nominal point and jitter/drift are the attacker's ground truth and are
  **never communicated to ATLAS.**

## Part 2 — ATLAS side: the launch-point estimator

### Observing a launch (radar-only, segmented by resolution cues)

Identical seam to the existing engagement cycle:
- **Launch event = first Search Radar acquisition of a new projectile.** The
  previous projectile's resolution (ground-hit ch 2 / bullet-hit ch 3, which drive
  the FSM `RESET`) marks the boundary; the next acquisition is the next launch.
  An arm/disarm latch yields exactly one observation per engagement and debounces
  mid-flight detection dropouts.
- **Observation = the cued world-frame position at first acquisition** (the Search
  Radar emits `struct.pack("ddd", x, y, z)`; the existing `SearchRadarLink`
  exposes it). Taken at first acquisition, the projectile is still near its launch
  point, so the cue is a noisy sample of the launch origin.

### `LaunchPointEstimator`

A small class (numpy or pure Python — **no scikit-learn**), sibling to the
existing link classes:

- Holds a per-axis estimate `mu = [x, y, z]` and a sample count `n`.
- `observe(position)`: on the first observation, `mu = position`. Thereafter,
  exponentially-weighted update `mu += alpha * (position - mu)` (a recursive mean
  with forgetting — variance falls roughly as the cues accumulate, and the
  forgetting lets it track a drifting point). `alpha` is the single tuning knob
  (the adaptation rate).
- `get_estimate()`: returns the current `mu`, or `None` before any observation.
- `get_ready_aim(turret_position)`: returns the `(pan, tilt)` aiming at the
  estimated launch point relative to the turret (`pan = atan2(dx, dy)`,
  `tilt = atan2(dz, sqrt(dx²+dy²))` — the FSM's convention), or `None` before any
  observation (cold start). This is the value the FSM `IDLE` state consumes.
- `samples`: count of observations (for telemetry / the convergence plot).

### Act — closing the loop

- **MUST:** `IDLE` pre-aims at the estimated launch point via `get_ready_aim()`
  (falls back to the fixed idle beam before the first observation — a cold-start
  safety property). So the idle turret pre-orients toward where launches come
  from, and the FCR acquires sooner once the next ball launches.
- **SHOULD:** seed the Kalman `TrackFilter` at the estimated launch point on
  acquisition (a low-noise prior) so the ballistic prediction converges in fewer
  steps → earlier fire. Telemetry of estimate vs launch number (the convergence
  curve) and time-to-lock with/without the estimate.

### FSM impact — minimal and pure

Add an optional provider to `SensorSuite` (default `None` → today's fixed beam)
exposing `get_ready_aim()`; `IDLE` uses it when present. The FSM holds no
estimator and no I/O — it consumes a `(pan, tilt)` value, exactly as it consumes
cues. Only `IDLE` gains an input; the five-state structure (ADR-0012) is
unchanged. Hook point: `_do_idle()` (`fsm.py`).

## Data flow

```
attacker: jittered/drifting launch point → launch ball   (point never sent to ATLAS)
                              │
                              ▼
atlas:    Search Radar first-acquisition of new projectile   [existing SearchRadarLink]
          segmented by ground-hit / bullet-hit resolution cues [existing ch 2 / ch 3]
          → launch observation (world-frame position)
          → LaunchPointEstimator.observe()  (recursive mean w/ forgetting; variance ↓ with N)
          → get_ready_aim(turret) → (pan, tilt)
          → FSM IDLE pre-aims at the estimated launch point → faster acquisition; adapts to drift
          → (SHOULD) seed TrackFilter at the estimate on acquisition → earlier fire

no dataset, no new device, no scikit-learn: a classical online estimator over existing cues.
```

## Testing

- `LaunchPointEstimator`:
  - first `observe()` sets the estimate to that position; `get_estimate()` returns it.
  - repeated observations of a noisy point **converge toward the true mean** (feed
    samples ~N(true, σ); after many, `||mu − true||` is small — well below σ).
  - **adaptation:** after the true point shifts, the estimate **moves toward the
    new point** within a bounded number of observations (forgetting works).
  - `get_ready_aim(turret)` returns `(atan2(dx,dy), atan2(dz, hyp))` for the
    estimate relative to the turret; `None` before any observation.
  - `samples` increments per observation.
- attacker jitter: `select_launch` returns points within the configured jitter of
  the nominal (seeded → reproducible); optional drift moves the nominal as configured.
- launch-observation segmentation: a first acquisition after a resolution cue
  yields exactly one observation; a reacquired track within the same engagement
  does not (debounce holds).
- FSM: `IDLE` adopts the ready-aim when present; falls back to the fixed beam when
  the estimator returns `None` (cold-start safety). Existing FSM tests stay green
  (the new `SensorSuite` field defaults to `None`).

## Report linkage

Feeds **§ Technical Requirements & System Integration** (the advanced
AI/adaptive element: online estimation / perception improvement) and **§ System
Architecture** (a Layer-3 estimator feeding a suggestion to the Layer-2 FSM). The
estimate-vs-launch-number convergence curve and the time-to-lock improvement are
the **adaptive-behaviour** evidence. The cold-start fallback is a **Safety,
Robustness & Reliability** point.

## Scope summary

**MUST** — attacker jitters a single launch point (point never sent to ATLAS);
ATLAS observes launches radar-only (first-acquisition, segmented by existing
resolution cues); `LaunchPointEstimator` recursively estimates the launch origin
(forgetting mean, one `alpha` knob); `IDLE` pre-aims at the estimate;
cold-start fallback to the fixed beam.
**SHOULD** — slow drift of the launch point + visible re-convergence; seed the
TrackFilter at the estimate on acquisition; convergence / time-to-lock telemetry.
**COULD** — per-axis variance tracking (a true 1-D Kalman) to expose confidence;
gate the pre-aim on estimate confidence.

## Out of scope

- Predicting *which* sector / direction a launch comes from (that is the
  alternative direction-learning feature on the other PR).
- Multi-target tracking (one projectile alive at a time stays in force).
- Any pre-trained or serialized model, dataset, or new sensor/radio device.
