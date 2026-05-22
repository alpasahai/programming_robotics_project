# Attack plan + adaptive launch-direction learning (design)

**Date:** 2026-05-22
**Status:** Proposed — spec for #44 (sub-issue of #34)
**Supersedes the ML portion of:** `2026-05-18-attack-pattern-and-ml-detection-design.md`
(that doc left ML consumption "out of scope" and predicted *cadence* only; this one
closes the loop and predicts *direction*, so the feature counts as visible
*adaptive behaviour*, not offline analysis).

## Why this exists

The assignment's **Advanced / Contextual Component (10%)** requires *at least one
advanced element*; we are taking **"AI or Adaptive Behaviour."** The spec is
explicit: *"Build a system that demonstrates intelligence, not just motion."*

The Week-4 *Embedded AI* lecture frames the win condition almost exactly:
- **"Adaptive"** = the FSM's transition inputs are *dynamic variables calculated
  in real time*, not hard-coded numbers.
- A **learning system** *improves from experience/data* rather than following
  fixed rules.
- **"Layer 3: Strategic AI"** watches patterns and sends *suggestions* down to the
  reactive FSM (Layer 2).
- Its headline example of intelligence is *predict-and-act-ahead*: notice a
  recurring pattern, pre-calculate, and be **already in position when the event
  arrives** (the school-crowd crossing).

So the feature is a visible **Sense → Think → Act** loop built around *where* the
attacker launches from:

- **Sense** — ATLAS observes each launch: the channel-4 launch pulse (timing) ×
  the Search Radar bearing of the projectile (direction). This is sensor fusion.
- **Think** — a small classifier, **trained live during the run** on ATLAS's own
  observations, learns the attacker's *spatial launch pattern* and predicts the
  **next launch sector**.
- **Act** — ATLAS pre-aims the idle turret at the predicted sector, so it is
  already pointed when the next launch comes (shorter time-to-lock) — and
  **visibly re-learns** when the attacker changes its pattern mid-run.

This spec covers two co-designed halves, because the attack plan only matters in
that it is **learnable by construction**:

1. **The attack plan** (#34 / #44) — what the attacker launches and from where.
2. **The adaptive learning feature** — what ATLAS infers from it and how that
   changes behaviour.

Each part is split into **MUST** (the minimum that satisfies the advanced rubric
and is demonstrable in the video), **SHOULD** (refinements), and **COULD** (only
if time remains).

### Key decisions (resolved in brainstorming)

- **Predict direction, not frequency.** Direction prediction is *visible* (the
  turret physically swings to the right bearing before the threat appears),
  whereas cadence prediction is invisible. It is also a more honest ML problem —
  binning mean-interval into SLOW/MEDIUM/FAST is so separable it reads as a
  threshold, not a classifier.
- **No training dataset — learn online.** There is **no synthetic data and no
  pre-collected dataset**. ATLAS learns purely from the launches it observes
  during the run. This sidesteps the synthetic-data question entirely and gives
  the strongest "learns from experience" / re-adaptation demo. There is no
  committed model artifact.
- **Sector label is perceived *and discovered*, not handed over.** The launch
  pulse carries timing only (no sector label). ATLAS is **not told how many
  launch directions exist or where they are** — it **discovers the sectors itself**
  by online-clustering the radar bearings it observes, then labels each launch
  with a discovered cluster. The prediction is therefore fully perception-driven,
  not a lookup against preset boundaries.
- **Run-length patterns are the target.** e.g. *5 launches from sector A, then 2
  from B, then 4 from C, repeat.* This needs a **run-length feature**, which is
  why the learner is a **depth-limited DecisionTree refit on a sliding window**
  (handles `run >= 5 then switch` thresholds natively and is the most explainable
  in the viva), rather than an order-1 transition model.

## Part 1 — The attack plan (attacker side)

### Launch sectors (attacker ground truth — never told to ATLAS)

The attacker launches from a set of **physical launch sectors** — distinct
bearings around the turret's engagement arc. Each is defined purely as data: a
`(spawn_position, launch_velocity)` pair whose trajectory flies toward the turret
from that bearing. (The attacker's launch is already fully data-driven:
`ProjectileConfig.spawn_position` / `launch_velocity` are plain lists set before
each `projectile.spawn()`, so multiple corridors are a near-trivial change.)

**The number and bearings of these sectors are configurable on the attacker and
are deliberately *not* communicated to ATLAS.** ATLAS rediscovers them from
observation (see "Discovering the sectors" below). The only design constraint is
that distinct sectors are far enough apart in bearing to be separable under the
Search Radar's bearing noise. The pattern examples below use three sectors for
illustration, but nothing in ATLAS hard-codes that count.

### Pattern generation — run-length cycles (the "attack plan method" #44 names)

The attack plan is a **run-length sequence**: a list of `(sector, run_length)`
segments, expanded and repeated. Example:

```
[(A, 5), (B, 2), (C, 4)]  →  A A A A A B B C C C C  A A A A A B B C C C C  …
```

- A pure `build_pattern(segments, rng) -> Iterator[sector]` (and the matching
  launch *times*) drives the attacker. Pure + seeded → reproducible and
  unit-testable.
- **This is the attacker's behaviour, not a training set.** ATLAS never sees
  `build_pattern`; it only observes the resulting launches. This is what removes
  the need for any synthetic or collected dataset.
- **Optional count jitter (COULD):** perturb run lengths by ±1 occasionally so the
  pattern is statistical rather than a perfect counter. By default counts are
  deterministic; the radar-bin step already introduces realistic wobble.
- Launch **timing** keeps a base period + bounded jitter so **one projectile is
  alive at a time** (period > worst-case resolve time of destroy-or-land). Timing
  is now *secondary* — it only sets the optional pre-slew lead time, not the
  headline.

### Mid-run pattern switch (the demo highlight)

Partway through the run the attacker swaps its segment list (e.g.
`[(A,5),(B,2),(C,4)]` → `[(C,3),(A,4),(B,3)]`). Because ATLAS learns online, its
predictions converge to the new pattern within a sliding window — **visible
re-adaptation on screen**, the strongest evidence of "intelligence, not motion."

### Launch cue (new authoritative event)

The attacker already owns ground-hit (ch 2) and bullet-hit (ch 3) truth and emits
them as one-shot pulses (see `2026-05-21-attacker-ground-hit-cue-design.md`). We
add a third, mirroring that exact pattern:

- **New `ATTACKER_LAUNCH_EMITTER`, `channel 4`** on `Attacker.proto`; new
  `ATTACKER_LAUNCH_RECEIVER`, `channel 4` on `AtlasTurret.proto`. (Channels 1–3
  are taken: cue / ground-hit / bullet-hit.)
- Payload `struct.pack("i", launch_count)` — one pulse on the step a ball
  launches, nothing otherwise. ATLAS reads the **receive time** as the launch
  timestamp; the count lets it detect dropped packets.
- **The pulse carries no sector label.** The label is what ATLAS perceives from
  radar. Leaking it would make the "AI" a lookup, not a perception-driven
  predictor.

## Part 2 — The adaptive learning feature (ATLAS side)

### Task framing

**Multiclass classification: predict the next launch sector.** Given a short
history of observed launches, predict which sector the *next* launch will come
from, then pre-aim the turret at that sector's bearing.

### Perceiving the sector (sensor fusion)

On each channel-4 launch pulse, ATLAS snapshots the projectile's **Search Radar
bearing**:

- Radar reports a world-frame position; bearing is `atan2(dx, dy)` (the same
  convention the FSM already uses for aim).
- This pairing — **launch-cue event (timing) × radar bearing (direction)** — is
  the raw observation and a concrete **sensor-fusion** point for the report.

### Discovering the sectors (unsupervised, online — no preset count or boundaries)

ATLAS is not given the number of launch directions or their bearings. It
**discovers them** by online-clustering the stream of observed bearings:

- **Online 1-D angular leader-clustering** (a `SectorMap`): maintain a list of
  discovered cluster centres (bearings). For each new launch bearing, if it lies
  within an angular tolerance `tol` of an existing centre, it joins that cluster
  (and nudges the centre toward it); otherwise it **opens a new cluster** with a
  fresh stable id. The number of sectors *emerges* from the data — no `k`, no
  preset boundaries.
- 1-D angular leader-clustering (rather than `KMeans`/`MiniBatchKMeans`, which
  need `k`) is chosen precisely because it discovers the cluster count itself, is
  trivial and explainable, handles angle wrap-around, and assigns **stable ids**
  so the downstream pattern learner sees consistent labels over time.
- `tol` is the one tuning knob; it just needs to be smaller than the gap between
  real sectors and larger than the radar bearing noise — both easily satisfied by
  spacing the attacker's sectors apart.

Each launch is thus labelled with a **discovered sector id**, with no knowledge of
the attacker's true sector set.

### Features

A pure `extract_features(history) -> np.ndarray`, shared by learning and
inference (no skew):

- **MUST:** one-hot(current **discovered** sector id) ⊕ **run_length** (consecutive
  launches from the current sector) ⊕ one-hot(previous discovered sector id). The
  `run_length` feature is what makes run-length patterns learnable — without it,
  an order-1 model cannot tell the 5th A (switch to B) from the 1st A (stay on A).
  The one-hot width tracks the number of clusters discovered so far, so the
  feature space grows as new sectors appear (the tree is refit over the current
  label set each launch — see below).
- **COULD:** longer context (last *k* sectors), launch-interval stats.

### Model — sliding-window DecisionTree (online by refit)

- A depth-limited **`sklearn.tree.DecisionTreeClassifier`** (`max_depth ≈ 4`),
  **refit every launch on a sliding window** of the last `W` observed
  `(features, next_sector)` pairs (`W ≈ 30–40`, ≥ ~3 pattern periods). Trees train
  in microseconds on this little data.
- Why a tree: run-length thresholds (`if run_length >= 5 and sector == A → B`) are
  learned **natively**, and the fitted tree is **directly explainable in the
  viva** (show the tree). Why sliding-window refit: stale observations fall out of
  the window, so a **mid-run pattern switch is forgotten and re-learned cleanly**.
  (sklearn trees have no `partial_fit`; refit-on-window is the online mechanism,
  and is cheap at this scale.)
- **`scikit-learn` must be added to `pyproject.toml`.** No `joblib` / serialized
  artifact is needed — the model is built live, never loaded from disk.

### Online inference — `AttackPredictor` (ATLAS side)

A new class, sibling to `SearchRadarLink` / `AttackerGroundHitLink`, fed by a thin
`LaunchCueLink` over the channel-4 receiver (same shape as the existing link
classes: controller enables the receiver, link drains the queue, exposes
`launched_this_step()` / `count`):

- Owns the `SectorMap` (sector discovery) and a deque of recent observed launches
  `(discovered_sector_id, time)`.
- On each launch: feed the radar bearing to `SectorMap` → discovered sector id →
  append it → form the `(features, label)` pair
  for the *just-arrived* launch (label = its sector, features computed from the
  history *before* it) → push into the window → **refit** the tree → predict the
  **next** sector from the current history.
- Exposes `predicted_sector` (a discovered id), `predicted_bearing` (that
  cluster's centre from the `SectorMap`), and `next_launch_eta`
  (`last_launch + mean_interval - now`).
- **Cold-start:** before the window holds `min_samples` (≈ 12, > one period),
  reports `UNKNOWN`; the system behaves exactly as it does today (graceful
  fallback — a **safety/robustness** point for the report). Cold-start is slightly
  longer than a sweep pattern because run-length cells must each be seen.
- This is distinct from the per-projectile `BallisticTrajectoryPredictor` —
  `AttackPredictor` reasons about the *threat stream*, matching (and sharpening)
  the `AttackPredictor` glossary entry in `CONTEXT.md` (update its wording from
  "attack frequency" to "next launch sector/origin").

### Act — closing the loop (this is what makes it "adaptive")

- **MUST:** `AttackPredictor` produces a **ready-aim bearing** (the predicted
  sector's pan/tilt). The controller passes it into the FSM `SensorSuite`; the
  `IDLE` state holds that bearing instead of the fixed `idle_pan`/`idle_tilt` when
  a prediction is available, and falls back to the fixed beam when the predictor
  reports `UNKNOWN`. So the idle turret is **pre-aimed at the sector the next
  launch is predicted to come from** — a perception-driven decision satisfying
  "AI," with a measurably shorter time-to-lock when the prediction is right.
- **SHOULD:** use `next_launch_eta` to time the pre-slew (lead the predicted
  launch); telemetry/log overlay of *predicted vs actual* sector each launch.
- **The re-adaptation is the money-shot:** during a run the prediction is "same
  sector" (turret holds), then at each switch point it correctly **swings ahead**
  to the next sector — and after a mid-run pattern change it re-learns the new
  switch points within a window.

### FSM impact — kept minimal and pure

The FSM stays pure (no model, no I/O). `AttackPredictor` produces the ready-aim
bearing + ETA that the controller passes into the FSM `SensorSuite`; `IDLE`
consumes it as its hold direction / pre-slew trigger. This adds an input to `IDLE`
only — the **five-state structure (ADR-0012) is unchanged**. If pre-slew timing
proves fiddly under the deadline, dropping to the static-ready-aim MUST is a
one-line behaviour change, not a redesign. Hook point: `_do_idle()`
(`fsm.py:~241`), replacing the fixed `_command_angles(idle_pan, idle_tilt)`.

## Data flow

```
attacker: build_pattern([(A,5),(B,2),(C,4)]) → choose sector's (spawn,velocity) → launch ball
          → ATTACKER_LAUNCH_EMITTER.send(pack("i",n))            [ch 4, pulse, timing only]
                              │  (one-step radio lag, per ADR-0009)
                              ▼
atlas:    ATTACKER_LAUNCH_RECEIVER → LaunchCueLink  ─┐
          Search Radar bearing of the projectile  ──┴→ SectorMap (online clustering)
          → discovered sector id  (count + bearings discovered, not preset)
          → AttackPredictor: append (sector_id,t) → refit DecisionTree on sliding window
          → predict next sector → predicted_bearing (cluster centre) / next_launch_eta
          → FSM IDLE pre-aims at predicted sector  →  shorter time-to-lock; re-adapts on switch

no offline pipeline: no synthetic data, no collected dataset, no committed model artifact.
```

## Testing

- `build_pattern`: deterministic under a seeded RNG; expands run-length segments
  correctly and repeats; launch times respect the `period > resolve_time`
  invariant; optional count jitter stays within ±1.
- `SectorMap` (sector discovery): from a canned bearing stream it discovers the
  correct *number* of clusters with no `k` given; assigns stable ids; bearings
  within `tol` join an existing cluster, a novel bearing opens a new one; handles
  angle wrap-around. (Explicitly: the count is never passed in.)
- `extract_features`: shape and values on hand-built histories; `run_length`
  counts consecutive same-sector launches correctly (resets on switch); same
  function exercised by both learning and inference (no skew).
- `AttackPredictor`: feed a canned run-length launch sequence → after warm-up,
  predicts the switch points (e.g. predicts B on the 5th A); reports `UNKNOWN`
  before `min_samples`; **re-adapts** after a mid-run segment swap within ~`W`
  launches; `next_launch_eta` arithmetic. Inject a stub/fitted model for
  determinism where needed.
- `LaunchCueLink`: mirror `test_attacker_ground_hit_link.py` (empty queue, one
  packet, multiple drained, flag clears, count).
- FSM: `IDLE` adopts the ready-aim bearing when present; falls back to the fixed
  beam when the predictor reports `UNKNOWN` (cold-start safety).

## Report linkage

Feeds **§ Technical Requirements & System Integration** (the advanced AI/adaptive
element) and **§ System Architecture** (the Layer-3 strategic predictor feeding
suggestions to the Layer-2 FSM). The launch-cue × radar pairing is a concrete
**sensor-fusion** example. The cold-start `UNKNOWN` fallback is a **Safety,
Robustness & Reliability** point. The live-learned tree is an explainability
artifact for **Individual Technical Understanding** / the viva.

## Scope summary

**MUST** — attacker launches from multiple sectors with a run-length attack
pattern; channel-4 launch cue (timing-only) + `LaunchCueLink`; **`SectorMap`
unsupervised online discovery of the launch directions from radar bearings (no
preset count or boundaries)**; `AttackPredictor` with a sliding-window
`DecisionTreeClassifier` (run-length feature) predicting the next discovered
sector; `IDLE` pre-aims at the predicted sector; `UNKNOWN` cold-start fallback;
**mid-run pattern switch demonstrating re-adaptation** (cheap with online learning
— this is the headline).
**SHOULD** — `next_launch_eta`-timed pre-slew lead; predicted-vs-actual sector
telemetry overlay; show the discovered sectors + learned tree in the video/report.
**COULD** — per-sector launch velocities; run-length count jitter; longer context
(last-*k*) features; attacker introducing a *new* sector mid-run (showcases live
discovery of a previously-unseen direction).

## Out of scope

- Predicting exact next-launch *timestamp* as a regression target.
- Multi-target tracking (one projectile alive at a time stays in force).
- Drag / damping physics ("hard mode").
- Any pre-trained or serialized model, synthetic dataset, or offline collection
  run — the model is always learned live from observed launches.
