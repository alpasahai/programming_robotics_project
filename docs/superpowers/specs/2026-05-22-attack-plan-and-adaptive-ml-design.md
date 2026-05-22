# Attack plan + adaptive ML pattern detection (design)

**Date:** 2026-05-22
**Status:** Proposed — spec for #44 (sub-issue of #34)
**Supersedes the ML portion of:** `2026-05-18-attack-pattern-and-ml-detection-design.md`
(that doc left ML consumption "out of scope"; this one closes the loop so the
feature counts as *adaptive behaviour*, not offline analysis).

## Why this exists

The assignment's **Advanced / Contextual Component (10%)** requires *at least one
advanced element*; we are taking **"AI or Adaptive Behaviour."** The grading note
is explicit: *"Build a system that demonstrates intelligence, not just motion."*
A classifier that only prints a label offline is weak evidence. The win condition
is a visible **Sense → Think → Act** loop:

- **Sense** — ATLAS observes the attacker's launch events (timestamps only).
- **Think** — a small trained model infers *which launch cadence is active* and
  *when the next launch is due*.
- **Act** — ATLAS pre-positions the turret toward the expected launch corridor
  ahead of the predicted launch, cutting acquisition latency — and visibly
  *re-adapts* when the attacker changes cadence mid-run.

This spec covers two co-designed halves, because the attack plan only matters in
that it is **ML-predictable by construction**:

1. **The attack plan** (#34 / #44) — what the attacker launches and when.
2. **The adaptive ML feature** — what ATLAS infers from it and how that changes
   behaviour.

We are time-constrained, so each part is split into **MUST** (the minimum that
satisfies the advanced rubric and is demonstrable in the 3-minute video),
**SHOULD** (the adaptation money-shot), and **COULD** (only if time remains).

## Part 1 — The attack plan (attacker side)

### Pattern classes

A small, fixed, **well-separated** set of cadence classes, each defined purely as
**data** (a base period, not hardcoded logic):

| Class    | Base period `T` | Notes                                  |
| -------- | --------------- | -------------------------------------- |
| `SLOW`   | e.g. 8 s        | comfortably one ball alive at a time   |
| `MEDIUM` | e.g. 5 s        |                                        |
| `FAST`   | e.g. 3 s        | must still exceed the ball resolve time |

Three classes is the **MUST**. A 4th `BURST` class is **COULD**. The base periods
are tuning constants chosen so (a) only one projectile is alive at a time
(launch period > worst-case resolve time of destroy-or-land), and (b) the classes
are far enough apart that mean-interval alone separates them cleanly.

### Schedule generation (the "attack plan method" #44 names)

- Each launch time is `T + jitter`, where `jitter ~ Uniform(-j, +j)` with `j`
  small relative to `T` (e.g. `j = 0.15·T`). Jitter makes detection non-trivial
  (so the model has a real job) without blurring the classes together.
- The schedule is **preloaded at startup** from a chosen class (or class
  sequence — see SHOULD), via a pure `build_schedule(class, rng) -> list[float]`
  function. Pure and seeded → unit-testable and reproducible. The controller loop
  only *reads* the next due time; it does not generate reactively.
- Launch **origin and velocity** stay fixed in the MUST (single corridor toward
  the turret), so the predictor's "where to pre-aim" is a constant and the only
  thing the model infers is *timing*. Multiple origins are COULD.

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
- **The pulse carries no class label.** The label is what the model must infer.
  Leaking it would make the "AI" a lookup, not a classifier.

### Training-data log (attacker side, offline)

The attacker *knows* the class it is running, so it logs every launch as
`(sim_time, class_label, interval_since_prev)` to a CSV. This file is the labelled
dataset — no manual labelling. Crucially, see the synthetic-data note in Part 2:
we do **not** depend on long sim runs to get enough training data.

## Part 2 — The adaptive ML feature (ATLAS side)

### Task framing

**Classification, not regression.** Given a sliding window of recent
inter-launch intervals, predict the active class. (Predicting the exact next
timestamp is regression and far harder for the same payoff.) From the predicted
class we recover `T`, and `next_launch ≈ last_launch + T` gives the timing the
Act step needs — so one classification yields both "which pattern" and "when
next."

### Model

- A **micro scikit-learn** model — **logistic regression** (MUST; linear,
  trivially explainable in the viva) or a depth-limited `DecisionTreeClassifier`
  (COULD, if a non-linear boundary helps). No deep learning: tiny feature vector,
  well-separated classes, trains in <1 s, predicts in microseconds.
- **`scikit-learn` must be added to `pyproject.toml`** (`joblib` ships with it for
  serialization). The trained model is serialized to a committed artifact
  (`controllers/atlas_controller/attack_pattern_model.joblib`) so the sim loads a
  fixed model — no training at sim time.

### Features

From a sliding window of the last `N` intervals (`N ≈ 5`):

- **MUST:** `mean(interval)`, `std(interval)`. These alone separate three
  base-period classes under modest jitter.
- **COULD:** dominant FFT frequency of the launch train (only needed if a
  `BURST`/multi-rate class is added).

A pure `extract_features(intervals) -> np.ndarray` function, shared by training
and inference so there is **no train/serve skew**.

### Training data — synthetic first (de-risks the deadline)

Because the schedule generator is a pure function, training data is generated
**in code**: sample thousands of labelled interval windows directly from
`build_schedule` across classes and jitter draws. No long Webots runs required —
this is the key time-saver. The attacker's real CSV log (Part 1) is an optional
augmentation / sanity check, **not** a blocker. A `train_attack_pattern_model.py`
script: generate → `extract_features` → fit → report held-out accuracy → dump
`.joblib`. Expect near-perfect accuracy given how separated the classes are; that
is fine and expected for this rubric.

### Online inference — `AttackPredictor` (ATLAS side)

A new class, sibling to `SearchRadarLink` / `AttackerGroundHitLink`, fed by a
thin `LaunchCueLink` over the channel-4 receiver (same shape as the existing link
classes: controller enables the receiver, link drains the queue, exposes
`launched_this_step()` / `count`):

- Maintains a deque of the last `N+1` launch timestamps → intervals.
- Once the window is full, every step: `extract_features` → `model.predict` →
  exposes `predicted_class`, `predicted_period`, and
  `next_launch_eta = last_launch + predicted_period - now`.
- Before the window fills, reports `UNKNOWN` and the system behaves exactly as it
  does today (graceful cold-start — a safety/robustness point for the report).
- The loaded model is injected (constructor arg), keeping `AttackPredictor`
  unit-testable with a stub/fitted-in-test model.

This is distinct from the existing `BallisticTrajectoryPredictor` (per-projectile
trajectory) — `AttackPredictor` reasons about the *threat stream*, matching the
`AttackPredictor` glossary entry already in `CONTEXT.md`.

### Act — closing the loop (this is what makes it "adaptive")

- **MUST:** the predicted class is surfaced live (telemetry/log + the value the
  FSM `IDLE` state uses), and ATLAS sets its `IDLE` ready-aim toward the (fixed,
  known) launch corridor. Even static, this is a perception-driven decision and
  satisfies "AI."
- **SHOULD (the demo highlight):** ATLAS uses `next_launch_eta` to **pre-slew**
  from `IDLE` toward the launch corridor a fixed lead time before the predicted
  launch, so the turret is already pointed when the Search Radar cue arrives —
  measurably shorter time-to-lock. When the attacker switches class mid-run, the
  classifier re-detects within a window and the pre-slew timing changes on screen.
  That visible re-adaptation is the strongest evidence of "intelligence, not just
  motion."

### FSM impact — kept minimal and pure

The FSM stays pure (no model, no I/O). `AttackPredictor` produces a **ready-aim
suggestion** + ETA that the controller passes into the FSM `SensorSuite`; `IDLE`
consumes it as its hold direction / pre-slew trigger. This adds an input to
`IDLE` only — the five-state structure (ADR-0012) is unchanged. If pre-slew
timing proves fiddly under the deadline, dropping to the MUST (static ready-aim)
is a one-line behaviour change, not a redesign.

## Data flow

```
attacker: build_schedule(class) → launch ball → ATTACKER_LAUNCH_EMITTER.send(pack("i",n))   [ch 4, pulse]
          └─ also append (t, class, interval) → training CSV (offline)
                              │  (one-step radio lag, per ADR-0009)
                              ▼
atlas:    ATTACKER_LAUNCH_RECEIVER → LaunchCueLink → AttackPredictor
          → extract_features(window) → model.predict → predicted_class / next_launch_eta
          → FSM IDLE ready-aim + pre-slew  →  shorter time-to-lock, re-adapts on class switch

offline (no sim): build_schedule sampling → extract_features → LogisticRegression.fit → attack_pattern_model.joblib (committed)
```

## Testing

- `build_schedule`: deterministic under a seeded RNG; periods respect the
  `period > resolve_time` invariant; jitter stays within `±j`.
- `extract_features`: shape and values on hand-built interval windows; the same
  function is exercised by both training and `AttackPredictor` (no skew).
- `AttackPredictor`: injected fitted/stub model → correct class on canned
  windows; reports `UNKNOWN` before the window fills; `next_launch_eta` arithmetic.
- `LaunchCueLink`: mirror `test_attacker_ground_hit_link.py` (empty queue,
  one packet, multiple drained, flag clears).
- FSM: `IDLE` adopts the ready-aim suggestion when present; falls back to the
  fixed beam when the predictor reports `UNKNOWN` (cold-start safety).
- Training script: held-out accuracy above a threshold (sanity gate, not a
  research metric).

## Report linkage

Feeds **§2 Technical Requirements Compliance** (the advanced element) and
**§5 Perception & Sensor Processing** (launch-event observation → inference).
The cold-start `UNKNOWN` fallback is a **§6 Safety & Robustness** point.

## Scope summary

**MUST** — 3 cadence classes; channel-4 launch cue; logistic-regression model
trained on synthetic data, committed as `.joblib`; live classification consumed
as `IDLE` ready-aim.
**SHOULD** — ETA-driven pre-slew + mid-run class switch demonstrating re-adaptation.
**COULD** — `BURST` class, FFT feature, decision-tree model, multiple launch
origins.

## Out of scope

- Predicting exact next-launch *timestamp* as a regression target.
- Multi-target tracking (one projectile alive at a time stays in force).
- Drag / damping physics ("hard mode").
- Online (sim-time) model training — the model is always pre-trained and loaded.
