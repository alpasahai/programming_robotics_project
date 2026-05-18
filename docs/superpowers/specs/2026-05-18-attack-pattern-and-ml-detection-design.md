# Programmable multi-projectile attack pattern + ML pattern detection

**Status:** Concept capture — future work. Not scheduled for implementation.
**Date:** 2026-05-18

## Purpose

Capture, for later implementation, two linked ideas:

1. An **adversary system** that launches multiple projectiles at the ATLAS
   turret in a programmable, time-periodic pattern.
2. A **micro ML model** that detects the launch frequency of that pattern from
   what the turret observes.

The pattern is deliberately designed to carry detectable *frequency* structure
so the ML side has a well-posed, easy problem.

## Part 1 — Attack pattern system

### Architecture

A **separate supervisor `Robot` controller** (the "attacker" / "launcher"),
distinct from `atlas_controller`. This pulls projectile launch/reset *out* of
the turret controller, which today carries it as an acknowledged hack (see the
`atlas_controller.py` module docstring). Clean split of responsibility:

- Turret controller = defense (sense, track, aim, fire).
- Attacker controller = offense (launch projectiles per a pattern).

Projectiles do **not** get their own controllers — that would mean N
controllers to coordinate. A single attacker supervisor owns them all.

### Projectile handling

Use a **pool** of pre-placed `SimulatedProjectile` nodes, recycled (reset on
ground hit) rather than dynamically spawned via `importMFNode`. Simpler and
faster. Spawning is an option later if unbounded waves are needed.

### Pattern

Launch events scheduled on a base period / frequency, optionally with jitter so
detection is non-trivial. The pattern is stored as **data** (a period plus
variation, or an explicit list of launch times) — not hardcoded logic — so new
patterns are easy to author.

The pattern is **preloaded** at startup via a dedicated function (the attacker
controller calls it once during setup to obtain the launch schedule) rather than
generated reactively each step.

### Run flow and scoring (refined 2026-05-18)

For the first version, the rules are deliberately simple:

- **One projectile alive at a time.** Launch timing in the pattern is spaced
  widely enough that each ball reliably *resolves* — either destroyed by the
  laser or landed on the ground — before the next ball is launched. This avoids
  any need for multi-target tracking in the first cut.
- **Both outcomes recycle the ball** (reset + relaunch the next one); the run is
  endless. There is no sim-ending condition.
- **Scoring:**
  - Ball reaches the ground → **attacker** scores a point.
  - Ball destroyed by the ATLAS laser → **defender (ATLAS)** scores a point.
- "Destroyed" / "removed" = **recycle**, not true node deletion: `reset` the
  ball (teleport away, zero velocity) and relaunch. This is the same mechanism
  the pooled-projectile design uses.

Score is held in the attacker controller (it owns the projectiles and knows each
launch). Laser-destroy detection is reported to it by the turret side, or the
attacker observes the ball state directly via the supervisor API — interface to
be decided at implementation time.

### Projectile physics — start simple

Begin with **ideal, drag-free projectile physics** so reality matches the
gravity-only model the `BallisticTrajectoryPredictor` assumes:

- No `damping` node (drag is deferred future complexity / a later "hard mode").
- No explicit `inertiaMatrix` / `centerOfMass` — let Webots auto-compute inertia
  from the spherical `boundingObject`.

Damping reintroduces a baked-in prediction bias unrelated to filter quality, so
it is left out until the baseline is validated.

### Turret-side follow-up

`atlas_controller` uses `getFromDef("PROJECTILE")` (singular). Multi-projectile
will need a naming scheme (`PROJECTILE_0/1/2…`) plus list handling. `SearchRadar`
already accepts a *list* of projectile nodes, so that side is ready.

## Part 2 — ML pattern detector

Kept deliberately minimal — "easiest" is the explicit goal.

- **Task** — Classification. Given a window of recently observed launch events,
  predict which attack pattern (frequency class) is active. Chosen over
  next-launch-timing prediction, which is regression and much harder.
- **Classes** — A small fixed set, e.g. 3–4 named patterns (`slow`, `medium`,
  `fast`, possibly `burst`). Each = a base launch frequency.
- **Input / features** — A sliding window of launch timestamps → inter-launch
  intervals → a tiny feature vector: mean interval, std of intervals, and
  optionally the dominant FFT frequency of the launch event train.
- **Model** — A "micro" model: a small scikit-learn classifier (logistic
  regression or a shallow decision tree). Trains in seconds, predicts in
  microseconds. No deep learning — tiny feature vector, well-separated classes.
- **Training data** — The attacker controller logs every launch
  `(timestamp, pattern_label)` to a CSV across runs. That file *is* the dataset;
  no manual labelling, since the attacker knows the pattern it is running.
- **Pipeline** — Offline: collect logs → extract features → train → save model
  file. Online (deferred): a turret-side module loads the model and classifies
  live from observed launches; inference is feature extraction + one `predict()`
  call, cheap enough to run every step.

### Connection between the two halves

The attacker's launch log is directly the ML's training set — that is the
single interface between the two parts.

## Out of scope (future work)

- Actual ML training, feature tuning, hyperparameters.
- Consuming the prediction (e.g. pre-aiming the turret from the classification).
- Dynamic projectile spawning (vs the pooled approach).
- Drag / damping physics ("hard mode").
