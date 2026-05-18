# Attack pattern, turret weapon, referee scoring + ML pattern detection

**Status:** Concept capture — future work. Not scheduled for implementation.
**Date:** 2026-05-18

## Purpose

Capture, for later implementation, a turret-defense "game" and a linked ML idea:

1. An **attacker** that launches projectiles at the ATLAS turret in a
   programmable, time-periodic pattern.
2. A **turret weapon** — the turret fires a physical projectile to intercept and
   destroy the incoming ball.
3. A **referee** that observes the world, applies scoring rules, and owns game
   state.
4. A **micro ML model** that detects the attacker's launch frequency from
   observed launches.

The pattern is deliberately designed to carry detectable *frequency* structure
so the ML side has a well-posed, easy problem.

## Controller architecture

Each role is a separate supervisor `Robot` with its own controller (consistent
with the project already running `atlas_controller` and
`search_radar_controller` as separate controllers):

- **Attacker** — offense. Launches incoming projectiles per the pattern.
- **ATLAS turret** (`atlas_controller`) — defense. Senses, tracks, predicts,
  aims, and fires the turret's projectile.
- **Referee** — scoring and game state. Observes the world, no devices.

Incoming projectiles do **not** get their own controllers — a single attacker
supervisor owns them all.

## Part 1 — Attacker (attack pattern system)

### Projectile handling

First version uses **one** `SimulatedProjectile` node, recycled (reset, not
deleted) after each ball resolves. A *pool* of pre-placed nodes is the later
generalization for overlapping/simultaneous projectiles; dynamic spawning via
`importMFNode` is a further option if waves must be unbounded.

### Pattern

Launch events scheduled on a base period / frequency, optionally with jitter so
detection is non-trivial. The pattern is stored as **data** (a period plus
variation, or an explicit list of launch times) — not hardcoded logic — so new
patterns are easy to author.

The pattern is **preloaded** at startup via a dedicated function (the attacker
controller calls it once during setup to obtain the launch schedule) rather than
generated reactively each step.

### Projectile physics — start simple

Begin with **ideal, drag-free projectile physics** so reality matches the
gravity-only model the `BallisticTrajectoryPredictor` assumes:

- No `damping` node (drag is deferred future complexity / a later "hard mode").
- No explicit `inertiaMatrix` / `centerOfMass` — let Webots auto-compute inertia
  from the spherical `boundingObject`.

Damping reintroduces a baked-in prediction bias unrelated to filter quality, so
it is left out until the baseline is validated.

## Part 2 — Turret weapon: a fired projectile

The turret destroys the incoming ball by **firing a physical projectile** at it
— not by an instant-hit raycast laser.

### Why a fired projectile (not an instant laser)

The system already has a `BallisticTrajectoryPredictor` and an FSM `PREDICT`
state that computes an **intercept point** `lookahead_steps` ahead. That
prediction pipeline only has a reason to exist if the weapon has **travel
time**:

- An instant laser hits immediately → aim at the ball's current position → the
  predictor becomes vestigial dead code.
- A fired projectile travels → the turret must **lead** the target → the
  predictor and intercept logic become the core of the game.

A fired projectile is therefore more consistent with the architecture already
built and tested.

### Mechanics

- The turret's "bullet" is a small `Solid` with `Physics` and a `boundingObject`
  — structurally the same as `SimulatedProjectile`.
- The turret controller (a supervisor) launches it with `setVelocity()` along
  the current aim direction — the same mechanism the attacker uses.
- A **hit** is a genuine Webots physical collision between the bullet's
  `boundingObject` and the ball's. No raycast, no sphere-approximation math —
  the simulation engine computes the real intersection.
- The bullet is **recycled** (reset after each shot), matching the
  one-projectile-at-a-time rule below. A bullet pool is future work.

### Visual laser

The existing `AtlasLaser` cylinder is **kept**, purely cosmetic — an aiming line
/ tracer. It has no `boundingObject` and plays no role in hit detection.

## Part 3 — Referee and scoring

### Referee

The referee is a supervisor `Robot` with no devices — it only observes. Because
a supervisor can read any node in the scene, it detects **both** outcomes itself,
with **no inter-controller messaging** (no `Emitter`/`Receiver`):

- **Ground hit** — watch the incoming ball's world `z`; at/below ground level →
  the ball landed.
- **Bullet–ball collision** — detect the real collision via the supervisor API,
  e.g. `ball.getContactPoints()` (does the ball report contact with the bullet?)
  or a proximity check (`distance(bullet, ball) < bullet_radius + ball_radius`).

### Scoring rules

- Incoming ball reaches the ground → **attacker** scores a point.
- Incoming ball destroyed by the turret's projectile → **defender (ATLAS)**
  scores a point.

The referee is the single owner/writer of the score — one source of truth.

### `SCOREBOARD` node — optional

A dedicated `SCOREBOARD` node with a `customData` (`SFString`) field, into which
the referee *publishes* the running score for other controllers or a HUD to
read. This is **optional** — purely for display. The game is fully playable
without it; the score lives in the referee regardless.

## Part 4 — Run flow

For the first version the rules are deliberately simple:

- **One projectile alive at a time.** Launch timing in the pattern is spaced
  widely enough that each incoming ball reliably *resolves* — destroyed by the
  turret's projectile, or landed — before the next is launched. This avoids
  multi-target tracking in the first cut.
- **Both outcomes recycle** the incoming ball (and the bullet); the run is
  **endless** — no sim-ending condition.
- "Destroyed" / "removed" = **recycle**, not true node deletion: `reset` the
  node (teleport away, zero velocity) and relaunch.

## Part 5 — ML pattern detector

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

The attacker's launch log is directly the ML's training set — the single
interface between the game and the ML.

## Turret-side follow-up

`atlas_controller` uses `getFromDef("PROJECTILE")` (singular). If multiple
incoming projectiles are ever alive at once it will need a naming scheme
(`PROJECTILE_0/1/2…`) plus list handling. `SearchRadar` already accepts a *list*
of projectile nodes, so that side is ready.

## Out of scope (future work)

- Actual ML training, feature tuning, hyperparameters.
- Consuming the prediction (e.g. pre-aiming the turret from the classification).
- A pool of incoming projectiles / turret bullets (vs single recycled nodes).
- Dynamic projectile spawning (vs recycling).
- Drag / damping physics ("hard mode").
