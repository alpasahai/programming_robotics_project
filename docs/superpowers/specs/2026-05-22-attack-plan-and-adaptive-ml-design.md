# Attack plan + adaptive launch-direction learning (design)

**Date:** 2026-05-22
**Status:** Proposed — spec for #44 (sub-issue of #34)
**Supersedes the ML portion of:** `2026-05-18-attack-pattern-and-ml-detection-design.md`
(that doc left ML consumption "out of scope" and predicted *cadence* only; this one
closes the loop and predicts *direction* with an online classical-ML model, so the
feature is genuine *adaptive behaviour*, not offline analysis).

## Why this exists

The assignment's **Advanced / Contextual Component (10%)** requires *at least one
advanced element*; we are taking **"AI or Adaptive Behaviour."** The spec is
explicit: *"Build a system that demonstrates intelligence, not just motion."*

The Week-4 *Embedded AI* lecture frames the win condition almost exactly:
- **"Adaptive"** = the FSM's transition inputs are *dynamic variables calculated
  in real time*, not hard-coded numbers.
- A **learning system** *improves from experience/data* rather than following
  fixed rules; *"Statistical (usually does Y)"* models beat brittle rules in
  *"complex, messy environments."*
- **"Layer 3: Strategic AI"** watches patterns and sends *suggestions* down to the
  reactive FSM (Layer 2).
- Its headline example of intelligence is *predict-and-act-ahead*: notice a
  recurring pattern, pre-calculate, and be **already in position when the event
  arrives** (the school-crowd crossing).

So the feature is a visible **Sense → Think → Act** loop built around *where* the
attacker launches from:

- **Sense** — ATLAS observes each launch *entirely from its own Search Radar*: the
  bearing at which a new projectile is first acquired is the launch direction. No
  cooperative signal from the attacker is used.
- **Think** — a **classical statistical ML classifier, trained online during the
  run** on ATLAS's own noisy observations, learns the attacker's launch pattern
  and predicts the **next launch sector**.
- **Act** — ATLAS pre-aims the idle turret at the predicted sector, so it is
  already pointed when the next launch comes (shorter time-to-lock) — and
  **visibly re-adapts** when the attacker's pattern drifts mid-run.

This spec covers two co-designed halves, because the attack plan is built *to make
classical statistical ML genuinely pay off* on the defensive side:

1. **The attack plan** (#34 / #44) — a structured-but-noisy, drifting launch pattern.
2. **The adaptive learning feature** — the online classifier that learns it and
   changes ATLAS's behaviour.

Each part is split into **MUST** (the minimum that satisfies the advanced rubric
and is demonstrable in the video), **SHOULD** (refinements), and **COULD** (only
if time remains).

### Key decisions (resolved in brainstorming)

- **Predict direction, not cadence.** Direction prediction is *visible* (the turret
  physically swings to the right bearing before the threat appears), whereas
  cadence prediction is invisible. It is also a more honest ML problem.
- **The attack pattern is designed for classical statistical ML.** There is a
  **real, persistent underlying pattern** (structure ATLAS can learn), but it is
  **corrupted by noise** and **drifts over time**. Recovering it reliably requires
  *estimating it statistically from many samples* — exactly what classical
  statistical ML does, and exactly what a lookup table or exact string-matcher
  cannot. This is the core design principle.
- **Online classical-ML classifier with forgetting.** An online **logistic
  regression** (`SGDClassifier`, log-loss, `partial_fit`) estimates
  `P(next sector | context)` from the noisy stream and predicts the most probable
  next sector. Its constant learning rate provides natural **forgetting**, so the
  estimate tracks drift. The learning rate is the single, *meaningful* adaptation
  knob (the dial between noise-robustness and adaptation speed) — replacing the
  pile of arbitrary knobs (window length, tree depth) earlier drafts carried.
- **No training dataset — learn live.** There is **no synthetic data and no
  pre-collected dataset and no committed model artifact**. ATLAS learns purely from
  the launches it observes during the run.
- **The attacker tells ATLAS nothing.** There is **no launch-announcement radio
  channel**. ATLAS detects launches from its own radar (first acquisition of each
  new projectile), segmented by the ground-hit / bullet-hit resolution cues it
  already receives.
- **ATLAS discovers the sectors itself.** It is **not told how many launch
  directions exist or where they are** — it discovers them by online-clustering the
  observed bearings. Fully perception-driven, not a lookup against preset boundaries.

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
observation. The only design constraint is that distinct sectors are far enough
apart in bearing to be separable under the Search Radar's bearing noise.

### Pattern generation — structured, noisy, and drifting

The attack plan is **not a fixed string** — it is a structured *rule* with noise,
so that a statistical model is required to recover it:

- **Underlying structure (the "general pattern"):** the next launch sector is a
  function of recent context — primarily the **current sector** and the **length
  of the current burst**. The attacker fires a burst of several launches from one
  sector, with the probability of moving on rising as the burst lengthens, then
  transitions to the next sector in a characteristic tour (e.g. tends to fire
  ~5 from A, ~2 from B, ~4 from C). This yields the run-length structure we want
  *without* fixed counts.
- **Noise:** burst lengths are **stochastic** (drawn around their characteristic
  mean, not exact), and with a small probability ε the attacker **deviates** to an
  off-pattern sector. On top of this, the radar bearing ATLAS observes is itself
  noisy. So no two runs look the same and there is no exact sequence to memorise.
- **Drift (the adaptive payoff):** partway through the run the underlying rule
  **changes** — the favoured tour and/or burst lengths shift. This is the
  "pattern changes over time" the feature is built to handle.
- A pure, seeded `build_pattern(rule, rng) -> Iterator[sector]` (plus launch
  *times*) drives the attacker → reproducible and unit-testable. **This is the
  attacker's behaviour, not a training set** — ATLAS never sees it; it only
  observes the resulting launches.
- Launch **timing** keeps a base period + bounded jitter so **one projectile is
  alive at a time** (period > worst-case resolve time of destroy-or-land). Timing
  is secondary — it only sets the optional pre-slew lead time, not the headline.

### Mid-run pattern drift (the demo highlight)

Partway through, the attacker switches to a different underlying rule (different
favoured sectors / burst lengths). Because ATLAS learns online with forgetting,
its predicted-sector distribution follows the change within a few bursts —
**visible re-adaptation on screen**, the strongest evidence of "intelligence, not
motion."

### No new radio channel — the attacker stays silent about launches

The attacker emits ground-hit (ch 2) and bullet-hit (ch 3) *resolution* truth
(see `2026-05-21-attacker-ground-hit-cue-design.md`), but it does **not** announce
launches. There is **no new emitter/receiver and no channel 4.** Whether a launch
happened, when, and from where is left entirely to ATLAS to perceive — more
realistic (an adversary would not broadcast "I just fired") and a stronger AI
story. The attacker side therefore only changes its *behaviour* (structured/noisy/
drifting pattern); it gains no new device.

## Part 2 — The adaptive learning feature (ATLAS side)

### Task framing

**Multiclass classification: predict the next launch sector.** Given a feature
vector describing recent launch history, predict which sector the *next* launch
will come from, then pre-aim the turret at that sector's bearing. The model
estimates `P(next sector | context)` online and predicts its mode.

### Observing a launch (radar-only, segmented by resolution cues)

ATLAS turns its raw radar stream into discrete launch observations with no help
from the attacker:

- **Launch event = first fresh Search Radar acquisition of a new projectile.**
  With one projectile alive at a time, the previous projectile's resolution
  (ground-hit ch 2 / bullet-hit ch 3, which already drive the FSM `RESET`) marks
  the boundary; the *next* acquisition after that is the next launch. This
  rising-edge-after-resolution rule debounces mid-flight detection dropouts.
- **Launch bearing = the bearing at that first acquisition.** Bearing is
  `atan2(dx, dy)` (the convention the FSM already uses). Taken at *first*
  acquisition, the projectile is still near its spawn point, so the bearing
  reflects the launch sector, not its later in-flight position.
- **Launch time = the first-acquisition time** — lags the true launch by a small,
  roughly constant detection latency; irrelevant to direction, only a constant
  offset on the optional ETA pre-slew.
- This reuses the existing `SearchRadarLink` (acquisition + bearing) together with
  the ground-hit / bullet-hit links (segmentation) — a modest **sensor-fusion** of
  acquisition and resolution signals to carve the threat stream into labelled
  launches. (The heavier search-radar × FCR fusion already lives in tracking.)

### Discovering the sectors (unsupervised, online — no preset count or boundaries)

ATLAS is not given the number of launch directions or their bearings. It
**discovers them** by online-clustering the stream of observed bearings:

- **Online 1-D angular leader-clustering** (a `SectorMap`): maintain discovered
  cluster centres (bearings). For each new launch bearing, if it lies within an
  angular tolerance `tol` of an existing centre, it joins that cluster (and nudges
  the centre); otherwise it **opens a new cluster** with a fresh stable id. The
  number of sectors *emerges* from the data — no `k`, no preset boundaries.
- Chosen over `KMeans`/`MiniBatchKMeans` (which need `k`) precisely because it
  discovers the cluster count itself, is trivial and explainable, handles angle
  wrap-around, and assigns **stable ids** so the classifier sees consistent labels.
- **`tol` is grounded in the radar's known bearing noise** (derived from
  `SEARCH_RADAR_NOISE_STD_M`, a measured physical quantity), not hand-tuned: it is
  set to a few × the bearing-noise scale, which is comfortably smaller than the
  spacing between real sectors. It is a *perception* threshold, not a learning
  hyperparameter.

Each launch is thus labelled with a **discovered sector id**, with no knowledge of
the attacker's true sector set.

### Features

A pure `extract_features(history) -> np.ndarray`, shared by training and inference
(no train/serve skew):

- **MUST:** one-hot(current **discovered** sector id) ⊕ **run_length** (consecutive
  launches observed from the current sector). `run_length` is the feature that lets
  a linear model learn "the longer the current burst, the more likely the next
  launch switches sector" — a monotone relationship logistic regression captures
  naturally, and the statistical fit is what makes it robust to the noisy/stochastic
  burst lengths.
- **COULD:** one-hot(previous sector) for order-2 context; run-length bucketing;
  launch-interval stats.
- The one-hot width tracks the number of clusters discovered so far; the classifier
  is (re)fit over the current label set as new sectors appear.

### Model — online logistic regression (classical statistical ML, with forgetting)

- **`sklearn.linear_model.SGDClassifier(loss="log_loss")`** — multinomial logistic
  regression — updated **incrementally with `partial_fit`** on each observed launch
  (`(features_before_launch, observed_sector)`). It estimates
  `P(next sector | context)` and we predict the **argmax**.
- **Why this fits the pattern:** the pattern is statistical (structure + noise), so
  a probabilistic classifier that aggregates over many samples recovers the latent
  rule and **predicts the mode reliably despite noise** — where a lookup/exact-match
  would be derailed by a single deviation. Logistic regression is a textbook
  *classical statistical ML* classifier, fully explainable in the viva (learned
  weights / predicted probabilities).
- **Adaptation to drift:** with a constant learning rate, online SGD weights recent
  observations more, so the estimate **tracks the drifting pattern**. The learning
  rate is the **single meaningful hyperparameter** — the dial between noise-
  robustness and adaptation speed — and being able to explain that trade-off is
  itself a strong viva point.
- **`scikit-learn` must be added to `pyproject.toml`.** No `joblib` / serialized
  artifact — the model is built and updated live, never loaded from disk.

### Online inference — `AttackPredictor` (ATLAS side)

A new class, sibling to `SearchRadarLink` / `AttackerGroundHitLink`. It consumes
the **existing** links — `SearchRadarLink` (acquisition + bearing) and the
ground-hit / bullet-hit links (resolution boundaries) — and needs **no new radio
device**:

- Owns the `SectorMap` (sector discovery), the `SGDClassifier`, and a small record
  of recent observations needed to compute `run_length` and intervals.
- Each step the controller hands it the current acquisition state + resolution
  flags; it detects the **first-acquisition-after-resolution** edge, giving one new
  launch observation `(bearing, time)`.
- On each such launch: bearing → `SectorMap` → discovered sector id → form
  `(features_before, observed_sector)` → `partial_fit` (one online update) →
  predict the **next** sector from the current context.
- Exposes `predicted_sector` (a discovered id), `predicted_bearing` (that cluster's
  centre from the `SectorMap`), `predicted_proba` (confidence), and
  `next_launch_eta` (`last_launch + mean_interval - now`).
- **Cold-start:** until the classifier has observed launches from at least two
  sectors (it needs ≥2 classes to fit), it reports `UNKNOWN`; the system behaves
  exactly as it does today (graceful fallback — a **safety/robustness** point).
- Distinct from the per-projectile `BallisticTrajectoryPredictor` —
  `AttackPredictor` reasons about the *threat stream*, matching (and sharpening)
  the `AttackPredictor` glossary entry in `CONTEXT.md` (update its wording from
  "attack frequency" to "next launch sector/origin").

### Why ML actually helps (the measurable claim)

Because the pattern is structure + noise, a statistical learner **measurably beats
naive baselines** — "predict same sector as last" and "predict the globally most
common sector." Reporting next-sector prediction accuracy of the model vs these
baselines (and showing it recover after a drift) is concrete evidence that the
learning adds value, not decoration. This is a clean figure for the report/viva.

### Act — closing the loop (this is what makes it "adaptive")

- **MUST:** `AttackPredictor` produces a **ready-aim bearing** (the predicted
  sector's pan/tilt). The controller passes it into the FSM `SensorSuite`; the
  `IDLE` state holds that bearing instead of the fixed `idle_pan`/`idle_tilt` when a
  prediction is available, and falls back to the fixed beam when the predictor
  reports `UNKNOWN`. The idle turret is thus **pre-aimed at the sector the next
  launch is predicted to come from** — a perception-driven decision satisfying "AI,"
  with a measurably shorter time-to-lock when the prediction is right.
- **SHOULD:** use `next_launch_eta` to time the pre-slew (lead the predicted
  launch); telemetry/log overlay of *predicted vs actual* sector + the model-vs-
  baseline accuracy.
- **The re-adaptation is the money-shot:** during a burst the prediction holds on
  the active sector; as a burst lengthens the model raises the switch probability
  and swings ahead to the next sector; and after a mid-run drift it re-learns the
  new pattern within a few bursts.

### FSM impact — kept minimal and pure

The FSM stays pure (no model, no I/O). `AttackPredictor` produces the ready-aim
bearing + ETA that the controller passes into the FSM `SensorSuite`; `IDLE`
consumes it as its hold direction / pre-slew trigger. This adds an input to `IDLE`
only — the **five-state structure (ADR-0012) is unchanged**. Dropping to the
static-ready-aim MUST is a one-line behaviour change, not a redesign. Hook point:
`_do_idle()` (`fsm.py:~241`), replacing the fixed `_command_angles(idle_pan,
idle_tilt)`.

## Data flow

```
attacker: build_pattern(rule, rng)  [structured + noisy + drifting] → choose sector's (spawn,velocity) → launch ball
          (no launch radio — attacker says nothing about the launch)
                              │
                              ▼
atlas:    Search Radar first-acquisition of new projectile      [existing SearchRadarLink]
          segmented by ground-hit / bullet-hit resolution cues  [existing ch 2 / ch 3]
          → launch observation (bearing, time)
          → SectorMap (online clustering)  → discovered sector id  (count + bearings discovered, not preset)
          → AttackPredictor: extract_features → SGDClassifier.partial_fit (online update, forgets stale data)
          → predict next sector (argmax P) → predicted_bearing (cluster centre) / next_launch_eta
          → FSM IDLE pre-aims at predicted sector  →  shorter time-to-lock; re-adapts on drift

no offline pipeline, no new radio channel: no synthetic data, no collected dataset, no committed model artifact.
```

## Testing

- `build_pattern`: deterministic under a seeded RNG; produces the intended
  structure (burst-then-tour) with stochastic burst lengths and ε deviations within
  spec; launch times respect the `period > resolve_time` invariant; a configured
  drift changes the generating rule at the intended point.
- `SectorMap` (sector discovery): from a canned bearing stream it discovers the
  correct *number* of clusters with no `k` given; assigns stable ids; bearings
  within `tol` join an existing cluster, a novel bearing opens a new one; handles
  angle wrap-around; `tol` is computed from the radar noise constant, not literal.
- `extract_features`: shape and values on hand-built histories; `run_length` counts
  consecutive same-sector launches and resets on switch; same function used by both
  `partial_fit` and inference (no skew).
- `AttackPredictor`: on a canned noisy pattern, after warm-up its accuracy **beats
  the "predict-last" and "predict-mode" baselines** (the value-of-ML assertion);
  reports `UNKNOWN` before ≥2 sectors seen; after an injected drift, accuracy
  recovers within a bounded number of launches (**adaptation** assertion);
  `predicted_bearing` maps to the right cluster centre; `next_launch_eta`
  arithmetic. Seed the RNG for determinism.
- launch-observation segmentation: a first acquisition after a resolution cue yields
  exactly one launch; a reacquired track within the same engagement does not produce
  a spurious second launch (debounce holds).
- FSM: `IDLE` adopts the ready-aim bearing when present; falls back to the fixed
  beam when the predictor reports `UNKNOWN` (cold-start safety).

## Report linkage

Feeds **§ Technical Requirements & System Integration** (the advanced AI/adaptive
element) and **§ System Architecture** (the Layer-3 strategic predictor feeding
suggestions to the Layer-2 FSM). Segmenting the threat stream by fusing radar
acquisition with the resolution cues is a concrete **sensor-fusion** example. The
model-vs-baseline accuracy and post-drift recovery are the **adaptive-behaviour**
evidence. The cold-start `UNKNOWN` fallback is a **Safety, Robustness &
Reliability** point. Learned weights / predicted probabilities are an
explainability artifact for **Individual Technical Understanding** / the viva.

## Scope summary

**MUST** — attacker launches from multiple sectors with a **structured + noisy +
drifting** pattern (no new radio device); ATLAS observes launches **radar-only**
(first-acquisition segmented by the existing resolution cues); **`SectorMap`
unsupervised online discovery** of the launch directions (no preset count/
boundaries; `tol` from radar noise); `AttackPredictor` with an **online
`SGDClassifier` (logistic regression, `partial_fit`)** predicting the next
discovered sector from a run-length feature; `IDLE` pre-aims at the predicted
sector; `UNKNOWN` cold-start fallback; **mid-run drift demonstrating re-adaptation**
and **accuracy above naive baselines** (the headline evidence).
**SHOULD** — `next_launch_eta`-timed pre-slew lead; predicted-vs-actual + model-vs-
baseline telemetry overlay.
**COULD** — order-2 context (previous sector); run-length bucketing; per-sector
launch velocities; attacker introducing a *new* sector mid-run (showcases live
discovery of a previously-unseen direction).

## Out of scope

- Predicting exact next-launch *timestamp* as a regression target.
- Multi-target tracking (one projectile alive at a time stays in force).
- Drag / damping physics ("hard mode").
- Any pre-trained or serialized model, synthetic dataset, or offline collection run
  — the model is always learned live from observed launches.
