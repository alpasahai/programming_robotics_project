# ADR-0014: Adaptive launch-direction learning (online, radar-only)

**Status:** Accepted

## Decision

ATLAS learns the attacker's launch-direction pattern online during the run:

- The attacker launches from multiple sectors on a structured-but-noisy,
  drifting pattern, designed so a statistical learner is required and beats naive
  baselines.
- ATLAS observes launches radar-only (first acquisition after a resolution cue) —
  no launch-announcement radio channel — and discovers the sectors itself by
  online angular clustering (no preset count/boundaries; tolerance derived from
  radar noise; cluster count bounded by a capacity cap).
- The launch observation is segmented by `LaunchLatch`, a pure state machine that
  yields exactly one observation per engagement: the first fresh radar cue AFTER
  the previous engagement's cue has been cleared (FSM RESET). This avoids
  training on the resolved projectile's stale in-flight cue — the cue is cleared
  one step after a resolution is signalled, so the latch waits to see the cue go
  empty before accepting the next fresh cue.
- An online logistic regression (`SGDClassifier`, log-loss, `partial_fit`)
  estimates `P(next sector | context)`; its learning rate provides forgetting so
  it tracks drift. One meaningful hyperparameter (the adaptation rate).
- The prediction feeds the pure FSM `IDLE` state as a ready-aim bearing
  (Layer-3 strategic suggestion → Layer-2 FSM); the five-state structure
  (ADR-0012) is unchanged.
- No synthetic data, no collected dataset, no serialized model — learned live.

## Context

The Advanced / Contextual Component (10%) requires a genuine "AI or Adaptive
Behaviour" element. Earlier drafts predicted launch cadence offline and/or with
synthetic data; that read as offline analysis, not adaptive behaviour, and risked
being a lookup rather than learning.

## Reasoning

**Online direction learning, not cadence:** Launch direction encodes the
attacker's strategic choice each engagement; cadence is a fixed parameter. A
learner over directions has signal to exploit and must genuinely update its
belief as the pattern drifts — it cannot be satisfied by a lookup table or a
replay of a fixed schedule.

**Radar-only observation, segmented by LaunchLatch:** The Search Radar's first
acquisition after a resolution cue is the earliest observable signal of a new
launch. Using it directly avoids a dedicated launch-announcement channel (which
would make the learning trivial). `LaunchLatch` ensures the observation is from
the new launch, not a residual cue from the prior in-flight projectile.

**`SectorMap` for sector discovery:** Presetting sector boundaries would require
knowledge of the attacker's layout before the run. Online clustering lets ATLAS
discover the sectors itself, with the only prior being the clustering tolerance
(derived from radar noise) and a capacity cap. This keeps the system genuinely
adaptive and avoids any encoded map of the scenario.

**`SGDClassifier` + `partial_fit` for P(next sector | context):** Online
logistic regression provides calibrated probabilities, a controllable adaptation
rate (learning rate = forgetting), and a single hyperparameter to tune. It is
well-understood, ships with scikit-learn, and is straightforwardly testable
against predict-last / predict-mode baselines.

**Layer separation preserved:** The predictor is a strategic hint, not a
command. The FSM `IDLE` state holds the final aim decision; the predictor's
ready-aim bearing is a pre-slew suggestion. ADR-0012's five-state structure is
unchanged.

## Consequences

- Adds a `scikit-learn` dependency.
- The "value of ML" is measurable: prediction accuracy beats predict-last /
  predict-mode baselines, and recovers after a mid-run drift.
- Cold start (no prediction until ≥2 sectors seen) falls back to the fixed idle
  beam — a safety/robustness property.
- The observed launch bearing is the projectile's first radar acquisition, which
  is near — but not exactly at — the launch point (a small mid-flight
  displacement). Direction, not exact position, is the headline, and the wide
  sector spacing keeps this well within the clustering tolerance; the latch takes
  the first fresh cue to minimise the displacement.
- Launch timing is approximated by first-acquisition time (small constant lag),
  acceptable because direction, not timing, is the headline.
