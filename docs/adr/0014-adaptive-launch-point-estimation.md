# 0014 — Adaptive launch-point estimation (online, radar-only)

## Status
Accepted

## Context
The Advanced / Contextual Component (10%) requires an "AI or Adaptive Behaviour"
element. This is the lower-risk alternative to the launch-direction learner: a
classical online estimator that improves acquisition latency.

## Decision
ATLAS estimates the attacker's roughly-static launch origin online from the noisy
Search Radar cues it already receives (first acquisition after a resolution cue),
using an exponentially-weighted recursive mean per axis (one forgetting/adaptation
knob). It pre-aims the pure FSM IDLE state at the estimate (and may seed the
tracker), cutting acquisition latency; the estimate improves with experience and
adapts to slow drift. No new sensor/radio device, no dataset, no scikit-learn. The
five-state FSM structure (ADR-0012) is unchanged (only IDLE gains an input).

## Consequences
- Improves acquisition latency / pre-lock estimate, NOT terminal aim (the FCR is
  already precise once locked) — the demonstrable claim is framed accordingly.
- "Value of learning" is measurable: the estimate-vs-launch-number convergence
  curve and reduced time-to-lock.
- Cold start (no estimate yet) falls back to the fixed idle beam — a
  safety/robustness property.
