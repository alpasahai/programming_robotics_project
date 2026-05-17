# ADR-0005: filterpy Over From-Scratch Kalman Implementation

**Status:** Accepted

## Decision

The Kalman filter in TrackFilter is implemented using the `filterpy` library
rather than a from-scratch numpy implementation.

## Context

Two options were considered:

1. **filterpy** — well-documented Python Kalman filter library. Handles the
   predict/update cycle, matrix bookkeeping, and numerical stability. Supports
   multiple sensor sources by swapping H and R between update() calls.

2. **From scratch (numpy)** — implement F, H, Q, R, P, K matrices manually.
   Roughly 40 lines for a 6-state linear filter. Full ownership of every line.

## Reasoning

filterpy is chosen for the initial implementation because:
- It reduces implementation risk during a time-constrained assessment project.
- The filterpy API is transparent — `kf.F`, `kf.H`, `kf.R`, `kf.Q` are all
  directly inspectable attributes. Every matrix can be explained under examination.
- A from-scratch implementation can be substituted later without changing any
  other class — TrackFilter is the only place filterpy is imported.

## Personal exploration

A from-scratch numpy implementation of the same filter is a personal learning
exercise — not a project requirement. If pursued, it is a single-file change
inside TrackFilter and does not affect any other component.
