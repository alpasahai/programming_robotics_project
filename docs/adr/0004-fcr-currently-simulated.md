# ADR-0004: Fire-Control Radar Currently Simulated

**Status:** Superseded by ADR-0008

## Decision

The Fire-Control Radar (FCR) is currently simulated using Webots Supervisor `getPosition()` with low Gaussian noise, rather than a real Webots sensor node (e.g. RangeFinder, Radar device).

## Context

ATLAS is designed around a physical narrow-beam FCR on the turret. Adding a real Webots sensor node requires modelling beam width, scan rate, and detection range — significant scope for the current iteration.

## Reasoning

Simulating the FCR via `getPosition()` + low noise allows the FSM, Track Filter, and Prediction Module to be built and validated end-to-end before hardware modelling is added. The FCR lives in its own class (`fire_control_radar.py`) with a defined interface, so replacing the simulation backend with a real sensor does not affect any other component.

## Upgrade path

Replace the `getPosition()` call in `FireControlRadar` with a Webots `Radar` or `RangeFinder` device. The output interface (relative Cartesian Track Report) stays the same. The rest of the system is unaffected.
