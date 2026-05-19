"""Search Radar scan geometry constants — Webots-free.

Extracted so both ``search_radar_controller.py`` and the test suite can
import the live shipped values without triggering Webots bootstrap code.

Dwell invariant
---------------
``beam_width >= 2 * scan_rate`` guarantees that the beam dwells on a target
for at least 2 consecutive update() steps per pass, so the beam cannot step
completely past the ball between timesteps.

  dwell (steps) ≈ beam_width / scan_rate

The values below are Webots-tuned starting points:
  - revisit rate  ≈ 21 steps  (≈ 0.67 s at 32 ms/step)
  - dwell         ≈ 2.3 steps

Raise both constants together if further tuning is needed, preserving the
``beam_width >= 2 * scan_rate`` invariant.
"""

# Coupled by the dwell invariant: beam_width >= 2 * scan_rate.
SEARCH_RADAR_BEAM_WIDTH_RAD: float = 0.70
SEARCH_RADAR_SCAN_RATE_RAD_PER_STEP: float = 0.30
