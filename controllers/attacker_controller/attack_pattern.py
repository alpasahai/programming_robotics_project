"""The attacker's launch-direction pattern: structured, noisy, and drifting.

A pure, seeded generator of sector ids (ints). It is the attacker's *behaviour*,
NOT a training set — ATLAS never sees it; it only observes the resulting
launches via radar. See
docs/superpowers/specs/2026-05-22-attack-plan-and-adaptive-ml-design.md.

Model
-----
* Structure: a cyclic ``tour`` of sectors, each fired in a burst of a
  characteristic mean length (run-length structure).
* Noise: burst lengths are sampled stochastically around their mean, and with
  probability ``deviation_prob`` a launch deviates to a random other sector.
* Drift: ``launch_sequence`` takes a list of (rule, count) phases; when one
  phase's count is exhausted the generating rule switches, so the pattern the
  defender must learn changes mid-run.
"""
from dataclasses import dataclass


@dataclass
class PatternRule:
    """One stationary launch rule (one phase of the run)."""

    tour: list[int]            # cyclic sector order, e.g. [0, 1, 2]
    mean_burst: list[float]    # mean burst length per tour position, e.g. [5, 2, 4]
    deviation_prob: float = 0.1  # chance a launch jumps to a random other sector


def _sample_burst(rng, mean):
    """A stochastic burst length ≥ 1, centred on ``mean`` (Gaussian, ~30% spread)."""
    return max(1, int(round(rng.normal(mean, 0.3 * mean))))


def _random_other(rng, tour, current):
    """A sector from ``tour`` other than ``current`` (the deviation target)."""
    others = [s for s in tour if s != current]
    return current if not others else others[rng.integers(len(others))]


def launch_sequence(phases, rng):
    """Yield sector ids forever, walking through ``phases`` then holding the last.

    Args:
        phases: list of ``(PatternRule, count)``. ``count`` launches are emitted
            under that rule, then the next phase begins. The final phase should
            use ``count=None`` to emit indefinitely.
        rng:    a ``numpy.random.Generator`` (seed it for reproducibility).

    Yields:
        int sector ids.
    """
    for rule, count in phases:
        emitted = 0
        tour_idx = 0
        burst_left = _sample_burst(rng, rule.mean_burst[0])
        while count is None or emitted < count:
            if rng.random() < rule.deviation_prob:
                sector = _random_other(rng, rule.tour, rule.tour[tour_idx])
            else:
                sector = rule.tour[tour_idx]
                burst_left -= 1
                if burst_left <= 0:
                    tour_idx = (tour_idx + 1) % len(rule.tour)
                    burst_left = _sample_burst(rng, rule.mean_burst[tour_idx])
            emitted += 1
            yield sector
