"""Tests for launch_sequence — the attacker's structured/noisy/drifting pattern."""
import itertools
import numpy as np
from attack_pattern import PatternRule, launch_sequence


def _take(seq, n):
    return list(itertools.islice(seq, n))


def test_reproducible_under_seed():
    """Same seed → identical sequence (pure given the rng)."""
    rule = PatternRule(tour=[0, 1, 2], mean_burst=[5, 2, 4], deviation_prob=0.1)
    a = _take(launch_sequence([(rule, None)], np.random.default_rng(7)), 100)
    b = _take(launch_sequence([(rule, None)], np.random.default_rng(7)), 100)
    assert a == b


def test_all_sectors_in_tour_when_no_deviation():
    """deviation_prob=0 → every emitted sector is in the tour."""
    rule = PatternRule(tour=[0, 1, 2], mean_burst=[5, 2, 4], deviation_prob=0.0)
    out = _take(launch_sequence([(rule, None)], np.random.default_rng(1)), 200)
    assert set(out) <= {0, 1, 2}


def test_structure_produces_runs():
    """deviation_prob=0 → consecutive repeats (bursts) exist, not alternation."""
    rule = PatternRule(tour=[0, 1, 2], mean_burst=[5, 2, 4], deviation_prob=0.0)
    out = _take(launch_sequence([(rule, None)], np.random.default_rng(2)), 300)
    longest_run = max(
        len(list(g)) for _, g in itertools.groupby(out)
    )
    assert longest_run >= 3  # bursts of several launches occur


def test_drift_switches_distribution():
    """After phase A's count, sectors come from phase B's tour."""
    rule_a = PatternRule(tour=[0, 1], mean_burst=[4, 4], deviation_prob=0.0)
    rule_b = PatternRule(tour=[2, 3], mean_burst=[4, 4], deviation_prob=0.0)
    out = _take(launch_sequence([(rule_a, 40), (rule_b, None)], np.random.default_rng(3)), 120)
    assert set(out[:40]) <= {0, 1}
    assert set(out[60:]) <= {2, 3}  # well past the phase boundary
