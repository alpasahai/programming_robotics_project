"""Tests for AttackPredictor — online next-sector learner feeding the FSM."""
import os, sys
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), "..", "..", "attacker_controller"))

import itertools
import math
import numpy as np
from sklearn.linear_model import SGDClassifier

from attack_predictor import AttackPredictor
from sector_map import SectorMap
from attack_pattern import PatternRule, launch_sequence  # reused from attacker pkg path

MAX_SECTORS = 8
PRE_AIM_TILT = 0.3


class StubModel:
    """Records partial_fit calls; predict returns a fixed sector."""

    def __init__(self, prediction=0):
        self.fits = []
        self._prediction = prediction

    def partial_fit(self, X, y, classes=None):
        self.fits.append((X.tolist(), list(y), None if classes is None else list(classes)))

    def predict(self, X):
        return [self._prediction]


def _bearings_for(sectors):
    """Map sector ids to well-separated bearings for SectorMap to rediscover."""
    table = {0: 0.0, 1: 1.5, 2: 3.0, 3: -1.5}
    return [table[s] for s in sectors]


def test_observe_returns_discovered_sector_id():
    """observe() returns the SectorMap id the launch bearing was assigned to."""
    sm = SectorMap(tol_rad=0.3)
    p = AttackPredictor(StubModel(), sm, max_sectors=MAX_SECTORS, pre_aim_tilt=PRE_AIM_TILT)
    s0 = p.observe(0.0, 0.0)    # first sector → id 0
    s1 = p.observe(1.5, 1.0)    # new far bearing → id 1
    s0b = p.observe(0.02, 2.0)  # near first → id 0 again
    assert s0 == 0
    assert s1 == 1
    assert s0b == 0


def test_cold_start_reports_no_prediction():
    """Before two sectors are seen, get_ready_aim() is None."""
    p = AttackPredictor(StubModel(), SectorMap(tol_rad=0.3), max_sectors=MAX_SECTORS,
                        pre_aim_tilt=PRE_AIM_TILT)
    p.observe(bearing=0.0, time=0.0)              # first sector only
    assert p.get_ready_aim() is None


def test_first_partial_fit_passes_classes():
    """The first online update declares the full class set (SGD requirement)."""
    model = StubModel()
    p = AttackPredictor(model, SectorMap(tol_rad=0.3), max_sectors=MAX_SECTORS,
                        pre_aim_tilt=PRE_AIM_TILT)
    p.observe(0.0, 0.0)   # no fit yet (no prior context)
    p.observe(1.5, 1.0)   # now one transition → first partial_fit
    assert len(model.fits) == 1
    assert model.fits[0][2] == list(range(MAX_SECTORS))  # classes passed once


def test_ready_aim_uses_predicted_sector_bearing():
    """get_ready_aim() returns (predicted sector's centre bearing, pre_aim_tilt)."""
    model = StubModel(prediction=1)
    sm = SectorMap(tol_rad=0.3)
    p = AttackPredictor(model, sm, max_sectors=MAX_SECTORS, pre_aim_tilt=PRE_AIM_TILT)
    p.observe(0.0, 0.0)
    p.observe(1.5, 1.0)   # sectors 0 and 1 now seen; model predicts sector 1
    aim = p.get_ready_aim()
    assert aim is not None
    pan, tilt = aim
    assert abs(pan - sm.bearing(1)) < 1e-9
    assert tilt == PRE_AIM_TILT


def test_ready_aim_none_when_predicted_sector_undiscovered():
    """If the model predicts a class the SectorMap hasn't discovered yet,
    get_ready_aim() returns None (fall back to fixed beam) — never IndexErrors."""
    sm = SectorMap(tol_rad=0.3)
    model = StubModel(prediction=5)   # class 5 is in np.arange(8) but never observed
    p = AttackPredictor(model, sm, max_sectors=MAX_SECTORS, pre_aim_tilt=PRE_AIM_TILT)
    p.observe(0.0, 0.0)   # discovers sector 0
    p.observe(1.5, 1.0)   # discovers sector 1 → model now predicts class 5 (undiscovered)
    assert p.predicted_sector == 5            # property still reports the raw argmax
    assert p.get_ready_aim() is None          # but the FSM-facing aim is safely None


def test_get_ready_aim_never_raises_over_full_drifting_run():
    """End-to-end with the REAL classifier + SectorMap over the drifting pattern:
    get_ready_aim() must never raise and must only ever return a discovered
    sector's bearing (regression for the undiscovered-sector IndexError)."""
    rule_a = PatternRule(tour=[0, 1], mean_burst=[5, 5], deviation_prob=0.05)
    rule_b = PatternRule(tour=[2, 3], mean_burst=[5, 5], deviation_prob=0.05)
    sectors = list(itertools.islice(
        launch_sequence([(rule_a, 150), (rule_b, None)], np.random.default_rng(1)), 320))
    bearings = _bearings_for(sectors)

    sm = SectorMap(tol_rad=0.3, max_clusters=MAX_SECTORS)
    model = SGDClassifier(loss="log_loss", random_state=0)
    p = AttackPredictor(model, sm, max_sectors=MAX_SECTORS, pre_aim_tilt=PRE_AIM_TILT)

    for i, b in enumerate(bearings):
        aim = p.get_ready_aim()   # must NEVER raise
        if aim is not None:
            pan, tilt = aim
            assert tilt == PRE_AIM_TILT
            # pan must be an actually-discovered centre
            assert any(abs(pan - sm.bearing(j)) < 1e-12 for j in range(len(sm)))
        p.observe(b, float(i))


def test_beats_mode_baseline_on_structured_pattern():
    """A real SGD learner predicts next sector better than always-predict-mode."""
    rule = PatternRule(tour=[0, 1, 2], mean_burst=[5, 2, 4], deviation_prob=0.1)
    sectors = list(itertools.islice(
        launch_sequence([(rule, None)], np.random.default_rng(0)), 400))
    bearings = _bearings_for(sectors)

    model = SGDClassifier(loss="log_loss", random_state=0)
    p = AttackPredictor(model, SectorMap(tol_rad=0.3), max_sectors=MAX_SECTORS,
                        pre_aim_tilt=PRE_AIM_TILT)

    correct = total = 0
    for i, b in enumerate(bearings):
        pred = p.predicted_sector  # prediction made BEFORE seeing this launch
        if pred is not None and i > 50:        # skip warm-up
            total += 1
            correct += int(pred == sectors[i])
        p.observe(b, float(i))

    model_acc = correct / total
    mode = max(set(sectors), key=sectors.count)
    mode_acc = sum(s == mode for s in sectors[51:]) / len(sectors[51:])
    assert model_acc > mode_acc  # learning adds value over the naive baseline


def test_adapts_after_drift():
    """Accuracy recovers after the pattern drifts to a new rule."""
    rule_a = PatternRule(tour=[0, 1], mean_burst=[5, 5], deviation_prob=0.05)
    rule_b = PatternRule(tour=[2, 3], mean_burst=[5, 5], deviation_prob=0.05)
    sectors = list(itertools.islice(
        launch_sequence([(rule_a, 150), (rule_b, None)], np.random.default_rng(1)), 320))
    bearings = _bearings_for(sectors)

    model = SGDClassifier(loss="log_loss", random_state=0)
    p = AttackPredictor(model, SectorMap(tol_rad=0.3), max_sectors=MAX_SECTORS,
                        pre_aim_tilt=PRE_AIM_TILT)

    hits = []  # (index, correct?) after the drift at 150
    for i, b in enumerate(bearings):
        pred = p.predicted_sector
        if pred is not None:
            hits.append((i, int(pred == sectors[i])))
        p.observe(b, float(i))

    just_after = [c for i, c in hits if 150 <= i < 175]
    well_after = [c for i, c in hits if i >= 295]
    assert sum(well_after) / len(well_after) > sum(just_after) / len(just_after)
