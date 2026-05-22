"""Tests for extract_features — context features for next-sector prediction."""
import numpy as np
from attack_features import extract_features


def test_width_is_max_sectors_plus_one():
    f = extract_features([0], max_sectors=4)
    assert f.shape == (5,)  # 4 one-hot + 1 run_length


def test_one_hot_marks_current_sector():
    f = extract_features([2, 0, 1], max_sectors=4)
    assert f[1] == 1.0                 # current sector is 1
    assert f[0] == 0.0 and f[2] == 0.0


def test_run_length_counts_trailing_repeats():
    assert extract_features([0], max_sectors=4)[-1] == 1.0
    assert extract_features([0, 0, 0], max_sectors=4)[-1] == 3.0
    assert extract_features([1, 1, 0, 0], max_sectors=4)[-1] == 2.0


def test_run_length_resets_on_switch():
    assert extract_features([2, 2, 2, 1], max_sectors=4)[-1] == 1.0
