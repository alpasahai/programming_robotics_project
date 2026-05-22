"""Feature extraction for next-sector prediction (shared by training & inference).

The feature vector for "given the launch history, which sector launches next?" is
one-hot(current sector) followed by the current run length (consecutive launches
from the current sector). The run-length feature lets a linear model learn that a
longer current burst makes a sector switch more likely. Width is fixed to
``max_sectors`` + 1 so SGDClassifier.partial_fit sees a constant feature
dimension and a fixed class set. See the attack-plan spec.
"""
import numpy as np


def extract_features(history, max_sectors):
    """Return the feature vector for predicting the sector AFTER ``history``.

    Args:
        history:     non-empty list of discovered sector ids (ints), oldest first.
        max_sectors: capacity bound; one-hot width.

    Returns:
        np.ndarray of shape (max_sectors + 1,): one-hot(current) ⊕ [run_length].
    """
    current = history[-1]
    run_length = 1
    for s in reversed(history[:-1]):
        if s == current:
            run_length += 1
        else:
            break
    features = np.zeros(max_sectors + 1, dtype=float)
    features[current] = 1.0
    features[-1] = float(run_length)
    return features
