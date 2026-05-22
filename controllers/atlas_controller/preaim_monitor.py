"""PreAimMonitor — scores the AttackPredictor's pre-aim against actual launches.

Pure telemetry helper (no I/O). Each launch, the controller feeds the sector it
had predicted BEFORE the launch, the sector actually observed, the bearing the
turret was pre-aimed at, and the actual launch bearing. The monitor tracks the
running prediction accuracy, a naive predict-last baseline (guess the previous
launch's sector), and the angular pre-aim error — the on-screen evidence that the
learner is adding value and is "trained enough". The controller logs summary().
"""
import math

_TWO_PI = 2.0 * math.pi


def _ang_diff(a, b):
    """Signed smallest angle a−b in (−π, π]."""
    return (a - b + math.pi) % _TWO_PI - math.pi


class PreAimMonitor:
    """Running predicted-vs-actual scoring for launch-sector pre-aim."""

    def __init__(self):
        self._n = 0
        self._correct = 0
        self._baseline_correct = 0
        self._prev_actual = None
        self._error_count = 0
        self._sum_abs_error = 0.0
        self._last_error_rad = None

    def record(self, predicted_sector, actual_sector, predicted_bearing, actual_bearing):
        """Record one launch outcome. predicted_* may be None during cold start."""
        if predicted_sector is not None:
            self._n += 1
            if predicted_sector == actual_sector:
                self._correct += 1
            if self._prev_actual is not None and self._prev_actual == actual_sector:
                self._baseline_correct += 1
            if predicted_bearing is not None and actual_bearing is not None:
                err = abs(_ang_diff(predicted_bearing, actual_bearing))
                self._last_error_rad = err
                self._sum_abs_error += err
                self._error_count += 1
        self._prev_actual = actual_sector

    @property
    def n(self):
        return self._n

    @property
    def accuracy(self):
        return None if self._n == 0 else self._correct / self._n

    @property
    def baseline_accuracy(self):
        return None if self._n == 0 else self._baseline_correct / self._n

    @property
    def last_error_deg(self):
        return None if self._last_error_rad is None else math.degrees(self._last_error_rad)

    @property
    def mean_error_deg(self):
        if self._error_count == 0:
            return None
        return math.degrees(self._sum_abs_error / self._error_count)

    def summary(self):
        """One-line running-stats string for logging."""
        if self._n == 0:
            return "no scored predictions yet (cold start)"
        acc = 100.0 * self.accuracy
        base = 100.0 * self.baseline_accuracy
        mean_err = self.mean_error_deg
        err_str = "n/a" if mean_err is None else f"{mean_err:.1f}°"
        return (f"acc {acc:.0f}% ({self._correct}/{self._n}) "
                f"vs predict-last {base:.0f}% | mean pre-aim err {err_str}")
