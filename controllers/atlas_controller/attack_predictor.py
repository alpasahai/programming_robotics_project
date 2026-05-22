"""AttackPredictor — online learner of the attacker's launch-direction pattern.

The Think layer of the adaptive feature. Fed one launch observation at a time
(bearing + time), it:
  1. discovers the launch sector via an online SectorMap (no preset count),
  2. updates an online classifier (P(next sector | context)) with partial_fit,
  3. predicts the next sector and exposes a ready-aim bearing for the FSM IDLE
     state to pre-slew toward.

It reasons about the *threat stream*, distinct from BallisticTrajectoryPredictor
(per-projectile). There is no dataset and no serialized model — it learns live.
Cold start: until two sectors have been observed (the classifier needs ≥2
classes to be useful), it reports no prediction and the FSM falls back to its
fixed idle aim. See the attack-plan spec.
"""
import numpy as np

from attack_features import extract_features


class AttackPredictor:
    """Online next-sector predictor feeding a ready-aim bearing to the FSM.

    Args:
        model:        an online classifier exposing ``partial_fit(X, y, classes=)``
            and ``predict(X)`` (e.g. sklearn ``SGDClassifier(loss="log_loss")``).
            Injected for testability.
        sector_map:   a ``SectorMap`` for online sector discovery.
        max_sectors:  capacity bound = one-hot width and class set size.
        pre_aim_tilt: tilt (rad) to hold while pre-aiming at the predicted sector.
        min_sectors_seen: cold-start gate — no prediction until this many distinct
            sectors have been observed.
    """

    def __init__(self, model, sector_map, max_sectors, pre_aim_tilt,
                 min_sectors_seen=2):
        self._model = model
        self._sector_map = sector_map
        self._max_sectors = max_sectors
        self._pre_aim_tilt = pre_aim_tilt
        self._min_sectors_seen = min_sectors_seen
        self._history: list[int] = []
        self._seen: set[int] = set()
        self._fitted = False
        self._predicted_sector: int | None = None

    @property
    def predicted_sector(self):
        """The currently predicted next sector id, or None during cold start."""
        return self._predicted_sector

    def observe(self, bearing, time) -> int:
        """Ingest one launch: discover its sector, learn online, re-predict.

        Returns the SectorMap id the launch bearing was assigned to (the just-
        discovered/matched sector for this launch). The controller uses it to
        score the pre-aim it had been holding for this launch.
        """
        sector = self._sector_map.observe(bearing)

        # Online update: the just-arrived sector is the label for the context
        # that preceded it. Needs a prior context (history non-empty).
        if self._history:
            X = extract_features(self._history, self._max_sectors).reshape(1, -1)
            y = [sector]
            if not self._fitted:
                self._model.partial_fit(X, y, classes=np.arange(self._max_sectors))
                self._fitted = True
            else:
                self._model.partial_fit(X, y)

        self._history.append(sector)
        self._seen.add(sector)

        # Predict the next sector from the updated context.
        if self._fitted and len(self._seen) >= self._min_sectors_seen:
            X_next = extract_features(self._history, self._max_sectors).reshape(1, -1)
            self._predicted_sector = int(self._model.predict(X_next)[0])
        else:
            self._predicted_sector = None

        return sector

    def get_ready_aim(self):
        """Return (pan, tilt) toward the predicted sector, or None.

        Returns None during cold start AND when the model predicts a sector the
        SectorMap has not discovered yet (the classifier's fixed class set spans
        max_sectors, but only the sectors actually observed have a known bearing).
        In both cases the FSM IDLE state falls back to its fixed idle beam.

        ``pan`` is the predicted sector's discovered centre bearing (the turret's
        pan convention is pan = atan2(dx, dy), the same value SectorMap stores);
        ``tilt`` is the configured pre-aim elevation.
        """
        if self._predicted_sector is None:
            return None
        if self._predicted_sector >= len(self._sector_map):
            return None
        return (self._sector_map.bearing(self._predicted_sector), self._pre_aim_tilt)
