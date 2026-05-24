#figure(
  text(size: 6pt)[
```python
# attack_predictor.py — one learning step + one prediction per launch
def observe(self, bearing, time) -> int:
    sector = self._sector_map.observe(bearing)

    # Online update: previous context → current sector label.
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
```
  ],
  caption: [Online learning: one `partial_fit` per launch, then predict the next sector.],
) <code:attack-predictor>
