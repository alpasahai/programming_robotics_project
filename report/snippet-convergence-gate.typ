#figure(
  text(size: 6pt)[
```python
# fsm.py — TRACK_PREDICT → ENGAGING convergence gate
intercept = self.sensors.ballistic_predictor.get_intercept(
    self.config.lookahead_steps
)
pred_error = self._record_intercept_and_error(intercept, position)

if (
    pred_error is not None
    and pred_error < self.config.track_error_threshold
    and self._target_within_range(intercept)
):
    self._converge_count += 1
else:
    self._converge_count = 0

if self._converge_count >= self.config.converge_frames:
    self._intercept = intercept
    self._transition(self.ENGAGING)
```
  ],
  caption: [Convergence gate: fires only on sustained low prediction error AND in-range intercept.],
) <code:convergence>
