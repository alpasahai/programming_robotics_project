#figure(
  text(size: 6pt)[
```python
# track_filter.py — heterogeneous Kalman fusion
def predict(self) -> None:
    u = np.array([[0.0], [0.0], [_GRAVITY]])
    self.kf.predict(u=u)

def update_fcr(self, measurement):
    self.kf.R = np.eye(3) * self._R_fcr  # low noise — trusted
    self.kf.update(np.array(measurement).reshape(3, 1))
    self._initialised = True

def update_search(self, measurement):
    self.kf.R = np.eye(3) * self._R_search  # high noise — weak nudge
    self.kf.update(np.array(measurement).reshape(3, 1))
    self._initialised = True
```
  ],
  caption: [Kalman fusion: gravity is a control input; per-source $R$ is swapped before each update so FCR dominates corrections.],
) <code:kalman>
