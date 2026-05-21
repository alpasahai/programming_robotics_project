"""Per-step telemetry for the ATLAS turret — a development instrument.

This is NOT part of the fire-control system. It exists to make a running
simulation legible and to validate the estimator/predictor against ground
truth: it reads the projectile's true position (``projectile.getPosition()``),
which a real fire-control system would never have, and compares it to the
filter estimate and to the intercept that was predicted some steps earlier.

Keeping it here (rather than inline in atlas_controller's loop) keeps the loop
to pure orchestration and makes the prediction-accuracy bookkeeping
unit-testable. Verbosity is controlled centrally via lib/atlas_logging.py — set
AtlasController to INFO to silence the per-step DEBUG trace.
"""

from collections import deque

_ORIGIN = [0.0, 0.0, 0.0]


def _distance(a, b):
    """Euclidean distance between two 3-vectors."""
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5


class TrackTelemetry:
    """Logs a per-step view of what the FSM is working from, and tracks how
    accurate the prediction made ``lookahead_steps`` ago turned out to be."""

    def __init__(
        self,
        log,
        *,
        lookahead_steps,
        max_range,
        ground_threshold,
        turret_position,
        timestep_ms,
    ):
        self._log = log
        self._lookahead_steps = lookahead_steps
        self._max_range = max_range
        self._ground_threshold = ground_threshold
        self._turret_position = list(turret_position)
        self._timestep_ms = timestep_ms
        # Past intercepts, so each step can compare the prediction made
        # lookahead_steps ago against where the projectile actually ended up.
        # When full, pred_history[0] is the prediction that targeted this step.
        self._pred_history = deque(maxlen=lookahead_steps + 1)

    def record_intercept_and_error(self, intercept, true_rel):
        """Append this step's intercept and return the prediction error for the
        intercept that targeted *this* step, or None while warming up.

        Pure bookkeeping over the rolling history — the unit-testable core.
        """
        self._pred_history.append(intercept)
        if len(self._pred_history) == self._pred_history.maxlen:
            return _distance(true_rel, self._pred_history[0])
        return None  # not enough history yet

    def log_legend(self):
        """Emit the one-time legend explaining each telemetry field."""
        log = self._log
        log.info("=" * 78)
        log.info("ATLAS telemetry legend")
        log.info("  All positions below are TURRET-RELATIVE metres [dx, dy, dz] — the")
        log.info("  offset from the turret, which sits at [0,0,0] in this frame.")
        log.info("  filtered_pos  : Kalman filter's estimate of where the projectile is NOW")
        log.info("  filtered_vel  : Kalman filter's estimate of projectile velocity (m/s)")
        log.info("  intercept     : predicted aim point — where the projectile is expected")
        log.info(
            "                  to be %d timesteps ahead (the point PREDICT validates)",
            self._lookahead_steps,
        )
        log.info("  true_pos      : actual projectile position, un-noised ground truth")
        log.info(
            "                  (shown beside filtered_pos for comparison; FSM never sees it)"
        )
        log.info("  filter_error  : distance between filtered_pos and true_pos — how")
        log.info("                  accurate the Kalman estimate is RIGHT NOW (lower = better)")
        log.info("  pred_error    : distance between true_pos NOW and the intercept that was")
        log.info(
            "                  predicted %d steps ago FOR now — how accurate the",
            self._lookahead_steps,
        )
        log.info("                  prediction was (spikes at projectile relaunch — expected)")
        log.info("  range_check   : intercept's distance from turret vs config.max_range")
        log.info("  ground_check  : intercept's height (z) vs config.ground_threshold")
        log.info("-" * 78)
        log.info(
            "turret_position (WORLD frame, fixed) = %s",
            [round(v, 3) for v in self._turret_position],
        )
        log.info(
            "timestep = %d ms   lookahead_steps = %d   max_range = %.1f m   "
            "ground_threshold = %.2f m",
            self._timestep_ms,
            self._lookahead_steps,
            self._max_range,
            self._ground_threshold,
        )
        log.info("=" * 78)

    def report(
        self,
        step_count,
        sim_time,
        fsm_state,
        track_filter,
        ballistic_predictor,
        projectile_node,
    ):
        """Log one step of telemetry comparing estimate/prediction to truth."""
        if not track_filter.is_initialised():
            # No estimate yet — before the first fuse, and for one step after
            # ACQUIRE calls track_filter.reset().
            self._log.debug(
                "FSM=%s   step=%d   t=%.2fs\n"
                "    (track filter not initialised — no estimate this step)",
                fsm_state,
                step_count,
                sim_time,
            )
            return

        fpos = track_filter.get_position()
        fvel = track_filter.get_velocity()
        icept = ballistic_predictor.get_intercept(self._lookahead_steps)
        true_rel = [
            projectile_node.getPosition()[i] - self._turret_position[i]
            for i in range(3)
        ]

        filter_error = _distance(fpos, true_rel)

        pred_err = self.record_intercept_and_error(icept, true_rel)
        pred_error = "%.3f m" % pred_err if pred_err is not None else "n/a (warming up)"

        dist = _distance(icept, _ORIGIN)
        in_range = dist <= self._max_range
        above_ground = icept[2] > self._ground_threshold

        self._log.debug(
            "FSM=%s   step=%d   t=%.2fs\n"
            "    filtered_pos = %-22s true_pos = %-22s filter_error = %.3f m\n"
            "    filtered_vel = %s\n"
            "    intercept    = %-22s pred_error = %s\n"
            "    range_check  : %.2f m vs max %.1f m   -> %s\n"
            "    ground_check : intercept height %.2f m vs min %.2f m   -> %s",
            fsm_state,
            step_count,
            sim_time,
            str([round(v, 2) for v in fpos]),
            str([round(v, 2) for v in true_rel]),
            filter_error,
            str([round(v, 2) for v in fvel]),
            str([round(v, 2) for v in icept]),
            pred_error,
            dist,
            self._max_range,
            "IN-RANGE" if in_range else "OUT-OF-RANGE",
            icept[2],
            self._ground_threshold,
            "ABOVE-GROUND" if above_ground else "BELOW-GROUND",
        )
