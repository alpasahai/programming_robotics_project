= Technical Requirements Compliance
// TODO: In this section you must explicitly explain how your system satisfies all technical
// requirements listed in the assignment sheet.
// For each requirement, briefly describe how your system implements it.
// This section helps reviewers verify that your system meets the required specifications

This section maps ATLAS to each technical requirement; details live in the cited
sections. The figure and `FSM` label are defined here once and referenced elsewhere.

#figure(
  image("fsm.pdf", width: 30%),
  caption: [ATLAS finite state machine (FSM) diagram],
) <FSM>

- *Inputs ($gt.eq$ 2).* Search Radar cue (world-frame, via radio) and Fire Control Radar
  position (turret-relative, when locked). Full list and noise figures in
  @sec:perception.
- *Outputs ($gt.eq$ 2).* Pan/tilt motor `setPosition()` commands and a bullet launch
  (teleport-to-muzzle plus velocity write). Detailed in @sec:architecture.
- *FSM with $gt.eq$ 4 states.* Five states — `IDLE`, `AIM`, `TRACK_PREDICT`, `ENGAGING`,
  `RESET` (@FSM, @sec:behaviour).
- *Multi-condition decision logic.* `TRACK_PREDICT` $arrow$ `ENGAGING` requires
  prediction error below threshold AND intercept in range AND sustained for $N$ frames;
  FCR lock combines FOV-cone membership AND range. See @sec:perception.
- *Safety / fail-safe.* Convergence gate before firing, universal `RESET` recovery, and
  the anti-friendly-fire pan-angle gate. Detailed in @sec:safety.
- *Sensor fusion.* Heterogeneous Kalman fusion of the Search Radar ($R_"SEARCH" = 0.1$)
  and FCR ($R_"FCR" = 0.001$), with predict-every-tick for dropout safety. See
  @sec:perception.
- *Context-aware behaviour.* The `AttackPredictor` learns launch-sector patterns online
  and pre-slews the turret in `IDLE`; the convergence gate adapts to track volatility.
  See @sec:behaviour.
- *Real-time logic.* Synchronous Sense $arrow$ Think $arrow$ Act loop on the 32 ms
  Webots tick, with sensing and FSM evaluation running every tick. See
  @sec:architecture.
- *Advanced AI / adaptive component.* Online logistic regression (`SGDClassifier`,
  `partial_fit` per launch) over online-discovered sectors with EMA drift tracking — one
  learning step and one next-sector prediction per engagement. See @sec:behaviour.
