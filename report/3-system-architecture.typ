= System Architecture <sec:architecture>
// TODO: Explain the system structure using the Sense → Think → Act model.
// Explain briefly how information flows through the system

#figure(
  image("system-architecture.pdf", width: 60%),
  caption: [ATLAS system architecture diagram],
) <ARCH>
Every 32 ms tick the ATLAS controller runs the three-stage Sense $->$ Think $->$ Act
loop in order; @ARCH groups its components by stage, with arrows showing the data that
crosses each boundary. Information flows in one direction each tick: raw sensor reads
$arrow$ world snapshot (Sense) $arrow$ filtered state $plus$ chosen action (Think)
$arrow$ motor and bullet writes (Act).

*Sense* reads the Webots devices — radio receivers, the on-board FCR, and the pan/tilt
joint-angle sensors — and emits a fixed-shape world snapshot for the tick: a Search
Radar cue (or none), an FCR position (when locked), any hit pulses, and the measured
boresight. No interpretation happens here. Noise figures and per-stream behaviour are in
@sec:perception.

*Think* takes the snapshot and produces a chosen action through three activities:
*estimation* (the Kalman `TrackFilter` maintains a fused position-velocity estimate),
*prediction* (the `BallisticPredictor` extrapolates an intercept and the
`AttackPredictor` learns launch-sector patterns online), and *decision* (the `AtlasFSM`
arbitrates via guarded transitions over those products). The output is a target pan/tilt
and a fire flag. The state machine is detailed in @sec:behaviour.

*Act* turns that decision into hardware writes: `setPosition()` on the pan and tilt
motors, and — when the FSM commits to a shot — teleport plus velocity writes on the
recycled bullet `Solid`. A pan-angle friendly-fire gate sits between the FSM and the
bullet to suppress shots that would cross the Search Radar's bearing.
