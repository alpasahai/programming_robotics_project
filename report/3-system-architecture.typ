= System Architecture
// TODO: Explain the system structure using the Sense → Think → Act model.
// Explain briefly how information flows through the system

#figure(
  image("system-architecture.pdf", width: 60%),
  caption: [ATLAS system architecture diagram],
) <ARCH>
@ARCH illustrates how ATLAS conforms to the Sense $->$ Think $->$ Act embedded control
loop.

*Sense* drains four inputs each tick: the world-frame Search Radar cue on ch1
(`SearchRadarLink`), the turret-relative position from the on-board Fire Control Radar
when locked (`FireControlRadar`), and ground-hit / bullet-hit resolution pulses on ch2
and ch3.

*Think* turns those inputs into a chosen action. A 6-state Kalman filter
($[x, y, z, v_x, v_y, v_z]$) fuses the radar streams, weighting precise FCR
measurements far above the noisy Search cues; the `BallisticPredictor` propagates that
estimate to a closed-form intercept, and the `AttackPredictor` learns launch-sector
patterns online to pre-aim for the next engagement. The `AtlasFSM` reads this
always-fresh world model and arbitrates the next action, advancing through `IDLE` $->$
`AIM` $->$ `TRACK_PREDICT` $->$ `ENGAGING` $->$ `RESET` based on lock state,
convergence, and resolution cues.

*Act* turns FSM decisions into physical commands: pan/tilt `RotationalMotor` writes slew
the boresight, and the recycled `Bullet` is teleported to the muzzle and launched when
the FSM commits to a shot — subject to the anti-friendly-fire pan-angle gate before
arming.
