= Technical Requirements Compliance
// TODO: In this section you must explicitly explain how your system satisfies all technical
// requirements listed in the assignment sheet.
// For each requirement, briefly describe how your system implements it.
// This section helps reviewers verify that your system meets the required specifications

This project satisfies all the technical requirements for this assignment through the
combination of autonomous sensing, decision-making, and robotic control systems. In
terms of inputs, ATLAS uses the perception of sources provided by the wide-area Search
Radar and the narrow-beam Fire Control Radar (FCR). These sensors are continuously
monitoring their surroundings and provide the target measurements for tracking and
engagement. In terms of outputs, key examples are the bullet system used when the target
is secured (bullet fires at point of intercept and destroys the projectile) and the pan
and tilt motors of the turret that respond to the projectiles positioning once the
radars have provided it with the co-ordinates.


FSM architecture was implemented containing the states IDLE, AIM, TRACK_PREDICT,
ENGAGING, and RESET. The state transitions occur dynamically based on target range,
tracking stability, prediction validity, and sensor confidence. This allows for
deterministic and explainable autonomous behavior. Multi-conditional decision logic has
been incorporated in areas such as the fail-safe mechanism to ensure that ATLAS does not
cause damage/harm and in the FSM states to ensure all relevant data has been secured
before moving to the next state e.g. for TRACK_PREDICT to enter ENGAGING, the exit
conditions must be so that the prediction error is low and the target is still in range
and stable. Fail-safe systems such as RESET recovery, prediction-confidence gating and
restructure engagement sectors for the sensors prevent unsafe use and firing.


ATLAS incorporates advanced adaptative behaviour components, specifically the
attack-pattern prediction system also supported by the Kalman filter. The attack-pattern
prediction system learns the launch patterns overtime using online machine learning and
then pre-rotates the turret towards the likely future attacks. The Kalman filtering is
used to adaptively smooth noisy sensor measurements and improve trajectory estimation.


In terms of Embedded Intelligence, sensor fusion is used through the integration of
external cure data and tracking measurements, taken from the Search Radar and FCR, to
improve accuracy and robustness. The FCR feeds the TrackFilter every timestep while the
Search Radar runs a separate process and reaches the FSM only as a world-frame cue.
Context-aware behaviour is shown through ATLAS’s confident decision making through the
constant gathering and monitoring of input data resulting in validity of predictions and
improved accuracy when a target is secured. Real-time logic is shown through the
immediate response provided by the pan and tilt motors of the current followed by the
bullet system that intercept the projectile once the co-ordinates are proved valid.
