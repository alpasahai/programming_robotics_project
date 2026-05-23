= Behavioural Model
// TODO: Describe how the robot behaves, including:
// ● FSM / Behaviour Tree / Control Logic
// ● Minimum 4 behavioural states
// ● Conditions that trigger state transitions
// Explain why this behaviour model was chosen

As mentioned, ATLAS uses a finite state machine (FSM) to manage autonomous target
detection, tracking, prediction, and engagement. FSM control was selected because it
provides deterministic behaviour, modular state transitions, and a reliable fail-safe
recovery during the uncertain tracking conditions. ATLAS implements the following five
behavioral states: IDLE, AIM, TRACK_PREDICT, ENGAGING, and RESET.


The system starts in the IDLE state where the Fire Control Radar (FCR) waits for the
target cue information from the Search Radar subsystems. During this, the turret may
pre-aim towards the attack sectors, predicted by the adaptive AttackPredictor module
(get_ready_aim()). The Search Radar continuously broadcasts approximate projectile
locations, and the FSM only transitions after a valid cue has been observed for multiple
frames. The debounce logic prevents incorrect transitions caused by noisy or
intermittent detections. Once stable cue is received from the Search Radar, the AIM
state handles turret motion so that it slews towards the approximate projectile
position. The FCR has not locked on to the target since the fused Kalman estimate is not
yet reliable. The turret aims using raw cue measurements until the projectile enters the
FCR’s narrow field-of-view cone. When fcr.is_locked() becomes true, the TRACK_PREDICT
stage begins.


The TRACK_PREDICT state preforms the main interception logic of the system. This state
functions as the unified sensor-fusion and prediction pipeline as during this phase the
Search Radar’s measurements and the FCR’s observations are combined using the
Kalman-based TrackFilter. The Kalman filter continuously estimates projectile’s position
and velocity while reducing sensor noise and stabilising trajectory estimation. The
turret tracks the filtered projectile position and computes the predictions for future
intercept points; a prediction history is maintained throughout this process. This
enables the system to compare previously predicted positions against the newly observed
target positions. Once the prediction error remains consistent (below a predefined
threshold), the prediction is considered reliable thus leading to the predicted
intercept point being stored and FSM transitioning to ENGAGING.


During the ENGAGING state, the turret maintains aim on the fixed intercept point
calculated during TRACK_PREDICT and fires a bullet toward the predicted intercept point.
There are no predictions occurring during the engagement as the system assumes that the
prediction has converged sufficiently. The ENGAGING stage terminates if projectile has
been successfully intercepted and destroyed, projectile hits the ground, or the
projectile has left the valid engagement range. The RESET stages act as a fail-safe
recovery mechanism as it clears all the temporary engagement state that has accumulated
during the previous interception cycle before returning to IDLE. This means resetting
the Kalman Filter memory, clearing the stale Search Radar cues, resetting the prediction
histories and clearing the intercept targets. This helps to prevent the stale sensor
information from incorrectly triggering future engagements and allowing every
interception cycle to begin from a clean system state.


The FSM architecture is essential in cleanly separating detection, aiming, tracking and
prediction, engagement and recovery into independent behavioral stages. This approach
improves the system’s readability, maintainability, and robustness while ensuring
reliable autonomous operation within a real-time robotics environment.
