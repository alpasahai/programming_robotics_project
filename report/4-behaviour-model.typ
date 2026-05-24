= Behavioural Model
// TODO: Describe how the robot behaves, including:
// ● FSM / Behaviour Tree / Control Logic
// ● Minimum 4 behavioural states
// ● Conditions that trigger state transitions
// Explain why this behaviour model was chosen

ATLAS uses an FSM for autonomous target detection, tracking, prediction, and engagement.
FSM control gives deterministic behaviour, modular state transitions, and a guaranteed
fail-safe recovery path under noisy sensing. The five behavioural states are summarised
below; @FSM gives the full transition diagram.

*`IDLE`.* The turret holds its search beam, or pre-aims at the `AttackPredictor`'s next
predicted sector, while waiting for a Search Radar cue. On the very first cue of a new
launch, the `AttackPredictor` ingests the observed launch sector — in a single atomic
call it performs one online learning step on the just-observed launch and then uses the
freshly updated model to predict the sector after that, which `IDLE` immediately
consumes for pre-aim. The FSM moves to `AIM` once a cue has been present for several
consecutive frames, or falls through to `RESET` if a ground-hit cue arrives.

*`AIM`.* The turret slews toward the raw Search Radar cue; the Kalman estimate is not
yet trustworthy at this stage, so no fusion runs. As soon as the FCR locks on (the
target is inside the narrow FOV cone and within range) the FSM moves to `TRACK_PREDICT`.
A ground-hit cue again sends the FSM to `RESET`.

*`TRACK_PREDICT`.* Each tick the turret slews to the latest filtered position, which
keeps the projectile inside the FCR's narrow FOV so the FCR keeps returning fresh
turret-relative measurements — closing a tight track-and-aim loop. The `TrackFilter`
(Kalman) fuses those FCR measurements with the Search Radar cue into a smoothed
estimate, and the `BallisticPredictor` propagates it forward to a closed-form intercept.
A convergence gate compares each predicted intercept against the later observed
position; once the prediction error stays below threshold and the intercept stays in
range for several consecutive frames, the FSM moves to `ENGAGING`. A hit cue sends the
FSM to `RESET`.

*`ENGAGING`.* The turret holds aim at the now-frozen intercept and fires a bullet toward
it, subject to the anti-friendly-fire pan-angle gate. No further prediction runs in this
state. The FSM moves to `RESET` on a bullet-hit cue, a ground-hit cue, or if the target
leaves the engagement range.

*`RESET`.* Wipes the Kalman state, clears the stale Search Radar cue, and resets the
prediction history and intercept target so no stale data leaks into the next engagement.
Once reset is complete the FSM returns to `IDLE`.

The FSM cleanly separates detection, aiming, tracking + prediction, engagement, and
recovery into independent stages, improving readability, maintainability, and robustness
under real-time constraints.

// === REFERENCE: original prose version (kept for comparison; not included in build) ===
/*
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
*/
