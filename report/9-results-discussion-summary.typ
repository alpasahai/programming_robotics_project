= Results / Demonstration Summary
// TODO: Describe what the robot successfully does:
// ● Detects motion
// ● Navigates around obstacles
// ● Coordinates multiple robots
// ● Interacts with a user
// Mention any limitations or challenges encountered

ATLAS successfully demonstrates autonomous projectile detection, tracking, prediction,
and interception within the Webots simulation environment. The system can detect a
moving projectile and track them using the Search Radar process scans which send the
coarse world-frame cues to the turret. The Fire Control Radar (FCR) located on the
turret only locks onto the target when turret is physically skewed in that direction.
The turret can use the Kalman track filter and ballistic trajectory predictor to
estimate the projectile’s future path and compute the intercept point. The result is a
sensor-driven engagement pipleine where real projectile motion and collisions determine
whether the turret bullet (fired when all the conditions are met) hits the incoming
projectile, or the projectile hits the ground.



The system successfully demonstrates multiple safety and fail-safe behaviors including
confidence-gated engagement, RESET recovery, and restricted firing sectors. These
mechanisms implemented in ATLAS improve the reliability and robustness of the system,
especially during unexpected circumstances like unstable sensor conditions and target
loss. For this project, the main challenges faced include modeling the Search Radar and
FCR as 3D radars in Webots environment, figuring out the incoming projectile’s motion so
the prediction and intercept pipeline functioned correctly, and adding the adaptive
attack-pattern feature that learns launch sectors from observed cues.
