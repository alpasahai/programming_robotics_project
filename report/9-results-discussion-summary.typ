= Results / Demonstration Summary
// TODO: Describe what the robot successfully does:
// ● Detects motion
// ● Navigates around obstacles
// ● Coordinates multiple robots
// ● Interacts with a user
// Mention any limitations or challenges encountered

ATLAS successfully demonstrates autonomous projectile detection, tracking, prediction,
and interception within the Webots simulation environment. The Search Radar cues the
turret with coarse world-frame positions; the on-board FCR locks once the turret has
slewed onto the target; the Kalman `TrackFilter` and `BallisticTrajectoryPredictor` then
compute an intercept and the turret fires a bullet at it. Real Webots physics determine
the outcome — the bullet either strikes the projectile or the projectile reaches the
ground. Across the test run, ATLAS shoots down approximately *52%* of incoming
projectiles. The remainder fall to ground hits, mostly when the convergence gate hasn't
tripped before the projectile leaves the engagement range.

The system's safety and fail-safe behaviours — the convergence gate, universal `RESET`
recovery, and the anti-friendly-fire pan-angle gate — all operated as specified in the
test run. The main implementation challenges were modelling the Search Radar and FCR as
3D radars in Webots (the built-in radar is 2D only), characterising the projectile's
ballistic motion well enough for the intercept pipeline to converge, and building the
adaptive attack-pattern learner so that prediction quality improved monotonically across
engagements.
