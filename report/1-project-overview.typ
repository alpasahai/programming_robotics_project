= Project Overview
// TODO: Briefly describe the goal of your project.
// Include:
// ● What problem the robot solves
// ● Type of robot/system
// ● Key features
// ● Environment where it operates
// Example points:
// ● Robot purpose
// ● Sensors used
// ● Actuators used
// ● Intelligent behaviour implemented

This report discusses the Autonomous Tracking and Laser Aiming System (ATLAS) project,
which is a robotic defense simulation developed and tested in Webots. The main purpose
of the system is to detect, track, predict, and intercept incoming projectile objects
within a bounded engagement sector using a stationary pan-tilt turret. ATLAS operates on
a cue-driven fire-control subsystem where a Search Radar first preforms a wide-area scan
before directing the target information to a precise Fire Control Radar (FCR) which is
located on the turret.


ATLAS solves the problem of autonomous projectile interception by utilising perception,
prediction, and real-time decision making. By using two radar-based perception sources,
the system can get a coarse target acquisition using the Search Radar and narrow-beam
precision tracking using the FCR. These sensors use the Kalman filter to help reduce
noise and help improve the target estimation accuracy. The system also predicts future
projectile trajectories and autonomously aims and fires towards the estimated point of
interception.


The system operates within a simulated Webots environment and contains realistic
projectile physics, turret motion, radar scanning, and autonomous target engagement. The
key actuators are the pan motor, tilt motor, and the projectile firing mechanism. This
project has also implemented intelligent behavior such as using a Finite State Machine
(FSM) control, adaptive attack-pattern prediction, Kalman-filter-based sensor fusion,
and engagement logic based on data confidence.
