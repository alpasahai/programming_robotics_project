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

The Autonomous Tracking and Laser Aiming System (ATLAS) is a simulated fire-control
system that detects, tracks, and neutralises airborne threats without human input,
operating in an outdoor Webots environment. It is split into two collaborative units: a
Search Radar that sweeps a wide area with a wide-beam radar sensor, and a
turret-mounted Fire-Control Radar (FCR) carrying a narrow-beam radar and projectile
launcher. The two units coordinate over emitter/receiver pairs, with rotational motors
driving the search sweep and the FCR's pan/tilt turret.

When the Search Radar spots a target, it cues the FCR, which locks on and fuses readings
from both radars through a Kalman filter to smooth noisy measurements and predict the
target's trajectory before firing. Alongside this, an online SGDClassifier learns the
attacker's launch-direction pattern live via `partial_fit`, pre-slewing the turret
toward the predicted next sector while idle and re-adapting continuously as the pattern
drifts.
