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
Autonomous Tracking and Laser Aiming System (ATLAS) is a complete fire-control system
capable of fully autonomous target identification, trajectory prediction and engagement.
ATLAS consists of two units: the fire-control radar (FCR) and the search radar. These
units operate as collaborative subsystems that communicate cueing information over
emitters and receivers. The system was designed to operate in a simulated outdoor
environment within Webots.

The role of the Search Radar is to continuously scan a wide search area and provide 3D
radar cues to the FCR. The FCR consists of a turret-mounted narrow-beam radar and a
projectile launch system used to engage incoming threats. Rotational motor actuators are
used to control both the Search Radar sweep motion and the pan/tilt movement of the FCR
turret, enabling the system to autonomously scan, track, and engage airborne targets.
When a target is identified by the Search Radar, the FCR attempts to lock onto the
target. Once the target has been locked, sensor data from both radar systems is fused
using a Kalman filter to reduce measurement noise and estimate the projectile’s future
location. Once the error between the estimation and true location has been minimised,
the projectile launch system attempts to neutralise the threat.

ATLAS uses online supervised learning---an SGDClassifier updated one launch at a time
via `partial_fit`---to learn the attacker's launch-direction pattern live and pre-slew
the turret toward the predicted next sector while idle, re-adapting continuously as the
pattern drifts.
