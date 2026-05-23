= System Architecture
// TODO: Explain the system structure using the Sense → Process → Decide → Act model.
// Explain briefly how information flows through the system

ATLAs follows the Sense -> Process -> Decide -> Act architecture. As the information
flows through the multiple modular subsystems that are responsible for perception,
prediction, decision-making and physical actuation, the system acts accordingly. The
architecture separates the behavioral logic, sensing logic, hardware control, and
prediction into independent modules to improve robustness, system reliability, and
maintainability.


The Sense layer focuses on the Search Radar and Fire Control Radar (FCR) subsystems. The
Search Radar preforms wide-area scanning and broadcasts the coarse target cues to the
FCR which performs the narrow-beam precision tracking once the projectile enters the
turret’s engagement region. The Process layer consists of the Kalman Filter and the
Ballistic Trajectory Predictor. These help to estimate the projectile’s position,
velocity, and future interception points while reducing the sensor noise and improving
stability.


The Decide layer is mainly the FSM logic. This is where the evaluation of the projectile
trajectory confidence occurs along with the validity of prediction, engagement
constraints, and the system’s safety and exit conditions before determining the next
behavioral state. Finally, the Act layer controls the physical behaviour of the turret
(pan and tilt motor) and the bullet firing system. Once the engagement conditions have
been met, the turret autonomously rotates and aligns with the predicted interception
point and launches the bullet toward the target.
