= Safety and Robustness
// TODO: Explain safety mechanisms used, for example:
// ● Collision avoidance
// ● Boundary limits
// ● Emergency stop
// ● Fail-safe behaviour
// ● Timeout protection
// Explain what happens if sensors fail or unexpected situations occur.

ATLAS implements multiple safety and robustness mechanisms to ensure stable operation
during uncertain conditions. The system uses confidence-gated engagement logic to
prevent firing at the target when there is insufficient tracking or prediction
confidence. It is ensured that engagement only occurs when the target has remained
stable for multiple consecutive tracking updates.


We have implemented a protected firing sector restriction to prevent the turret from
engaging with targets that fall within the restricted azimuth regions surrounding the
Search Radar. While turret may continue to track target through full rotational
movement, the firing commands are disabled within the restricted areas to prevent unsafe
behaviour. By having sensor confidence, ATLAS showcases its safety and robustness by
only engaging with targets when there is enough data to prove that the decision is
confident. The target confidence must exceed a certain threshold, and prediction error
must be low with the target remaining stable in order for firing to occur. The automatic
RESET behaviour is triggered when the target is out of range; this means the sensor data
has become stale, and invalid measurements can occur. The RESET state lets the systems
return to a known safe state if abnormal behaviour were to occur. The Kalman filter
supports the robustness of the system by reducing the noise so the sensor measurements
and unstable target detections.


Additional mechanisms including the bullet-lifetime timeout, single-bullet interlocks,
stable-cue clearing and target timeout handling further improve the safety of the
system. These mechanisms unstable firing and overlapping engagements during unexpected
simulation conditions.
