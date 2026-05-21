# Autonomous Tracking and Laser Aiming System (ATLAS)

## Executive Summary:  
Autonomous Tracking and Laser Aiming System (ATLAS) is an autonomous robotic system that tracks parabolic projectiles within a bounded forward sector, using a stationary pan-tilt turret. ATLAS is designed to operate as a cue-driven fire-control radar sub-system and assumes the existence of a higher-level external search system providing coarse target cueing.  This data is fed as input to a pan-motor command (controlled using the turret’s azimuth rotation), combined with a tilt-motor command for the turret's elevation and a laser that is used to mark the interception point.  

This project demonstrates the robotics concepts of control systems, perception, and finite state machines (FSM) designs. To extend the behaviour of this project, ATLAS is built to incorporate adaptive AI-based tracking, which will be used to improve prediction accuracy under a range of environmental conditions.  

The system is designed using embedded intelligence principles and can be deployed on microcontrollers such as ESP32, while being prototyped in Webots, specifically using a 2-DOF pan-tilt turret model. This will help to simulate realistic sensors, motion, and physics that the system will encounter.  

## System Architecture:  
ATLAS consists of four main subsystems: 
  #### Perception Module (AIM → TRACK_PREDICT) (SENSE) 
  -> Processes the inputs from the tracking radar and external cues to track the projectile and produce estimates based on positioning and velocity. 
  ### Prediction Module (TRACK_PREDICT) (THINK)  
  -> Using data to model projectile trajectory and estimate future interception points. It will validate predictions based on ALTAS’s constraints. 
  #### Control Module (TRACK_PREDICT → ENGAGING) (ACT)  
  -> Converts predicted intercept coordinates into the pan and tilt motor commands. Reaching the `ENGAGING` state is itself the signal that the turret is ready to fire on the validated intercept. 
  #### FSM Control Module (RESET) (SAFETY)  
  -> Governs all the modules and enforces the multi-condition decision logic. It also prioritises fail-safe behaviour through the RESET mechanism. 

## Specialisation:  
To fulfil the Embedded Intelligence requirements, ATLAS implements the following: 
  Sensor Fusion: Integration of external cue data and tracking measurements to improve accuracy and robustness. 
  Context-Aware Behaviour: Confident decision making, validity of predictions and environmental constraints. 
  Real-time Logic: Implementing a continuous closed-loop operation to provide immediate response to dynamic inputs and fail-safe conditions. 

 

## Safety, Robustness and Reliability:  
#### Safety: 
To ensure the safety and reliability of the system, if the motors begin operating above a certain movement frequency, the system will enter a reset state. This state resets the system and attempts to resume tracking the target, if one was being tracked. 
#### Robustness:  
For secure robustness, the edge cases we plan on testing are multiple objects (solutions include prioritisation since ATLAS focuses on single active targets), sudden lighting changes affecting the visual and object disappearance.  
#### Reliability:  
To ensure that the system has a fail-safe, the ATLAS will be automatically reset (state changes from RESET -> IDLE) when the object being tracked is lost/no longer in range. To better cater for this, we will explore filtering sensor noise.  

## Advanced Component (10%):  
ATLAS utilises an AI-based adaptive pattern detection capability (frequency, direction, size) to predict future attack pattern times and location. This means using previous data to build attack pattern models.  Assuming attack patterns exist, the model will be adaptive and improve over time. 
