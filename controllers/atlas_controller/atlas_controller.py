"""atlas_controller controller."""

from controller import Robot
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#Getting the motor's device
pan = robot.getDevice("PAN_MOTOR")
tilt = robot.getDevice("TILT_MOTOR")

time = 0

while robot.step(timestep) != -1:
#trying to make it rotate back and forward
    pan_angle = math.sin(time) * 1.57
    tilt_angle = math.sin(time) * 0.5
    
    pan.setPosition(pan_angle)
    tilt.setPosition(tilt_angle)
    
    time += 0.02
    
    #Confirmation of the testing
    print("Robot PAN MOTOR Rotation: ", pan_angle)
    print("Robot TILT MOTOR Rotation: ", tilt_angle)

    pass
    

