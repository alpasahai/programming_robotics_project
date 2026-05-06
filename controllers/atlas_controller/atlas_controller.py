"""atlas_controller controller."""

from controller import Supervisor
import math

# create the Supervisor instance
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#----------------------COMPONENT CONNECTIONS----------------------
pan = robot.getDevice("PAN_MOTOR")
tilt = robot.getDevice("TILT_MOTOR")
projectile = robot.getFromDef("PROJECTILE")

#----------------------VARIABLES-----------------------------------

time = 0
#flying objects have 6 coordinates [x, y, z, rx, ry, rx]
projectile.setVelocity([0, 5, 5, 0, 0, 0]) #z controls the height


#----------------------MAIN LOOP ---------------------------------
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
    

