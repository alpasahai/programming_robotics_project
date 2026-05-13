"""atlas_controller controller."""

#Importing libraries:
import math

from controller import Supervisor

#Implementing the multi-file system:
from radar_system import RadarSystem
from projectile_system import ProjectileSystem

#Create the Supervisor instance
robot = Supervisor()
#Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#----------------------COMPONENT CONNECTIONS----------------------
pan = robot.getDevice("PAN_MOTOR")
tilt = robot.getDevice("TILT_MOTOR")
projectile = robot.getFromDef("PROJECTILE")
#----------------------VARIABLES-----------------------------------

time = 0
#flying objects have 6 coordinates [x, y, z, rx, ry, rx]
#projectile.setVelocity([0, 5, 5, 0, 0, 0]) #z controls the height

radar = RadarSystem(projectile)
projectile_system = ProjectileSystem(projectile)

#Ensuring that it intially launches:
projectile_system.launch_projectile()
resetting = False #This is to ensure that projectile properly relaunches

#----------------------MAIN LOOP ---------------------------------
while robot.step(timestep) != -1:
    #TURRET SYSTEM (TEMP) - trying to make it rotate back and forward
    pan_angle = math.sin(time) * 1.57
    tilt_angle = math.sin(time) * 0.5
    
    pan.setPosition(pan_angle)
    tilt.setPosition(tilt_angle)
    
    time += 0.02
    
    #Confirmation of the testing
    print("Robot PAN MOTOR Rotation: ", pan_angle)
    print("Robot TILT MOTOR Rotation: ", tilt_angle)
   
    #Getting the projectile launching and resetting:
    projectile_position = projectile.getPosition()
    
    if projectile_position[1] < 0.05 and not resetting:
        projectile_system.reset_projectile()
        resetting = True
    elif resetting:
        projectile_system.launch_projectile()
        resetting = False
    
    
    #Working through the radar system:
    target_position = radar.get_target_position()
    print("Projectile Position: ", target_position)

    pass
    