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
projectile_launched = False
waiting_for_launch = False #THIS IS TO HELP IT ADD A GAP like a waiting thing
reset_time = 0
launch_delay = 2000 #2 secs

projectile_system.launch_projectile()
projectile_launched = True

#turret variables to follow the projectile:
turret_position = robot.getSelf().getPosition()

#----------------------MAIN LOOP ---------------------------------
while robot.step(timestep) != -1:
    #TURRET SYSTEM (TESTING) - trying to make it rotate back and forward
    #pan_angle = math.sin(time) * 1.57
    #tilt_angle = math.sin(time) * 0.5
    
    #pan.setPosition(pan_angle)
    #tilt.setPosition(tilt_angle)
    #time += 0.02
    
    #Confirmation of the testing
    #print("Robot PAN MOTOR Rotation: ", pan_angle)
    #print("Robot TILT MOTOR Rotation: ", tilt_angle)
  #-----------------------------PROJECTILE SYSTEM------------------------- 
    #Getting the projectile's positioning and velocity
    projectile_position = projectile.getPosition()
    projectile_velocity = projectile.getVelocity()
    
    #Getting the projectile launching and resetting:
    current_time = robot.getTime() * 1000 #converting to ms
    #Detecting when the projectile hits the ground:
    if projectile_position[2] < 0.05 and projectile_velocity[2] < 0 and projectile_launched:
    #if projectile_position[1] < 0.05 and abs(projectile_velocity[2] < 0) and projectile_launched:        
        projectile_launched = False
        waiting_for_launch = True
        reset_time = current_time
        projectile_system.reset_projectile()
        
    #Relaunching after the delay
    if waiting_for_launch and (current_time - reset_time > launch_delay):
        projectile_system.launch_projectile()
        
        projectile_launched = True
        waiting_for_launch = False
    
    #Working through the radar system:
    target_position = radar.get_target_position()
    print("Projectile Position: ", target_position)
  #-----------------------------TURRET SYSTEM-------------------------   
    dx = target_position[0] - turret_position[0]
    dy = target_position[1] - turret_position[1]
    dz = target_position[2] - turret_position[2]
    
    #Calculating horizontal distance of the projectile from turret POV
    horizontal_distance = math.sqrt(dx**2 + dz**2)
    
    #Calculating pan angle and tilt angle:
    pan_angle = math.atan2(dx, dz)
    tilt_angle = math.atan2(dy, horizontal_distance)
    
    #Setting the motors so that pan adn tilt follows the porjectile:
    pan.setPosition(pan_angle)
    tilt.setPosition(tilt_angle)
    print("Robot PAN MOTOR Rotation: ", pan_angle)
    print("Robot TILT MOTOR Rotation: ", tilt_angle)
    
    pass
    