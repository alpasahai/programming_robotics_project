from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

print("SEARCH RADAR CONTROLLER STARTED")

motor = robot.getDevice("SEARCH_RADAR_MOTOR")

if motor is None:
    print("ERROR: SEARCH_RADAR_MOTOR not found")
    while robot.step(timestep) != -1:
        pass

print("SEARCH_RADAR_MOTOR found")
motor.setPosition(float("inf"))
motor.setVelocity(2.0)

while robot.step(timestep) != -1:
    pass
