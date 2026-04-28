from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_motor.setVelocity(3.0)
right_motor.setVelocity(3.0)

while robot.step(timestep) != -1:
    pass