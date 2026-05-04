from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

SPEED = 6.28

left_motor.setVelocity(SPEED)
right_motor.setVelocity(SPEED)

while robot.step(timestep) != -1:
    pass
