"""movement_test controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math

# create the Robot instance.
robot = Robot()

left_motor = robot.getMotor("left wheel motor")
right_motor = robot.getMotor("right wheel motor")

left_encoder = left_motor.getPositionSensor()
right_encoder = right_motor.getPositionSensor()
    
gyro = robot.getGyro("gyro")
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

left_encoder.enable(timestep)
right_encoder.enable(timestep)
gyro.enable(timestep)


# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
def go_forward(x):
    left_motor.setPosition(x)
    right_motor.setPosition(x)
    left_motor.setVelocity(2)
    right_motor.setVelocity(2)
    while(robot.step(timestep) != -1 and abs(left_encoder.getValue() - left_motor.getTargetPosition())>=0.01):
        print("Going forward")
def turn(deg):
    angle = 0
    newangle = 0
    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))
    if(deg<0):
        left_motor.setVelocity(-2)
        right_motor.setVelocity(2)
    else:
        left_motor.setVelocity(2)
        right_motor.setVelocity(-2)
    newangle = 0
    while(robot.step(timestep) != -1 and newangle<abs(deg)):
        print("Gyro: ", gyro.getValues()[0])
        angle = angle+abs(((timestep / 1000.0) * gyro.getValues()[0]))
        newangle = angle * 180 / math.pi;
        print("Turning pos at", newangle)
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
        # Main loop:
# - perform simulation steps until Webots is stopping the controller
go_forward(5)
turn(90)
turn(-90)
print("Step", timestep)

while robot.step(timestep) != -1:
    print("Gyro: ", gyro.getValues())
    #print(left_encoder.getValue())
    #print(right_encoder.getValue())
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
