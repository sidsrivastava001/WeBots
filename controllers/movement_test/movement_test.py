"""movement_test controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math

# create the Robot instance.
robot = Robot()

posLeft = 0
posRight = 0
angle = 0
newangle = 0
pos = 0


left_motor = robot.getMotor("left wheel motor")
right_motor = robot.getMotor("right wheel motor")

left_encoder = left_motor.getPositionSensor()
right_encoder = right_motor.getPositionSensor()

left_heat_sensor = robot.getLightSensor("left_heat_sensor")
right_heat_sensor = robot.getLightSensor("right_heat_sensor")
    
gyro = robot.getGyro("gyro")
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

left_encoder.enable(timestep)
right_encoder.enable(timestep)
gyro.enable(timestep)
left_heat_sensor.enable(timestep)
right_heat_sensor.enable(timestep)

def update_sensors():
    global angle, newangle, posLeft, posRight
    #print("Gyro", gyro.getValues()[0])
    angle = angle+((timestep / 1000.0) * gyro.getValues()[0])
    newangle = angle * 180 / math.pi
    print("Angle", newangle)
    
    newangle = newangle-(360*math.floor(newangle/360))
    """
    if(newangle>180):
        newangle = newangle-360
        
    elif(newangle<-180):
        newangle = newangle+360
        """
    
    #print("New angle", newangle)
    posLeft = left_encoder.getValue()
    posRight = right_encoder.getValue()
    #print("Updated", posLeft, posRight)
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
def go_forward(x):
    global posLeft, posRight
    left_motor.setPosition(posLeft+x)
    right_motor.setPosition(posRight+x)
    print("Targets", left_motor.getTargetPosition(), right_motor.getTargetPosition())
    left_motor.setVelocity(2)
    right_motor.setVelocity(2)
    left = left_encoder.getValue()
    print("Starting, ", (left))
    while(robot.step(timestep) != -1 and abs(left-left_motor.getTargetPosition())>=0.005):
        update_sensors()
        #print("Going forward: ", abs(left-left_motor.getTargetPosition()))
        left = left_encoder.getValue()
    update_sensors()
    print("Done going forward")
    
def turn(deg):
    global posLeft, posRight
    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))
    kP = 0.05
    kI = 0.0003
    error = 0
    totalerror = 0
    i = 0
    while(robot.step(timestep) != -1 and (abs(error)>0.3 or i==0)):
        error = deg-newangle
        
        if(error > 180):
            error = 360 - error;
        elif(error < -180):
            error += 360;
        totalerror = totalerror+error;
        speed = error * kP + totalerror * kI;
        #print("Error", error)
        #print("Total error", totalerror)
        #print("Speed", speed)
        if(speed > 6.28):
            speed = 6.28
        if(speed<-6.28):
            speed = -6.28
        left_motor.setVelocity(-1*speed)
        right_motor.setVelocity(speed)
        update_sensors()
        i = i+1

        # Main loop:
# - perform simulation steps until Webots is stopping the controller
def goTile(dir):
    global pos
    if(dir == 'R'):

        pos =(-90+pos)%360
    
        if(pos>180):
            pos = pos-360
        if(pos<-180):
            pos = pos+360       
        print("Pos", pos)
    if(dir == 'L'):
        
        pos =(90+pos)%360
    
        if(pos>180):
            pos = pos-360
        if(pos<-180):
            pos = pos+360
        
        print("Pos", pos)
    turn(pos)
    go_forward(5.9)
     
#go_forward(5.85)
print(left_heat_sensor.getValue())
print(right_heat_sensor.getValue())
pos = 0
while robot.step(timestep) != -1:
    update_sensors()

    goTile('F')
    goTile('F')
    goTile('F')
    goTile('R')
    goTile('F')
    goTile('L')
    goTile('F')
    goTile('R')
    goTile('F')
    goTile('R')
    goTile('R')
    goTile('L')
    goTile('F')
    
    """
    go_forward(5.9)
    newpos =(-90+pos)%360
    if(newpos>=180):
        newpos = newpos-360
    if(newpos<=-180):
        newpos = newpos+360
    print("Pos", newpos)
    turn(pos, newpos)
    pos = newpos
    go_forward(5.9)
    newpos =(-90+pos)%360
    if(newpos>=180):
        newpos = newpos-360
    if(newpos<=-180):
        newpos = newpos+360
    print("Pos", newpos)
    turn(pos, newpos)
    pos = newpos
    go_forward(5.9)
    newpos =(180+pos)%360
    if(newpos>=180):
        newpos = newpos-360
    if(newpos<=-180):
        newpos = newpos+360
    print("Pos", newpos)
    turn(pos, newpos)
    pos = newpos
    go_forward(5.9)
    newpos =(90+pos)%360
    if(newpos>=180):
        newpos = newpos-360
    if(newpos<=-180):
        newpos = newpos+360
    print("Pos", newpos)
    turn(pos, newpos)
    pos = newpos
    go_forward(5.9)
    newpos =(90+pos)%360
    if(newpos>=180):
        newpos = newpos-360
    if(newpos<=-180):
        newpos = newpos+360
    print("Pos", newpos)
    turn(pos, newpos)
    pos = newpos
    go_forward(5.9)
    newpos =(90+pos)%360
    if(newpos>=180):
        newpos = newpos-360
    if(newpos<=-180):
        newpos = newpos+360
    print("Pos", newpos)
    turn(pos, newpos)
    pos = newpos
    go_forward(5.9)
    """
    """
    pos =(180+newangle)%360
    if(newangle>180):
        newangle = newangle-360
    print("Pos", pos)
    turn(pos)
    pos =(270+newangle)%360
    if(newangle>180):
        newangle = newangle-360
    print("Pos", pos)
    turn(pos)
    """
    #turn(90)

    #print("Gyro: ", gyro.getValues())
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
