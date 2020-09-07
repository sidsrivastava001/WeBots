"""movement_test controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from nav import Nav
import math

# Initialize the AI Navigator
AI = Nav()

# create the Robot instance.
robot = Robot()

posLeft = 0
posRight = 0
angle = 0
newangle = 0
prevAngle = 0
pos = 0
frontl = 0
frontr = 0
left = 0
right = 0
backl = 0
backr = 0
leftheat = 0
rightheat = 0

left_motor = robot.getMotor("left wheel motor")
right_motor = robot.getMotor("right wheel motor")

left_encoder = left_motor.getPositionSensor()
right_encoder = right_motor.getPositionSensor()

left_heat_sensor = robot.getLightSensor("left_heat_sensor")
right_heat_sensor = robot.getLightSensor("right_heat_sensor")
    
gyro = robot.getGyro("gyro")
emitter = robot.getEmitter("emitter")

front_sensor_l = robot.getDistanceSensor("ps7")
front_sensor_r = robot.getDistanceSensor("ps0")

left_sensor = robot.getDistanceSensor("ps5")
right_sensor = robot.getDistanceSensor("ps2")

back_sensor_l = robot.getDistanceSensor("ps3")
back_sensor_r = robot.getDistanceSensor("ps4")
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

left_encoder.enable(timestep)
right_encoder.enable(timestep)
gyro.enable(timestep)
left_heat_sensor.enable(timestep)
right_heat_sensor.enable(timestep)
front_sensor_l.enable(timestep)
front_sensor_r.enable(timestep)
left_sensor.enable(timestep)
right_sensor.enable(timestep)
back_sensor_l.enable(timestep)
back_sensor_r.enable(timestep)

def update_sensors():
    global angle, newangle, posLeft, posRight, frontl, frontr, left, right, backl, backr, prevAngle, leftheat, rightheat
    #print("Gyro", gyro.getValues()[0])
    curr = gyro.getValues()[0]
    angle = angle+((timestep / 1000.0) * (curr+prevAngle)/2)
    prevAngle = curr
    newangle = angle * 180 / math.pi
    #print("Angle", newangle)
    
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
    frontl = front_sensor_l.getValue()
    frontr = front_sensor_r.getValue()
    left = left_sensor.getValue()
    right = right_sensor.getValue()
    backl = back_sensor_l.getValue()
    backr = back_sensor_r.getValue()
    leftheat = left_heat_sensor.getValue()
    rightheat = right_heat_sensor.getValue()
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
            error = 360 - error
        elif(error < -180):
            error += 360
        totalerror = totalerror+error
        speed = error * kP + totalerror * kI
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
    if(dir == AI.convertCommand(1)):

        pos =(-90+pos)%360
    
        if(pos>180):
            pos = pos-360
        if(pos<-180):
            pos = pos+360       
        print("Pos", pos)
    if(dir == AI.convertCommand(3)):
        
        pos =(90+pos)%360
    
        if(pos>180):
            pos = pos-360
        if(pos<-180):
            pos = pos+360
        
        print("Pos", pos)
    if(dir == AI.convertCommand(2)):
        
        pos =(180+pos)%360
    
        if(pos>180):
            pos = pos-360
        if(pos<-180):
            pos = pos+360
        
        print("Pos", pos)
    turn(pos)
    go_forward(5.9)
    
def goTileWithVictim(dir):
    global pos
    victim = 0
    if(leftheat>30 or rightheat>30):
        print("SEE VICTIM")
        victim = 1
    if(dir == AI.convertCommand(1)):

        pos =(-90+pos)%360
    
        if(pos>180):
            pos = pos-360
        if(pos<-180):
            pos = pos+360       
        print("Pos", pos)
    if(dir == AI.convertCommand(3)):
        
        pos =(90+pos)%360
    
        if(pos>180):
            pos = pos-360
        if(pos<-180):
            pos = pos+360
        
        print("Pos", pos)
    if(dir == AI.convertCommand(2)):
        
        pos =(180+pos)%360
    
        if(pos>180):
            pos = pos-360
        if(pos<-180):
            pos = pos+360
        
        print("Pos", pos)
    turn(pos)
    if((leftheat>30 or rightheat>30) and victim != 1):
        print("SEE VICTIM")
    go_forward(5.9)

     
#go_forward(5.85)
print(left_heat_sensor.getValue())
print(right_heat_sensor.getValue())
pos = 0

# Task main()
while robot.step(timestep) != -1:
    update_sensors()
    if(frontl <= 0.1 and frontr<=0.1):
        print("Wall in front")
        AI.markWall(AI.direction)
    if(right <= 0.1):
        print("Wall to right")
        AI.markWall((1 + AI.direction) % 4)
    if(backl <= 0.1 and backr <= 0.1):
        print("Wall to back")
        AI.markWall((2 + AI.direction) % 4)
    if(left <= 0.1):
        print("Wall to left")
        AI.markWall((3 + AI.direction) % 4)
   
    commands = AI.calculate() # Get commands
    if(len(commands)>1):
        for command in commands:
           goTile(command)
           update_sensors()
    else:
        goTileWithVictim(commands[0])


    # Only write to file once commands are successfully executed
    AI.flush() # Actually write data to file

    """
    if(frontl > 0.1 and frontr>0.1):
        goTile('F')
    elif(right > 0.1):
        goTile('R')
    elif(left > 0.1):
        goTile('L')
    else:
        goTile('B')
    """
    
    """
    goTile('F')
    goTile('F')
    goTile('F')
    print(frontl, frontr, left, right)
    
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
