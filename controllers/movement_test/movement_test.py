"""movement_test controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from nav import Nav
import math
import struct
import time
import cv2
import numpy as np

def clearFile():
        print("Clearing file!")
        filePtr = open('wall.txt', 'w+')
        filePtr.truncate(0)
        filePtr.write('20 20 V\n')
        filePtr.close()

clearFile()
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
colorval = ""
posX = 0
posZ = 0
optimalFrontDistance = 0.04 # For front callibration

hole_colour = b'\x1e\x1e\x1e\xff'
hole_colour2 = b'\n\n\n\xff'
swamp_colour = b'R\x89\xa7\xff'
swamp_colour2 = b'R\x89\xa6\xff'

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

color = robot.getCamera("colour_sensor")

cam = robot.getCamera("camera_centre")



# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())


gps = robot.getGPS("gps")
gps.enable(timestep)
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
color.enable(timestep)


#Emitter Stuff:
emitter = robot.getEmitter("emitter")

#Sending Message for Emitter

#robot type, position x cm, position z cm, victim type
# The victim type is hardcoded as "H", but this should be changed to different victims for your program
# Harmed = "H"
# Stable = "S"  
# Unharmed = "U"
# Heated (Temperature) = "T"
def sendMessage(victimType='T'):
    position = gps.getValues()
    #robot type, position x cm, position z cm, victim type
    if victimType=='T':
        print("Sending message")
        message = struct.pack('i i c', int(position[0] * 100), int(position[2] * 100), b"T")
    elif victimType=="H":
        print("Sending message")
        message = struct.pack('i i c', int(position[0] * 100), int(position[2] * 100), b"H")
    elif victimType=="S":
        print("Sending message")
        message = struct.pack('i i c', int(position[0] * 100), int(position[2] * 100), b"S")
    elif victimType=="U":
        print("Sending message")
        message = struct.pack('i i c', int(position[0] * 100), int(position[1] * 100), b"U")
    print(struct.unpack('i i c', message))
    emitter.send(message)

def sendEndGame():
    message = struct.pack('i i c', 0, 0, b'E')
    emitter.send(message)

def update_sensors():
    global angle, newangle, posLeft, posRight, frontl, frontr, left, right, backl, backr, prevAngle, leftheat, rightheat, posX, posZ, colorval
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
    colorval = color.getImage()
    posX = gps.getValues()[1]
    posZ = gps.getValues()[2]

    #print("Updated", posX, posZ)
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Robot stops for three seconds
def stop():
    start = robot.getTime()
    while(robot.step(timestep) != -1 and (robot.getTime()-start)<3):
        update_sensors()
    update_sensors()
     # Sleep for 3 seconds

def go_forward(x):
    global posLeft, posRight
    left_motor.setPosition(posLeft+x)
    right_motor.setPosition(posRight+x)
    left_motor.setVelocity(4.0)
    right_motor.setVelocity(4.0)
    left = left_encoder.getValue()
    # print("Starting, ", (left))

    right = right_encoder.getValue()    
    while(robot.step(timestep) != -1 and abs(left-left_motor.getTargetPosition())>=0.005 and abs(right-right_motor.getTargetPosition())>=0.005):
        update_sensors()
        right_motor.setVelocity(left_motor.getVelocity())
        #print("Binary colorval:", colorval)
        img = np.array(np.frombuffer(colorval, np.uint8).reshape((color.getHeight(), color.getWidth(), 4)))
        img[:,:,2] = np.zeros([img.shape[0], img.shape[1]])
        #hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)[int(color.getHeight()/2)][int(color.getWidth()/2)]
        #hsv = cv2.cvtColor( cv2.cvtColor(img, cv2.COLOR_RGB2HSV),  cv2.COLOR_HSV2RGB)[0][0]
        #print("HSV : ", hsv)
        #print("RGB colorval : ", colorval)
        #print("Max velocity", left_motor.getMaxVelocity(), right_motor.getMaxVelocity())
        if(left_motor.getMaxVelocity()<6.28 and right_motor.getMaxVelocity()<6.28): # or color == swamp_colour
            print("SAW Swamp!")
            left_motor.setVelocity(2.0)
            right_motor.setVelocity(2.0)

        if(colorval == hole_colour or colorval == hole_colour2): # or color == swamp_colour
            print("SAW HOLE!")
            print("Blacking out")
            AI.blackout()
            go_backwards(abs(left-left_motor.getTargetPosition())-x)
            return False
        
        #print("Going forward: ", left_motor.getVelocity(), right_motor.getVelocity())
        #print("Going forward: ", left_motor.getVelocity(), right_motor.getVelocity())
        left = left_encoder.getValue()
        right = right_encoder.getValue()
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    update_sensors()
    return True
    # print("Done going forward")
    
def go_backwards(x):
    global posLeft, posRight
    left_motor.setPosition(posLeft+x)
    right_motor.setPosition(posRight+x)
    left_motor.setVelocity(-2)
    right_motor.setVelocity(-2)
    left = left_encoder.getValue()
    # print("Starting, ", (left))
    right = right_encoder.getValue()    
    while(robot.step(timestep) != -1 and abs(left-left_motor.getTargetPosition())>=0.005 and abs(right-right_motor.getTargetPosition())>=0.005):
        update_sensors()
        right_motor.setVelocity(left_motor.getVelocity())
        
        #print("Going forward: ", left_motor.getVelocity(), right_motor.getVelocity())
        #print("Going forward: ", left_motor.getVelocity(), right_motor.getVelocity())
        left = left_encoder.getValue()
        right = right_encoder.getValue()
        
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    update_sensors()
    
    print("Done going backwards")
    
def turn(deg):
    global posLeft, posRight, newangle
    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))
    kP = 0.03
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
    update_sensors()
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
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
    x = go_forward(6.0)
    
    if(not x):
        print("SAW Hole")
        return False
    else:
        print("No Hole")
        #go_forward(3.25)
        return True
    
def goTileWithVictim(dir):
    global pos
    victim = 0
    if(leftheat>30 or rightheat>30):
        print("SEE VICTIM")
        sendMessage('T')
        stop()
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
    if(frontl<=0.1 and frontr<=0.1):
        left_motor.setPosition(float("inf"))
        right_motor.setPosition(float("inf"))
        calcfront = 0.95510271724 * frontl
        while(robot.step(timestep) != -1 and calcfront>optimalFrontDistance):
            update_sensors()
            left_motor.setVelocity(0.5)
            right_motor.setVelocity(0.5)
            print("Calc", calcfront)
            calcfront = 0.95510271724 * frontl
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        update_sensors()
        while(robot.step(timestep) != -1 and calcfront<optimalFrontDistance):
            update_sensors()
            left_motor.setVelocity(-0.5)
            right_motor.setVelocity(-0.5)
            print("Calc", calcfront)
            calcfront = 0.95510271724 * frontl
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        update_sensors()
    turn(pos)

    # HEAT VICTIM DETECTION
    if((leftheat>30 or rightheat>30) and victim != 1):
        print("SEE VICTIM")
        sendMessage('T')
        stop()
    
    x = go_forward(6.0)
    
    if(not x):
        print("SAW Hole")
        return False
    
    else:
        print("No Hole")
        #go_forward(3.25)
        return True


#go_forward(5.85)
#print(left_heat_sensor.getValue())
#print(right_heat_sensor.getValue())
pos = 0

# Task main()
while robot.step(timestep) != -1:
    update_sensors()
    print("Left", left)
    print("Right", right)
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

    successful = False
    while not successful:
        commands = AI.calculate() # Get commands
        if len(commands)==0:
            print('FINISHED MAZE!')
            sendEndGame()
            exit(0)
        if(len(commands)>1):
            for command in commands:
                successful = goTile(command)
                update_sensors()
        else:
            successful = goTileWithVictim(commands[0])


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
