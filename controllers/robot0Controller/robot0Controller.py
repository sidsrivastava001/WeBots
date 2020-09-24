"""robot0Controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from nav import Nav
from HSUDetector import *
from HSURotateDetection import *
from visualVictim import *
import math
import struct
import time
import cv2
import numpy as np
import pytesseract

def clearFile():
        print("Clearing file!")
        filePtr = open('wall.txt', 'w+')
        filePtr.truncate(0)
        filePtr.write('20 20 V\n')
        filePtr.close()

clearFile()
# Initialize the AI Navigator
AI = Nav()
#Visual = visual()

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
posY = 0
posZ = 0
optimalFrontDistance = 0.04 # For front callibration
blackThresh = 70
letters = {"Left": "None", "Center" : "None", "Right" : "None"}
imgarr = [[0, 0, 0]]
leftAtCapture = 0
rightAtCapture = 0

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
cam_right = robot.getCamera("camera_right")
cam_left = robot.getCamera("camera_left")



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
cam.enable(timestep)
cam_left.enable(timestep)
cam_right.enable(timestep)

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
    global angle, newangle, posLeft, posRight, frontl, frontr, left, right, backl, backr, prevAngle, leftheat, rightheat, posX, posY, posZ, colorval
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
    posX = gps.getValues()[0]
    posY = gps.getValues()[1]
    posZ = gps.getValues()[2]
    #print("Color", colorval[0], colorval[1], colorval[2])

def getLetters():
    letters = ["None", "None", "None"]

    # imgLeft = cam_left.getImage()
    # imgRight = cam_right.getImage()

    custom_config = r'--oem 3 --psm 10'
    u = 0
    for imgList in imgarr:
        for camNum in range(0,3):
            img = np.array(np.frombuffer(imgList[camNum], np.uint8).reshape((cam.getHeight(), cam.getWidth(), 4)))
            img[:,:,2] = np.zeros([img.shape[0], img.shape[1]])
            #cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            thresh = cv2.threshold(gray, 140, 255, cv2.THRESH_BINARY)[1]
            # draw all contours in green and accepted ones in red
            contours, h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            #cv2.imwrite("unthresholded.png", gray)
            cv2.drawContours(img, contours, -1, (0, 255, 0), 1)
            if camNum == 0:
                cv2.imwrite(str(u)+" left.png", img)
            if camNum == 1:
                cv2.imwrite(str(u)+" center.png", img)
            if camNum == 2:
                cv2.imwrite(str(u)+" right.png", img)
            #cv2.drawContours(gray, contours, -1, (150, 155, 155),1)
            #cv2.imwrite("thresholded.png", gray)
            for i, c in enumerate(contours):
                rect = cv2.boundingRect(c)
                x,y,w,h = rect
                box = cv2.rectangle(img, (x,y), (x+w,y+h), (0,0,255), 2)
                cropped = img[y: y+h, x: x+w]
                text = pytesseract.image_to_string(cropped, config=custom_config)
                print("TEXT", text)
                if("S" in text or "s" in text):
                    print("Found S")
                    letters[camNum] = "S"
                    break
                elif("H" in text):
                    print("Found H")
                    letters[camNum] = "H"
                    break
                elif("U" in text):
                    print("Found U")
                    letters[camNum] = "U"
                    break
        print(str(u) + ":", letters)
        filePtr = open('letters.txt', 'a+')
        #filePtr.truncate(0)
        filePtr.write(str(u) + ": {" + "left: " + letters[0] +', center: ' + letters[1] + ', right: ' + letters[2] + "}\n")
        filePtr.close()
        u+=1
    filePtr = open('letters.txt', 'a+')
    filePtr.write("\n")
    filePtr.close()
    #print("Shape:",img.shape())
    #print(img)
    #letterCenter = Visual.getLetter(gray)
    # letterLeft = Visual.getLetter(imgLeft)
    # letterRight = Visual.getLetter(imgRight)
    return {"Left": letters[0], "Center" : letters[1], "Right" : letters[2]}

def clearVictims():
    global letters
    letters["Left"] = "None"
    letters["Right"] = "None"
    letters["Center"] = "None"

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Robot stops for three seconds
def stop():
    start = robot.getTime()
    while(robot.step(timestep) != -1 and (robot.getTime()-start)<4):
        update_sensors()
    update_sensors()
     # Sleep for 3 seconds

def go_forward(x):
    global posLeft, posRight
    """
    imgarr[0][1] = cam.getImage()
    imgarr[0][0] = cam_left.getImage()
    imgarr[0][2] = cam_right.getImage()
    print("FIRST SAVED IMAGE")
    """
    left_motor.setPosition(posLeft+x)
    right_motor.setPosition(posRight+x)
    left_motor.setVelocity(3.0)
    right_motor.setVelocity(3.0)
    left = left_encoder.getValue()
    # print("Starting, ", (left))
    stops = [False, False]
    right = right_encoder.getValue()    
    while(robot.step(timestep) != -1 and abs(left-left_motor.getTargetPosition())>=0.005 and abs(right-right_motor.getTargetPosition())>=0.005):
        update_sensors()
        right_motor.setVelocity(left_motor.getVelocity())
        #print("Binary colorval:", colorval)
        """
        if(left > left_motor.getTargetPosition()/3 and stops[0] == False):
            imgarr[1][1] = cam.getImage()
            imgarr[1][0] = cam_left.getImage()
            imgarr[1][2] = cam_right.getImage()
            stops[0] = True
            print("SECOND SAVED IMAGE")
        if(left > left_motor.getTargetPosition()*(2/3) and stops[1] == False):
            imgarr[2][1] = cam.getImage()
            imgarr[2][0] = cam_left.getImage()
            imgarr[2][2] = cam_right.getImage()
            stops[1] = True
            print("THIRD SAVED IMAGE")
        """
        #img = np.array(np.frombuffer(colorval, np.uint8).reshape((color.getHeight(), color.getWidth(), 4)))
        #img[:,:,2] = np.zeros([img.shape[0], img.shape[1]])
        #hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)[int(color.getHeight()/2)][int(color.getWidth()/2)]
        #hsv = cv2.cvtColor( cv2.cvtColor(img, cv2.COLOR_RGB2HSV),  cv2.COLOR_HSV2RGB)[0][0]
        #print("HSV : ", hsv)
        #print("RGB colorval : ", colorval)
        #print("Max velocity", left_motor.getMaxVelocity(), right_motor.getMaxVelocity())
        if(left_motor.getMaxVelocity()<6.28 and right_motor.getMaxVelocity()<6.28): # or color == swamp_colour
            #print("SAW Swamp!")
            left_motor.setVelocity(2.0)
            right_motor.setVelocity(2.0)

        if(colorval[0]<blackThresh and colorval[1]<blackThresh and colorval[2]<blackThresh): # or color == swamp_colour
            print("SAW HOLE!")
            print("Blacking out")
            AI.blackout()
            go_backwards(abs(left-left_motor.getTargetPosition())-x)
            return False
        # if(letterCenter == "None"):
        #    letterCenter = Visual.getLetter()
        #    print("Letter Saw:", letterCenter)
        
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
    clearVictims()
    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))
    kP = 0.03
    kI = 0.0003
    error = 0
    totalerror = 0
    i = 0
    while(robot.step(timestep) != -1 and (abs(error)>0.4 or i==0)):
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
    if(frontl<=0.1 and frontr<=0.1):
        left_motor.setPosition(float("inf"))
        right_motor.setPosition(float("inf"))
        calcfront = 0.95510271724 * frontl
        while(robot.step(timestep) != -1 and calcfront>optimalFrontDistance):
            update_sensors()
            left_motor.setVelocity(1.0)
            right_motor.setVelocity(1.0)
            print("Calc", calcfront)
            calcfront = 0.95510271724 * frontl
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        update_sensors()
        while(robot.step(timestep) != -1 and calcfront<optimalFrontDistance):
            update_sensors()
            left_motor.setVelocity(-1.0)
            right_motor.setVelocity(-1.0)
            print("Calc", calcfront)
            calcfront = 0.95510271724 * frontl
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        update_sensors()
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
    
    print("Go Forward")
    x = go_forward(2.0)
    
    if(not x):
        print("SAW Hole")
        return False
    
    else:
        print("No Hole")
        #return True
    
    imgarr[0][1] = cam.getImage()
    imgarr[0][0] = cam_left.getImage()
    imgarr[0][2] = cam_right.getImage()
    x = go_forward(4.0)
    update_sensors()
    leftAtCapture = left
    rightAtCapture = right
    
    if(not x):
        print("SAW Hole")
        return False
    
    else:
        print("No Hole")
        return True
        #go_forward(3.25)



#go_forward(5.85)
#print(left_heat_sensor.getValue())
#print(right_heat_sensor.getValue())
pos = 0

# Task main()
i = 0
while robot.step(timestep) != -1:
    update_sensors()
    if(i != 0):
        letters = getLetters()
        print("LETTERS", letters)
    i+=1
    print("Current position: X:", posX, "Y:", posY, "Z:", posZ)
    print("Left:", left)
    print("Right:", right)
    print("Front:", (frontl+frontr)/2)
    print("Back:", (backl+backr)/2)
    if(frontl <= 0.12 and frontr<=0.12):
        print("Wall in front")
        if letters["Center"] != "None":
            print("Reporting Victim Center!")
            sendMessage(victimType=letters["Center"])
            stop()      
            clearVictims()
        AI.markWall(AI.direction)
    if(right <= 0.14):
        print("Wall to right")
        if letters["Right"] != "None":
            print("Reporting Victim Right!")
            sendMessage(victimType=letters["Right"])
            stop()
            clearVictims()
        AI.markWall((1 + AI.direction) % 4)
    if(backl <= 0.12 and backr <= 0.12):
        print("Wall to back")
        AI.markWall((2 + AI.direction) % 4)
    if(left <= 0.14):
        print("Wall to left")
        if letters["Left"] != "None":
            print("Reporting Victim Left!")
            sendMessage(victimType=letters["Left"])
            stop()   
            clearVictims()
        AI.markWall((3 + AI.direction) % 4)
    clearVictims()
    #IMAGE STUFF:
    '''imgCenter = cam.getImage()
    imgLeft = cam_left.getImage()
    imgRight = cam_right.getImage()
    letterCenter = Visual.getLetter(imgCenter)'''
    #cam.saveImage("visionCenter.png", 100)
    #cam_left.saveImage("visionLeft.png", 100)
    #cam_right.saveImage("visionRight.png", 100)

    '''img = cv2.imread("visionCenter.png")
    testThreshold(img)    '''
    
    # letterRight = Visual.getLetter()
    # letterLeft = Visual.getLetter()
        
    
    successful = False
    while not successful:
        commands = AI.calculate() # Get commands
        if len(commands)==0:
            print('FINISHED MAZE!')
            sendEndGame()
            exit(0)
        if(len(commands)>1): # Backtracking (BFS)
            clearVictims()
            for i in range(len(commands)):
                if(i == len(commands)-1): # Last command
                    successful = goTileWithVictim(commands[i])
                else:
                    successful = goTile(commands[i])
                update_sensors()
        else:
            successful = goTileWithVictim(commands[0])


    # Only write to file once commands are successfully executed
    AI.flush() # Actually write data to file
    pass

# Enter here exit cleanup code.
