from controller import Robot
import math
import struct

class VR:
    def __init__(self):
        self.position = [0,0,0]
        self.victimProximity = 0.06

    def updatePosition(self, position):
        self.position = position

    def getObjectDistance(self):
        return math.sqrt((self.position[0] ** 2) + (self.position[2] ** 2))

    def nearObject(self):
        return self.getObjectDistance(position)  < 0.06

    def getVisibleVictims():
        #get all objects the camera can see
        objects = camera.getRecognitionObjects()

        victims = []

    for item in objects:
        if item.get_colors() == [1,1,1]:
            victim_pos = item.get_position()
            victim_image_pos = item.get_position_on_image()

            victims.append([victim_pos,victim_image_pos])

    return victims

