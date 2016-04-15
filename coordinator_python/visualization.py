#!/usr/bin/env python
import threading
from listener import Listener
from configs import *
from queue import Empty
from math import cos, sin, radians

#import basic pygame modules
import pygame, os
from pygame.locals import *

main_dir = os.path.split(os.path.abspath(__file__))[0]

MEASUREMENT_PERIOD = 0.5 # <- 500 milliseconds
CURRENT_DISTANCE = 0 # <- distance / anglej39h354
TARGET_DISTANCE = 0
WHITE = (255,255,255)
BLUE = (0,0,255)
BLACK = (0,0,0)
RED = (255,0,0)
GREEN = (0,255,0)
YELLOW = (255,255,0)
PURPLE = (128,0,128)
AQUA = (0,255,255)

class rover:
    def __init__(self, image, x, y, angle, move_speed, turn_speed):
        #self.position = image.get_rect().move(x, y)
        self.xPos = x
        self.yPos = y
        self.distanceSpeed =  move_speed
        self.angleSpeed = turn_speed
        self.angle = angle
        self.color = AQUA
        self.rectangle = image # <- initial location with size of 6" x 6"

    def turnLeft(self, angle):
        global CURRENT_DISTANCE
        global ROVER_ANGLE

        # Break if I've already gone however far I needed to
        if CURRENT_DISTANCE >= TARGET_DISTANCE:
            CURRENT_DISTANCE = 0
            return
        # Increment the current distance
        CURRENT_DISTANCE += (self.angleSpeed * MEASUREMENT_PERIOD)

        # Adds the degrees | Also eliminates the problem of negative angles by returning it to positive by
        # providing a full revolution of 360 degrees
        self.angle -= (self.angleSpeed * MEASUREMENT_PERIOD) + 360
        # Removes 360 degrees if a full revolution is realized
        self.angle %= 360
        threading.Timer(MEASUREMENT_PERIOD, self.turnLeft, angle).start()

    def turnRight(self, angle):
        global CURRENT_DISTANCE
        global ROVER_ANGLE

        # Break if I've already gone however far I needed to
        if CURRENT_DISTANCE >= TARGET_DISTANCE:
            CURRENT_DISTANCE = 0
            return
        # Increment the current distance
        CURRENT_DISTANCE += (self.angleSpeed * MEASUREMENT_PERIOD)

        # Adds the degrees | Also eliminates the problem of negative angles by returning it to positive by
        # providing a full revolution of 360 degrees
        self.angle += (self.angleSpeed * MEASUREMENT_PERIOD) + 360
        # Removes 360 degrees if a full revolution is realized
        self.angle %= 360
        threading.Timer(MEASUREMENT_PERIOD, self.turnRight, angle).start()

    def move(self, distance):
        global CURRENT_DISTANCE
        global ROVER_ANGLE

        # Break if I've already gone however far I needed to
        if CURRENT_DISTANCE >= TARGET_DISTANCE:
            CURRENT_DISTANCE = 0
            return
        # Increment the current distance
        CURRENT_DISTANCE += (self.distanceSpeed * MEASUREMENT_PERIOD)

        self.position = self.position.move((self.speed * cos(radians(self.angle - 90))) * MEASUREMENT_PERIOD,
                                           (self.speed * sin(radians(self.angle - 90))) * MEASUREMENT_PERIOD)
        threading.Timer(MEASUREMENT_PERIOD, self.move, distance).start()

"""
Extracted from the pygame.org Wiki example
Rotates the image while keeping its center
"""
def rotate(image, angle):
        orig_rect = image.get_rect()
        rot_image = pygame.transform.rotate(image, angle)
        rot_rect = orig_rect.copy()
        rot_rect.center = rot_image.get_rect().center
        rot_image = rot_image.subsurface(rot_rect).copy()
        return rot_image

def rot_center(image, rect, angle):
        """rotate an image while keeping its center"""
        rot_image = pygame.transform.rotate(image, angle)
        rot_rect = rot_image.get_rect(center=rect.center)
        return rot_image,rot_rect

#quick function to load an image
def load_image(name):
    path = os.path.join(main_dir, 'data', name)
    return pygame.image.load(path).convert()

def main():
    # Building the listener object
    listener = Listener()
    listener.start()

    pygame.init()

    # Screen of 720 pix x 720 pix
    screen = pygame.display.set_mode((720, 720))

    # Image is 150 pix x 150 pix, rectangle is 94 pix x 114 pix
    roverImage = load_image('C:/Windows_Serial_Simulation/rover7.png')

    # Fill background
    background = pygame.Surface(screen.get_size())
    background = background.convert()
    background.fill(WHITE)
    background.set_colorkey((255,0,0))

    # Displaying a rectangle as the rover
    roverObject = rover(roverImage, 100, 100, 0, 40, 100)
    pygame.draw.circle(screen, GREEN, (650, 650), 60, 0)

    time = pygame.time.Clock()

    angle = 0

    # Blit everything to the screen
    screen.blit(background, (0, 0))
    screen.blit(roverObject.rectangle, (roverObject.xPos, roverObject.yPos))
    pygame.display.flip()

    while 1:
        # Checking for the exit button being pressed
        for event in pygame.event.get():
            if event.type == QUIT:
                return

        screen.fill(WHITE)

        # Attempting to get a new message to update the system
        """try:
            msg = listener.queue.get()

            # Ensuring initialization occurs properly
            if msg.msgtype == CLIENT_ROLE:
                print("Message Received: {} - {}".format(VAL_TO_MSG[msg.msgtype], msg.msg))

            # Update on the position of the rover
            elif msg.msgtype == OBJECT_POS:
                print("Message Received: {} - {}".format(VAL_TO_MSG[msg.msgtype], msg.msg))

            # Update on the commands by the coordinator
            elif msg.msgtype == ROVER_MOVE:
                print("Message Received: {} - {}".format(VAL_TO_MSG[msg.msgtype], msg.msg))
        except Empty:
            pass"""

        #background.fill(WHITE)
        # Displaying a rectangle as the rover
        pygame.draw.circle(screen, GREEN, (650, 650), 60, 0)

        #screen.blit(background, roverObject.rectangle)
        #screen.blit(roverObject.rectangle, (200,200))

        #rotatedImage, rotatedRect = rotate(roverObject.rectangle, roverObject.rectangle.get_rect(), angle)
        #roverObject.rectangle = rotatedImage
        """rotatedImage = pygame.transform.rotate(roverObject.rectangle, angle)
        rotatedRect = rotatedImage.get_rect()
        screen.fill(WHITE)
        screen.blit(rotatedImage, rotatedRect)"""
        #roverObject.rectangle = rotatedImage.get_rect(center=roverObject.rectangle.center)
        #roverObject.rectangle.center = curCenter
        #token1 = rotatedImage.get_rect(center=token1.center)
        #token1.center = curCircleCenter

        #screen.blit(rotatedImage, rotatedRect)
        angle += 5
        if angle >= 360:
            angle = 0

        rotatedImage = rotate(roverObject.rectangle, angle)
        #rotatedRect = rotatedImage.get_rect()
        screen.blit(rotatedImage, (roverObject.xPos, roverObject.yPos))
        #pygame.draw.circle(background, GREEN, (650, 650), 60, 0)

        #screen.blit(background, roverObject.rectangle)
        #screen.blit(background, (0, 0))
        pygame.display.flip()
        time.tick(60)
    #listener.close()

if __name__ == '__main__':
    main()