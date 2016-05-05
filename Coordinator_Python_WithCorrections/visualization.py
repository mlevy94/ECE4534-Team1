#!/usr/bin/env python
import threading
from threading import Thread
import socket
import time
from common import *
from math import cos, sin, radians
from random import randint

#import basic pygame modules
import pygame, os
from pygame.locals import *

data = b''

# The counter for the number of messages sent and received
SEND_COUNT = 0
RECEIVE_COUNT = 0

main_dir = os.path.split(os.path.abspath(__file__))[0]

# Global variables for move and turning in PyGame
MEASUREMENT_PERIOD = 0.1 # <- 100 milliseconds
CURRENT_DISTANCE = 0 # <- distance / angle
# Current vs. Target (next) angle/distance
CURRENT_ANGLE = 0
TEMP_ANGLE = 0      # Delta angle to ignore 0/360 degree issue
TARGET_DISTANCE = 0
TARGET_ANGLE = 0
# Moving/turning speeds
DISTANCE_SPEED = 0
ANGLE_SPEED = 0

# Boolean to eliminate sending out more than one message every time a token is found
TOKEN_TOGGLE = True
"""
For determining whether or not the rover should turn left or right
________________________________
Left    1
Right   2
"""
LEFT_OR_RIGHT = 0

# Boolean for messages being dropped to provide continual support for correction
IS_STOPPED = True

# Variables for variance in values due to incorrect sensor/rover values
ANGLE_BIAS = 2 # degrees
X_BIAS = 1  # Bias in the x position is 1 inch
Y_BIAS = 1 # Bias in the y position is 1 inch

# Positions and orientations
ROVER_X = 0
ROVER_Y = 0
ROVER_LENGTH = 6
ROVER_WIDTH = 6
ROVER_X_PIX = 0
ROVER_Y_PIX = 0
ROVER_ANGLE = 0
TOKEN_1_X = 0
TOKEN_1_Y = 0
TOKEN_2_X = 0
TOKEN_2_Y = 0
TOKEN_3_X = 0
TOKEN_3_Y = 0
TOKEN_4_X = 0
TOKEN_4_Y = 0
OBSTACLE_1_X = 0
OBSTACLE_1_Y = 0
OBSTACLE_2_X = 0
OBSTACLE_2_Y = 0
OBSTACLE_3_X = 0
OBSTACLE_3_Y = 0
OBSTACLE_4_X = 0
OBSTACLE_4_Y = 0

# Colors
WHITE = (255,255,255)
BLUE = (0,0,255)
BLACK = (0,0,0)
RED = (255,0,0)
GREEN = (0,255,0)
YELLOW = (255,255,0)
PURPLE = (128,0,128)
AQUA = (0,255,255)

"""
Extracted from the pygame.org Wiki example
Rotates the image while keeping its center
"""
def rotate(image, angle):
    orig_rect = image.get_rect()
    rot_image = pygame.transform.rotate(image, (360 - CURRENT_ANGLE))
    rot_rect = orig_rect.copy()
    rot_rect.center = rot_image.get_rect().center
    rot_image = rot_image.subsurface(rot_rect).copy()
    return rot_image
"""
Extracted from the pygame.org Wiki example
Rotates the image while keeping its center
"""
def rot_center(image, rect, angle):
    """rotate an image while keeping its center"""
    rot_image = pygame.transform.rotate(image, angle)
    rot_rect = rot_image.get_rect(center=rect.center)
    return rot_image,rot_rect

"""
Extracted from the moveit.py example from PyGame
"""
#quick function to load an image
def load_image(name):
    path = os.path.join(main_dir, 'data', name)
    return pygame.image.load(path).convert()

# Used for the test cases to test that the PIC commands work properly
# The location of the rover and obstacles don't matter because we are only testing that the rover takes commands properly
# This will be used for all the rover test cases
def roverTestCase():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE

    # These are distances from the origin (upper left)
    # The rover is midway through the map
    ROVER_X = 18
    ROVER_Y = 18
    ROVER_ANGLE = 90

# Used for testing an obstacle in free space
def obstacleTestCase1():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE
    global OBSTACLE_1_X
    global OBSTACLE_1_Y

    ROVER_X = 15
    ROVER_Y = 15
    ROVER_ANGLE = 0

    OBSTACLE_1_X = 3
    OBSTACLE_1_Y = 9

# Used for testing an obstacle on the left side
def obstacleTestCase2():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE
    global OBSTACLE_1_X
    global OBSTACLE_1_Y

    ROVER_X = 15
    ROVER_Y = 15
    ROVER_ANGLE = 270

    OBSTACLE_1_X = 0
    OBSTACLE_1_Y = 15

# Used for testing an obstacle on the right side
def obstacleTestCase3():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE
    global OBSTACLE_1_X
    global OBSTACLE_1_Y

    ROVER_X = 15
    ROVER_Y = 15
    ROVER_ANGLE = 270

    OBSTACLE_1_X = 33
    OBSTACLE_1_Y = 15

# Used for testing an obstacle on the upper side
def obstacleTestCase4():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE
    global OBSTACLE_1_X
    global OBSTACLE_1_Y

    ROVER_X = 15
    ROVER_Y = 15
    ROVER_ANGLE = 270

    OBSTACLE_1_X = 15
    OBSTACLE_1_Y = 0

# Used for testing an obstacle on the lower side
def obstacleTestCase5():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE
    global OBSTACLE_1_X
    global OBSTACLE_1_Y

    ROVER_X = 15
    ROVER_Y = 15
    ROVER_ANGLE = 270

    OBSTACLE_1_X = 15
    OBSTACLE_1_Y = 33

# Test case for horizontally moving around an obstacle
def obstacleTestCase6():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE
    global OBSTACLE_1_X
    global OBSTACLE_1_Y

    ROVER_X = 21
    ROVER_Y = 15
    ROVER_ANGLE = 270

    OBSTACLE_1_X = 15
    OBSTACLE_1_Y = 15

# Test case for vertically moving around an obstacle
def obstacleTestCase7():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE
    global OBSTACLE_1_X
    global OBSTACLE_1_Y

    ROVER_X = 9
    ROVER_Y = 3
    ROVER_ANGLE = 270

    OBSTACLE_1_X = 15
    OBSTACLE_1_Y = 15

# Used for testing rover traversal in an empty map on the upper left corner
def traversalTestCase1():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE

    ROVER_X = 0
    ROVER_Y = 0
    ROVER_ANGLE = 270

# Used for testing rover traversal in an empty map on the lower left corner
def traversalTestCase2():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE

    ROVER_X = 0
    ROVER_Y = 33
    ROVER_ANGLE = 270

# Used for testing rover traversal in an empty map on the upper right corner
def traversalTestCase3():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE

    ROVER_X = 33
    ROVER_Y = 0
    ROVER_ANGLE = 90

# Used for testing rover traversal in an empty map on the lower right corner
def traversalTestCase4():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE

    ROVER_X = 36
    ROVER_Y = 36
    ROVER_ANGLE = 90

# Used for testing rover traversal in an empty map in the center
def traversalTestCase5():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE

    ROVER_X = 18
    ROVER_Y = 18
    ROVER_ANGLE = 0

# Used for testing rover traversal in an occupied map on the upper left corner
def traversalTestCase6():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE
    global OBSTACLE_1_X
    global OBSTACLE_1_Y

    global OBSTACLE_2_X
    global OBSTACLE_2_Y

    global OBSTACLE_3_X
    global OBSTACLE_3_Y

    ROVER_X = 3
    ROVER_Y = 3
    ROVER_ANGLE = 0

    OBSTACLE_1_X = 0
    OBSTACLE_1_Y = 15

    OBSTACLE_2_X = 15
    OBSTACLE_2_Y = 15

    OBSTACLE_3_X = 15
    OBSTACLE_3_Y = 33

# Used for testing rover traversal in an occupied map on the lower left corner
def traversalTestCase7():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE
    global OBSTACLE_1_X
    global OBSTACLE_1_Y

    global OBSTACLE_2_X
    global OBSTACLE_2_Y

    global OBSTACLE_3_X
    global OBSTACLE_3_Y

    ROVER_X = 3
    ROVER_Y = 33
    ROVER_ANGLE = 180

    OBSTACLE_1_X = 3
    OBSTACLE_1_Y = 15

    OBSTACLE_2_X = 15
    OBSTACLE_2_Y = 15

    OBSTACLE_3_X = 15
    OBSTACLE_3_Y = 33

# Used for testing rover traversal in an occupied map on the upper right corner
def traversalTestCase8():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE
    global OBSTACLE_1_X
    global OBSTACLE_1_Y

    global OBSTACLE_2_X
    global OBSTACLE_2_Y

    global OBSTACLE_3_X
    global OBSTACLE_3_Y

    ROVER_X = 33
    ROVER_Y = 3
    ROVER_ANGLE = 0

    OBSTACLE_1_X = 3
    OBSTACLE_1_Y = 15

    OBSTACLE_2_X = 15
    OBSTACLE_2_Y = 15

    OBSTACLE_3_X = 15
    OBSTACLE_3_Y = 33

# Used for testing rover traversal in an occupied map on the lower right corner
def traversalTestCase9():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE
    global OBSTACLE_1_X
    global OBSTACLE_1_Y

    global OBSTACLE_2_X
    global OBSTACLE_2_Y

    global OBSTACLE_3_X
    global OBSTACLE_3_Y

    ROVER_X = 33
    ROVER_Y = 33
    ROVER_ANGLE = 180

    OBSTACLE_1_X = 3
    OBSTACLE_1_Y = 15

    OBSTACLE_2_X = 15
    OBSTACLE_2_Y = 15

    OBSTACLE_3_X = 15
    OBSTACLE_3_Y = 33

# Used for testing rover traversal in an occupied map on the left side
def traversalTestCase10():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE
    global OBSTACLE_1_X
    global OBSTACLE_1_Y

    global OBSTACLE_2_X
    global OBSTACLE_2_Y

    global OBSTACLE_3_X
    global OBSTACLE_3_Y

    ROVER_X = 3
    ROVER_Y = 15
    ROVER_ANGLE = 270

    OBSTACLE_1_X = 33
    OBSTACLE_1_Y = 15

    OBSTACLE_2_X = 15
    OBSTACLE_2_Y = 15

    OBSTACLE_3_X = 15
    OBSTACLE_3_Y = 33

# Used for testing rover traversal in an occupied map on the right side
def traversalTestCase11():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE
    global OBSTACLE_1_X
    global OBSTACLE_1_Y

    global OBSTACLE_2_X
    global OBSTACLE_2_Y

    global OBSTACLE_3_X
    global OBSTACLE_3_Y

    ROVER_X = 33
    ROVER_Y = 15
    ROVER_ANGLE = 90

    OBSTACLE_1_X = 3
    OBSTACLE_1_Y = 15

    OBSTACLE_2_X = 15
    OBSTACLE_2_Y = 15

    OBSTACLE_3_X = 15
    OBSTACLE_3_Y = 33

# Used for testing rover traversal in an occupied map on the upper side
def traversalTestCase12():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE
    global OBSTACLE_1_X
    global OBSTACLE_1_Y

    global OBSTACLE_2_X
    global OBSTACLE_2_Y

    global OBSTACLE_3_X
    global OBSTACLE_3_Y

    ROVER_X = 15
    ROVER_Y = 3
    ROVER_ANGLE = 0

    OBSTACLE_1_X = 3
    OBSTACLE_1_Y = 15

    OBSTACLE_2_X = 15
    OBSTACLE_2_Y = 15

    OBSTACLE_3_X = 15
    OBSTACLE_3_Y = 33

# Used for testing rover traversal in an occupied map on the lower side
def traversalTestCase13():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE
    global OBSTACLE_1_X
    global OBSTACLE_1_Y

    global OBSTACLE_2_X
    global OBSTACLE_2_Y

    global OBSTACLE_3_X
    global OBSTACLE_3_Y

    ROVER_X = 15
    ROVER_Y = 33
    ROVER_ANGLE = 180


    OBSTACLE_1_X = 3
    OBSTACLE_1_Y = 15

    OBSTACLE_2_X = 15
    OBSTACLE_2_Y = 15

    OBSTACLE_3_X = 15
    OBSTACLE_3_Y = 3

# Used for testing rover traversal in an occupied map in the center
def traversalTestCase14():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE

    global OBSTACLE_1_X
    global OBSTACLE_1_Y

    global OBSTACLE_2_X
    global OBSTACLE_2_Y

    global OBSTACLE_3_X
    global OBSTACLE_3_Y

    ROVER_X = 15
    ROVER_Y = 15
    ROVER_ANGLE = 0

    OBSTACLE_1_X = 3
    OBSTACLE_1_Y = 15

    OBSTACLE_2_X = 21
    OBSTACLE_2_Y = 15

    OBSTACLE_3_X = 15
    OBSTACLE_3_Y = 33

"""
Test cases for full traversal with multiple obstacles
"""

# Kyle's Test Case
def traversalTestCase15():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE

    global OBSTACLE_1_X
    global OBSTACLE_1_Y
    global OBSTACLE_2_X
    global OBSTACLE_2_Y
    global OBSTACLE_3_X
    global OBSTACLE_3_Y
    global OBSTACLE_4_X
    global OBSTACLE_4_Y

    global TOKEN_1_X
    global TOKEN_1_Y
    global TOKEN_2_X
    global TOKEN_2_Y
    global TOKEN_3_X
    global TOKEN_3_Y
    global TOKEN_4_X
    global TOKEN_4_Y

    ROVER_X = 15
    ROVER_Y = 15
    ROVER_ANGLE = 90

    OBSTACLE_1_X = 9
    OBSTACLE_1_Y = 9

    OBSTACLE_2_X = 21
    OBSTACLE_2_Y = 15

    OBSTACLE_3_X = 9
    OBSTACLE_3_Y = 27

    OBSTACLE_4_X = 27
    OBSTACLE_4_Y = 33

    TOKEN_1_X = 21
    TOKEN_1_Y = 3

    TOKEN_2_X = 27
    TOKEN_2_Y = 15

    TOKEN_3_X = 9
    TOKEN_3_Y = 21

    TOKEN_4_X = 21
    TOKEN_4_Y = 33

# Test case that is similar to Kyle's test case but has one obstacle blocking the way
# to get to the upper left hand corner
def traversalTestCase16():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE

    global OBSTACLE_1_X
    global OBSTACLE_1_Y
    global OBSTACLE_2_X
    global OBSTACLE_2_Y
    global OBSTACLE_3_X
    global OBSTACLE_3_Y
    global OBSTACLE_4_X
    global OBSTACLE_4_Y

    global TOKEN_1_X
    global TOKEN_1_Y
    global TOKEN_2_X
    global TOKEN_2_Y
    global TOKEN_3_X
    global TOKEN_3_Y
    global TOKEN_4_X
    global TOKEN_4_Y

    ROVER_X = 15
    ROVER_Y = 15
    ROVER_ANGLE = 90

    OBSTACLE_1_X = 3
    OBSTACLE_1_Y = 15

    OBSTACLE_2_X = 21
    OBSTACLE_2_Y = 15

    OBSTACLE_3_X = 9
    OBSTACLE_3_Y = 27

    OBSTACLE_4_X = 21
    OBSTACLE_4_Y = 33

    TOKEN_1_X = 21
    TOKEN_1_Y = 3

    TOKEN_2_X = 27
    TOKEN_2_Y = 15

    TOKEN_3_X = 9
    TOKEN_3_Y = 21

    TOKEN_4_X = 21
    TOKEN_4_Y = 33

# Test case for having two obstacles at both ends near the corners to test the ability to move around but also
# not miss a single blank space
def traversalTestCase17():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE

    global OBSTACLE_1_X
    global OBSTACLE_1_Y
    global OBSTACLE_2_X
    global OBSTACLE_2_Y
    global OBSTACLE_3_X
    global OBSTACLE_3_Y
    global OBSTACLE_4_X
    global OBSTACLE_4_Y

    global TOKEN_1_X
    global TOKEN_1_Y
    global TOKEN_2_X
    global TOKEN_2_Y
    global TOKEN_3_X
    global TOKEN_3_Y
    global TOKEN_4_X
    global TOKEN_4_Y

    ROVER_X = 15
    ROVER_Y = 15
    ROVER_ANGLE = 90

    OBSTACLE_1_X = 9
    OBSTACLE_1_Y = 3

    OBSTACLE_2_X = 9
    OBSTACLE_2_Y = 33

    OBSTACLE_3_X = 27
    OBSTACLE_3_Y = 3

    OBSTACLE_4_X = 27
    OBSTACLE_4_Y = 33

    TOKEN_1_X = 21
    TOKEN_1_Y = 3

    TOKEN_2_X = 27
    TOKEN_2_Y = 15

    TOKEN_3_X = 9
    TOKEN_3_Y = 21

    TOKEN_4_X = 21
    TOKEN_4_Y = 33

# Test case for all obstacles in each side
def traversalTestCase18():
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE

    global OBSTACLE_1_X
    global OBSTACLE_1_Y
    global OBSTACLE_2_X
    global OBSTACLE_2_Y
    global OBSTACLE_3_X
    global OBSTACLE_3_Y
    global OBSTACLE_4_X
    global OBSTACLE_4_Y

    global TOKEN_1_X
    global TOKEN_1_Y
    global TOKEN_2_X
    global TOKEN_2_Y
    global TOKEN_3_X
    global TOKEN_3_Y
    global TOKEN_4_X
    global TOKEN_4_Y

    ROVER_X = 15
    ROVER_Y = 15
    ROVER_ANGLE = 90

    OBSTACLE_1_X = 3
    OBSTACLE_1_Y = 15

    OBSTACLE_2_X = 15
    OBSTACLE_2_Y = 3

    OBSTACLE_3_X = 21
    OBSTACLE_3_Y = 33

    OBSTACLE_4_X = 33
    OBSTACLE_4_Y = 15

    TOKEN_1_X = 21
    TOKEN_1_Y = 3

    TOKEN_2_X = 27
    TOKEN_2_Y = 15

    TOKEN_3_X = 9
    TOKEN_3_Y = 21

    TOKEN_4_X = 27
    TOKEN_4_Y = 33

# Function to send out the rover update
def finishedMoving():
    if IS_STOPPED == True:
        print('IS_STOPPED was called')
        content = toObjectPositionMessage(LEAD_ROVER_UPDATE, 0x0000, 0x0000, 0x0000, ROVER_LENGTH, ROVER_WIDTH)
        fullMessage = convertToMessage(OBJECT_POS, content)
        s.send(fullMessage)
    else:
        print('Target Angle: ' + str(TARGET_ANGLE) + '\tDelta Angle: ' + str(TEMP_ANGLE))

    # Timer to continually increment
    threading.Timer(1.2, finishedMoving).start()

# Assumes TARGET_ANGLE was reassigned elsewhere
def roverTurn():
    global CURRENT_ANGLE
    global TARGET_ANGLE
    global TEMP_ANGLE
    global IS_STOPPED

    #if fabs(fmod(CURRENT_ANGLE, 360) - 180) != TARGET_ANGLE:
    if TEMP_ANGLE < TARGET_ANGLE:
        # LEFT_OR_RIGHT == 1 means turn left
        if LEFT_OR_RIGHT == 1:
            CURRENT_ANGLE -= ANGLE_SPEED
            TEMP_ANGLE += ANGLE_SPEED
        # LEFT_OR_RIGHT == 2 means turn right
        elif LEFT_OR_RIGHT == 2:
            CURRENT_ANGLE += ANGLE_SPEED
            TEMP_ANGLE += ANGLE_SPEED

        # Prevents CURRENT_ANGLE from getting absurdly negative or positive
        CURRENT_ANGLE %= 360
        print('Current Angle: ' + str(CURRENT_ANGLE) + '\tTarget Angle: ' + str(TARGET_ANGLE)) #<- this is masking the negative values but they exist
        #print(TARGET_ANGLE)
        #CURRENT_ANGLE %= 360

        # Need to send the lead rover update to notify the rover to give out the next message
        #if fabs(fmod(CURRENT_ANGLE, 360) - 180) == TARGET_ANGLE:
        if TEMP_ANGLE >= TARGET_ANGLE:
            IS_STOPPED = True

    # Timer to continually increment
    threading.Timer(MEASUREMENT_PERIOD, roverTurn).start()

def roverMove():
    global CURRENT_DISTANCE
    global ROVER_X_PIX
    global ROVER_Y_PIX
    global ROVER_X
    global ROVER_Y
    global IS_STOPPED

    RANDOM_VARIANCE = randint(-1, 1)

    if CURRENT_DISTANCE < TARGET_DISTANCE:
        CURRENT_DISTANCE += DISTANCE_SPEED
        """ROVER_X_PIX += DISTANCE_SPEED * cos(radians(CURRENT_ANGLE - 90)) + RANDOM_VARIANCE
        ROVER_Y_PIX += DISTANCE_SPEED * sin(radians(CURRENT_ANGLE - 90)) + RANDOM_VARIANCE"""
        ROVER_X_PIX += DISTANCE_SPEED * cos(radians(CURRENT_ANGLE - 90))
        ROVER_Y_PIX += DISTANCE_SPEED * sin(radians(CURRENT_ANGLE - 90))

        # Need to send the lead rover update to notify the rover to give out the next message
        if CURRENT_DISTANCE >= TARGET_DISTANCE:
            IS_STOPPED = True

    # Timer to continually increment
    threading.Timer(MEASUREMENT_PERIOD, roverMove).start()

"""
Message Structure:
        Bytes:                                              Content:
_______________________________________________________________________________________________
        0                                                   Start Byte
        1                                                   Sender
        2                                                   Message Number
        3                                                   Message Type
        4                                                   Message Size
        5-16                                                Message
17 (or anything after the last byte of Message)             End Byte
_______________________________________________________________________________________________
"""

# Converts a byte message into an internal message that can be used to send out a message
# Really this is just a fancy way of extracting data I want to keep a hold of in a class
def convertFromMessage(message):
    global RECEIVE_COUNT

    # Increment the send counter
    RECEIVE_COUNT += 1
    # The sender / client of the message
    client = message[1]
    # The message number for the number of messages this client has received
    RECEIVE_COUNT = message[2]
    # Message type
    messagetype = message[3]
    # Message size
    messagesize = message[4]
    msg = []
    # Takes the message as a bytes string
    for x in range (0, messagesize):
        msg.append(message[x + 5])

    # Creates an internal message for easy data extraction
    convertedMessage = InternalMessage(client, messagetype, msg)
    return convertedMessage

# Converts to a message that can be sent wirelessly
def convertToMessage(messageType, message):
    global SEND_COUNT

    # Client / Message sender
    sender = VISUALIZATION
    # The message number for the number of messages this client has sent
    messageNo = SEND_COUNT % 256
    # Message type
    type = messageType
    # Message size
    msgSize = len(message)
    # Message without the start and end bytes
    incompleteMessage = bytes([sender, messageNo, type, msgSize]) + message

    # Increment the send counter
    SEND_COUNT += 1

    # Return the full message
    return START_BYTE + incompleteMessage + END_BYTE

# Function to convert a message to a list of data used for the rover command
# Returns in the following format: roverCommand, distance
# e.g. roverCommand = ROVER_LEFT, distance 120 <- degrees
# distance will always be a 16-bit value and roverCommand will always be an 8-bit value
def fromRoverMoveMessage(message):
    # Get the type of movement the rover needs to do
    roverCommand = message[0]

    # Get the actual distance / angle change that must be performed
    # Bit manipulation to get the correct distance
    # Distance on the PIC end uses only a char as the bit size
    distance = message[1]

    return roverCommand, distance

def fromObjectPositionMessage(message):
    # Picks up the whole message to return the data as a list
    messagesize = len(message)
    msg = []
    for x in range (0, messagesize):
        msg.append(message[x])

    objectType = msg[0]
    x = ((msg[1]) << 8) | msg[2]
    y = ((msg[3]) << 8) | msg[4]
    angle = ((msg[5]) << 8) | msg[6]
    length = ((msg[7]) << 8) | msg[8]
    width = ((msg[9]) << 8) | msg[10]

    return objectType, x, y, angle, length, width

# Function to convert a command for the rover to move into a bytes string to be
# available for immediate sending to the socket
def toRoverMoveMessage(roverCommand, distance):
    """
    Rover command is different from the message type 'ROVER_MOVE'
    Rover command contains one of the following types:
    __________________________________________________________
    ROVER_FORWARD       0x01
    ROVER_BACKWARD      0x02
    ROVER_LEFT          0x04
    ROVER_RIGHT         0x08
    ROVER_STOP          0x10
    """
    distanceHigh = (distance & 0xFF00) >> 8
    distanceLow = (distance & 0x00FF)
    return bytes([roverCommand, distanceHigh, distanceLow])

# Function to convert an object position to a bytes string to be available
# for immediate sending to the socket
def toObjectPositionMessage(objectType, x, y, angle, length, width):
    """
    For tokens and obstacles, the angle represents the total number of those
    objects because the angle does not matter
    """
    xHigh = (int(x) & 0xFF00) >> 8
    xLow = (int(x) & 0x00FF)
    yHigh = (int(y) & 0xFF00) >> 8
    yLow = (int(y) & 0x00FF)
    angleHigh = (int(angle) & 0xFF00) >> 8
    angleLow = (int(angle) & 0x00FF)
    lengthHigh = (int(length) & 0xFF00) >> 8
    lengthLow = (int(length) & 0x00FF)
    widthHigh = (int(width) & 0xFF00) >> 8
    widthLow = (int(width) & 0x00FF)
    return bytes([objectType, xHigh, xLow, yHigh, yLow, angleHigh,
                  angleLow, lengthHigh, lengthLow, widthHigh, widthLow])

# Function for the thread for continually listening on the socket
def listening():
    global data
    global TARGET_ANGLE
    global TEMP_ANGLE
    global CURRENT_DISTANCE
    global TARGET_DISTANCE
    global LEFT_OR_RIGHT
    global IS_STOPPED
    global s
    size = 1024

    while 1:
        data = s.recv(size)
        if data:
            print('Received:', data)

            # Converting the bytes string to the data I want
            intMessage = convertFromMessage(data)
            print('Receive: {} - {} : {}'.format(VAL_TO_ROLE[intMessage.client], VAL_TO_MSG[intMessage.msgtype],
                                                         intMessage.msg))

            # Interpreting the data for the visualization
            if intMessage.msgtype == ROVER_MOVE:
                roverCommandType, distance = fromRoverMoveMessage(intMessage.msg)
                # Command for making the rover move forward
                if roverCommandType == ROVER_FORWARD:
                    IS_STOPPED = False
                    CURRENT_DISTANCE = 0
                    TARGET_DISTANCE = distance * 20
                # Command for making the rover turn left
                elif roverCommandType == ROVER_LEFT:
                    print('I entered ROVER_LEFT')
                    TEMP_ANGLE = 0
                    # Assign LEFT_OR_RIGHT for the distance thread
                    LEFT_OR_RIGHT = 1
                    # TARGET_ANGLE is affected by the negative values of CURRENT_ANGLE
                    # This is an attempt to maintain a positive TARGET_ANGLE
                    TARGET_ANGLE = distance

                    if TARGET_ANGLE != 0:
                        IS_STOPPED = False

                #Command for making the rover turn right
                elif roverCommandType == ROVER_RIGHT:
                    print('I entered ROVER_RIGHT')
                    TEMP_ANGLE = 0
                    # Assign LEFT_OR_RIGHT for the distance thread
                    LEFT_OR_RIGHT = 2
                    # TARGET_ANGLE is affected by the negative values of CURRENT_ANGLE
                    # This is an attempt to maintain a positive TARGET_ANGLE
                    TARGET_ANGLE = distance

                    if TARGET_ANGLE != 0:
                        IS_STOPPED = False

                # Command for making the rover stop moving
                elif roverCommandType == ROVER_STOP:
                    IS_STOPPED = True
                    TARGET_ANGLE = 0
                    TARGET_DISTANCE = 0
                    # Update message to do corrections
                    content = toObjectPositionMessage(LEAD_ROVER_UPDATE, 0x0000, 0x0000, 0x0000, ROVER_LENGTH, ROVER_WIDTH)
                    fullMessage = convertToMessage(OBJECT_POS, content)
                    s.send(fullMessage)

            # Receiving an object position message
            # I would never need to see this unless I needed to debug
            elif intMessage.msgtype == OBJECT_POS:
                objectType, xPos, yPos,angleDegrees, objectLength, objectWidth = fromObjectPositionMessage(intMessage.msg)
            elif intMessage.msgtype == TOKEN_FOUND:
                print('Token acknowledged')
            elif intMessage.msgtype == TASK_COMPLETED:
                print('Failure')

# Function for the periodic thread to emulate the sensor reporting data on the rover
def sensorUpdate():
    global TOKEN_TOGGLE
    global X_BIAS
    global Y_BIAS

    RANDOM_VARIANCE = randint(-1, 1) # Random variance in values from -2 to 2 inches / degrees
    ROVER_X_ERROR = ((ROVER_X_PIX + 60) / 20) + X_BIAS + RANDOM_VARIANCE
    ROVER_Y_ERROR = ((ROVER_Y_PIX + 60) / 20) + Y_BIAS + RANDOM_VARIANCE
    ROVER_ANGLE_ERROR = (CURRENT_ANGLE + ANGLE_BIAS + RANDOM_VARIANCE) % 360
    content = toObjectPositionMessage(ROVER, ROVER_X_ERROR, ROVER_Y_ERROR,
                                      ROVER_ANGLE_ERROR, ROVER_LENGTH, ROVER_WIDTH)
    """content = toObjectPositionMessage(ROVER, ((ROVER_X_PIX + 60) / 20), ((ROVER_Y_PIX + 60) / 20),
                                      CURRENT_ANGLE, ROVER_LENGTH, ROVER_WIDTH)"""
    fullMessage = convertToMessage(OBJECT_POS, content)
    s.send(fullMessage)

    if int((ROVER_X_PIX + 60) / 20) == (TOKEN_1_X + RANDOM_VARIANCE) and int((ROVER_Y_PIX + 60) / 20) == (TOKEN_1_Y + RANDOM_VARIANCE):
        print('Token 1')
        content = toObjectPositionMessage(0, 0, 0, 4, 6, 6)
        fullMessage = convertToMessage(TOKEN_FOUND, content)
        if TOKEN_TOGGLE == False:
            print('Sending token 1')
            s.send(fullMessage)
            TOKEN_TOGGLE = True
    elif int((ROVER_X_PIX + 60) / 20) == (TOKEN_2_X + RANDOM_VARIANCE) and int((ROVER_Y_PIX + 60) / 20) == (TOKEN_2_Y + RANDOM_VARIANCE):
        print('Token 2')
        content = toObjectPositionMessage(0, 0, 0, 4, 6, 6)
        fullMessage = convertToMessage(TOKEN_FOUND, content)
        if TOKEN_TOGGLE == False:
            print('Sending token 2')
            s.send(fullMessage)
            TOKEN_TOGGLE = True
    elif int((ROVER_X_PIX + 60) / 20) == (TOKEN_3_X + RANDOM_VARIANCE) and int((ROVER_Y_PIX + 60) / 20) == (TOKEN_3_Y + RANDOM_VARIANCE):
        print('Token 3')
        content = toObjectPositionMessage(0, 0, 0, 4, 6, 6)
        fullMessage = convertToMessage(TOKEN_FOUND, content)
        if TOKEN_TOGGLE == False:
            print('Sending token 3')
            s.send(fullMessage)
            TOKEN_TOGGLE = True
    elif int((ROVER_X_PIX + 60) / 20) == (TOKEN_4_X + RANDOM_VARIANCE) and int((ROVER_Y_PIX + 60) / 20) == (TOKEN_4_Y + RANDOM_VARIANCE):
        print('Token 4')
        content = toObjectPositionMessage(0, 0, 0, 4, 6, 6)
        fullMessage = convertToMessage(TOKEN_FOUND, content)
        if TOKEN_TOGGLE == False:
            print('Sending token 4')
            s.send(fullMessage)
            TOKEN_TOGGLE = True
    else:
        TOKEN_TOGGLE = False

    # Timer to continually increment
    threading.Timer(0.1, sensorUpdate).start()

def main():
    global ROVER_X_PIX
    global ROVER_Y_PIX
    global ROVER_X
    global ROVER_Y
    global ROVER_ANGLE
    global DISTANCE_SPEED
    global ANGLE_SPEED
    global CURRENT_ANGLE
    global TARGET_ANGLE
    global TARGET_DISTANCE

    # Test case
    traversalTestCase17()

    # Making sure the current angle reflects the angle the rover should be facing
    CURRENT_ANGLE = ROVER_ANGLE
    RANDOM_VARIANCE = randint(-1, 0) # Random variance in values from -2 to 2 inches / degrees

    # Test Case Messages
    #ROVER_X_ERROR = ROVER_X + X_BIAS + RANDOM_VARIANCE
    #ROVER_Y_ERROR = ROVER_Y + Y_BIAS + RANDOM_VARIANCE
    ROVER_ANGLE_ERROR = ROVER_ANGLE + ANGLE_BIAS + RANDOM_VARIANCE
    OBSTACLE_1_X_ERROR = OBSTACLE_1_X + RANDOM_VARIANCE
    OBSTACLE_1_Y_ERROR = OBSTACLE_1_Y + RANDOM_VARIANCE
    OBSTACLE_2_X_ERROR = OBSTACLE_2_X + RANDOM_VARIANCE
    OBSTACLE_2_Y_ERROR = OBSTACLE_2_Y + RANDOM_VARIANCE
    OBSTACLE_3_X_ERROR = OBSTACLE_3_X + RANDOM_VARIANCE
    OBSTACLE_3_Y_ERROR = OBSTACLE_3_Y + RANDOM_VARIANCE
    OBSTACLE_4_X_ERROR = OBSTACLE_4_X + RANDOM_VARIANCE
    OBSTACLE_4_Y_ERROR = OBSTACLE_4_Y + RANDOM_VARIANCE
    """msg1 = convertToMessage(OBJECT_POS, bytes([ROVER, ROVER_X & 0xFF00, ROVER_X & 0x00FF, ROVER_Y & 0xFF00,
                                               ROVER_Y & 0x00FF, ROVER_ANGLE_ERROR & 0xFF00, ROVER_ANGLE_ERROR & 0x00FF,
                                               0x00, 0x06, 0x00, 0x06]))
    msg2 = convertToMessage(OBJECT_POS, bytes([OBSTACLE, OBSTACLE_1_X_ERROR & 0xFF00, OBSTACLE_1_X_ERROR & 0x00FF,
                                               OBSTACLE_1_Y_ERROR & 0xFF00, OBSTACLE_1_Y_ERROR & 0x00FF,
                                               0x00, 0x04, 0x00, 0x06, 0x00, 0x06]))
    msg3 = convertToMessage(OBJECT_POS, bytes([OBSTACLE, OBSTACLE_2_X_ERROR & 0xFF00, OBSTACLE_2_X_ERROR & 0x00FF,
                                               OBSTACLE_2_Y_ERROR & 0xFF00, OBSTACLE_2_Y_ERROR & 0x00FF,
                                               0x00, 0x04, 0x00, 0x06, 0x00, 0x06]))
    msg4 = convertToMessage(OBJECT_POS, bytes([OBSTACLE, OBSTACLE_3_X_ERROR & 0xFF00, OBSTACLE_3_X_ERROR & 0x00FF,
                                               OBSTACLE_3_Y_ERROR & 0xFF00, OBSTACLE_3_Y_ERROR & 0x00FF,
                                               0x00, 0x04, 0x00, 0x06, 0x00, 0x06]))
    msg5 = convertToMessage(OBJECT_POS, bytes([OBSTACLE, OBSTACLE_4_X_ERROR & 0xFF00, OBSTACLE_4_X_ERROR & 0x00FF,
                                               OBSTACLE_4_Y_ERROR & 0xFF00, OBSTACLE_4_Y_ERROR & 0x00FF,
                                               0x00, 0x04, 0x00, 0x06, 0x00, 0x06]))"""
    msg1 = convertToMessage(OBJECT_POS, bytes([ROVER, ROVER_X & 0xFF00, ROVER_X & 0x00FF, ROVER_Y & 0xFF00,
                                               ROVER_Y & 0x00FF, ROVER_ANGLE & 0xFF00, ROVER_ANGLE & 0x00FF,
                                               0x00, 0x06, 0x00, 0x06]))
    msg2 = convertToMessage(OBJECT_POS, bytes([OBSTACLE, OBSTACLE_1_X & 0xFF00, OBSTACLE_1_X & 0x00FF,
                                               OBSTACLE_1_Y & 0xFF00, OBSTACLE_1_Y & 0x00FF,
                                               0x00, 0x04, 0x00, 0x06, 0x00, 0x06]))
    msg3 = convertToMessage(OBJECT_POS, bytes([OBSTACLE, OBSTACLE_2_X & 0xFF00, OBSTACLE_2_X & 0x00FF,
                                               OBSTACLE_2_Y & 0xFF00, OBSTACLE_2_Y & 0x00FF,
                                               0x00, 0x04, 0x00, 0x06, 0x00, 0x06]))
    msg4 = convertToMessage(OBJECT_POS, bytes([OBSTACLE, OBSTACLE_3_X & 0xFF00, OBSTACLE_3_X & 0x00FF,
                                               OBSTACLE_3_Y & 0xFF00, OBSTACLE_3_Y & 0x00FF,
                                               0x00, 0x04, 0x00, 0x06, 0x00, 0x06]))
    msg5 = convertToMessage(OBJECT_POS, bytes([OBSTACLE, OBSTACLE_4_X & 0xFF00, OBSTACLE_4_X & 0x00FF,
                                               OBSTACLE_4_Y & 0xFF00, OBSTACLE_4_Y & 0x00FF,
                                               0x00, 0x04, 0x00, 0x06, 0x00, 0x06]))

    # Sending out all the initial messages
    print('Sending obstacle 1 Message')
    s.send(msg2)
    time.sleep(0.5)
    print('Sending obstacle 2 Message')
    s.send(msg3)
    time.sleep(0.5)
    print('Sending obstacle 3 Message')
    s.send(msg4)
    time.sleep(0.5)
    print('Sending obstacle 4 Message')
    s.send(msg5)
    time.sleep(0.5)
    print('Sending rover Message')
    s.send(msg1)
    time.sleep(0.5)

    # Initializes the PyGame usage
    pygame.init()

    # Screen of 720 pix x 720 pix
    screen = pygame.display.set_mode((720, 720))
    # Image is 150 pix x 150 pix, rectangle is 94 pix x 114 pix
    roverImage = load_image('C:/ECE_4534_WithCorrections/rover_orig.png')

    # Fill background
    background = pygame.Surface(screen.get_size())
    background = background.convert()
    background.fill(WHITE)
    # Default background for when the screen is being flushed
    background.set_colorkey((255,255,255))

    # Displaying a rectangle as the rover
    # Subtracting the X position by 60 pixels due to image pixelation for easier look of turning
    ROVER_X_PIX = (ROVER_X * 20) - 60
    ROVER_Y_PIX = (ROVER_Y * 20) - 60
    DISTANCE_SPEED = 10 #<- 10 pix / call
    ANGLE_SPEED = 5 # degrees / call

    # Clock used to control the frame rate
    animationTime = pygame.time.Clock()

    # Blit everything to the screen
    screen.blit(background, (0, 0))

    # Update the screen
    pygame.display.update()

    roverTurn()
    roverMove()
    finishedMoving()

    sensorUpdate()

    while 1:
        # Checking for the exit button being pressed
        for event in pygame.event.get():
            if event.type == QUIT:
                return

        # Flushing the screen every time we re-blit
        screen.fill(WHITE)

        # Tokens
        pygame.draw.circle(screen, GREEN, ((TOKEN_1_X * 20), (TOKEN_1_Y * 20)), 60, 0)
        pygame.draw.circle(screen, GREEN, ((TOKEN_2_X * 20), (TOKEN_2_Y * 20)), 60, 0)
        pygame.draw.circle(screen, GREEN, ((TOKEN_3_X * 20), (TOKEN_3_Y * 20)), 60, 0)
        pygame.draw.circle(screen, GREEN, ((TOKEN_4_X * 20), (TOKEN_4_Y * 20)), 60, 0)
        # Obstacles
        pygame.draw.rect(screen, RED, ((OBSTACLE_1_X * 20) - 60, (OBSTACLE_1_Y * 20) - 60, 120, 120), 0)
        pygame.draw.rect(screen, RED, ((OBSTACLE_2_X * 20) - 60, (OBSTACLE_2_Y * 20) - 60, 120, 120), 0)
        pygame.draw.rect(screen, RED, ((OBSTACLE_3_X * 20) - 60, (OBSTACLE_3_Y * 20) - 60, 120, 120), 0)
        pygame.draw.rect(screen, RED, ((OBSTACLE_4_X * 20) - 60, (OBSTACLE_4_Y * 20) - 60, 120, 120), 0)

        rotatedImage = rotate(roverImage, CURRENT_ANGLE)
        screen.blit(rotatedImage, (ROVER_X_PIX, ROVER_Y_PIX))

        # Update the screen and clock
        pygame.display.update()
        animationTime.tick(60)
    s.close()

if __name__ == '__main__':
    global s
    host = '192.168.71.135'
    port = 59784
    size = 1024

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host,port))

    # Starting the listening thread
    listeningThread = Thread(target=listening, daemon=True)
    listeningThread.start()

    # Starting the main function
    main()