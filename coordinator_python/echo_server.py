#!/usr/bin/env python
from listener import Listener
from configs import *
import threading
from queue import Empty
from math import cos,sin,radians
#from visualization import *

# Global variables for the rover coordinates
# Current rover locations
ROVER_X = 0
ROVER_Y = 0
ROVER_ANGLE = 0

# Time-based constants for use in calculating the distance taken
ROVER_SPEED = 2 # <- inches / second
ROVER_ANGLE_SPEED = 90 # <- this speed is to make the simulation run faster otherwise use 5 # degrees / second
MEASUREMENT_PERIOD = 0.5 # <- seconds = 100 milliseconds

# The distance from a ROVER_MOVE command
TARGET_DISTANCE = 0

# The current distance of travel
CURRENT_DISTANCE = 0

# Global variables for the obstacle(s) coordinates
OBSTACLE_1_X = 0
OBSTACLE_1_Y = 0

OBSTACLE_2_X = 0
OBSTACLE_2_Y = 0

OBSTACLE_3_X = 0
OBSTACLE_3_Y = 0

OBSTACLE_4_X = 0
OBSTACLE_4_Y = 0

# Global variables for the token(s) coordinates
TOKEN_1_X = 0
TOKEN_1_Y = 0

TOKEN_2_X = 0
TOKEN_2_Y = 0

TOKEN_3_X = 0
TOKEN_3_Y = 0

TOKEN_4_X = 0
TOKEN_4_Y = 0

# Building an internal message to be sent to the coordinator
"""
Function for converting data to an internal message to be sent
Object type:
  - ROVER
  - OBSTACLE

  Origin is the upper left corner

X Position:
  0 inches to 36 inches

Y position:
  0 inches to 36 inches

Angle:
  - Distance in terms of degrees. Increments of 5 degrees.
  North = 0 degrees

  - Total count of obstacles
  0x02 = 2 obstacles in total on the map
  Example:
    ROVER_LEFT, 90  = turn left 90 degrees
    ROVER_RIGHT, 15 = turn right 15 degrees

Length:
  Length of the rover. This should be the North to South distance.

Width:
  Width of the rover. This should be the West to East distance.
"""
def convertToMessage(objectType, xPos, yPos, angle, length, width):
  correctedXPosH = (xPos & 0xFF00) >> 8
  correctedXPosL = (xPos & 0x00FF)
  correctedYPosH = (yPos & 0xFF00) >> 8
  correctedYPosL = (yPos & 0x00FF)
  correctedAngleH = (angle & 0xFF00) >> 8
  correctedAngleL = (angle & 0x00FF)
  correctedLengthH = (length & 0xFF00) >> 8
  correctedLengthL = (length & 0x00FF)
  correctedWidthH = (width & 0xFF00) >> 8
  correctedWidthL = (width & 0x00FF)
  return InternalMessage(ROUTER, OBJECT_POS, bytes([objectType, correctedXPosH, correctedXPosL, correctedYPosH,
                                                    correctedYPosL, correctedAngleH, correctedAngleL, correctedLengthH,
                                                    correctedLengthL, correctedWidthH, correctedWidthL]))

"""
Function to return a list of the message data in the order expected <- I don't think I'll ever need to decipher this
                                                                       kind of message but good practice for the PIC
"""
def decipherMessage(message):
  objectType = message & 0x001
  xPosition = (message & 0x002) << 8 | (message & 0x004)
  yPosition = (message & 0x008) << 8 | (message & 0x010)
  angle = (message & 0x020) << 8 | (message & 0x040)
  length = (message & 0x080) << 8 | (message & 0x100)
  width = (message & 0x200) << 8 | (message & 0x400)
  return (objectType, xPosition, yPosition, angle, length, width)

"""
Function to send a message to all clients connected
"""
def sendMessage(message, clientList):
  if message is not None:
    for client in clientList:
      try:
        client.send(message)
        # Need to update the print to print out the OBJECT_POS in the same format as convertToMessage
        print("Message Sent {}: {} - {}".format(client.address, VAL_TO_MSG[message.msgtype], message.msg))
      except (ConnectionResetError, BrokenPipeError):
        clientList.remove(client)
        print("Client Disconnected: {}".format(client.address))
        continue

"""
Function to change the coordinates
"""
def processDistance(distance, clientList):
  """
  global ROVER_X
  global ROVER_Y

  ROVER_X += int((distance * cos(radians(ROVER_ANGLE-90))))
  ROVER_Y += int((distance * sin(radians(ROVER_ANGLE-90))))
  """

  global TARGET_DISTANCE

  print('Location: ({}, {}), {} degrees'.format(ROVER_X, ROVER_Y, ROVER_ANGLE))

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

"""
Function to change the coordinates
"""
def processBackwardDistance(distance, clientList):
  """
  global ROVER_X
  global ROVER_Y

  ROVER_X += int((distance * cos(radians(ROVER_ANGLE+90))))
  ROVER_Y += int((distance * sin(radians(ROVER_ANGLE+90))))
  """

  print('Location: ({}, {}), {} degrees'.format(ROVER_X, ROVER_Y, ROVER_ANGLE))

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

"""
Function to change the angle of the rover. Most likely will be used for correction factoring
North = 0 degrees
"""
def turnLeft(angleDegrees, clientList):
  """
  global ROVER_ANGLE
  # Subtracts the degrees
  ROVER_ANGLE -= angleDegrees

  # Removes 360 degrees if a ful revolution is realized
  ROVER_ANGLE %= 360
  """

  print('Location: ({}, {}), {} degrees'.format(ROVER_X, ROVER_Y, ROVER_ANGLE))

  # Sends an update on the location of the rover/orientation
  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

"""
Function to change the angle of the rover. Most likely will be used for correction factoring
North = 0 degrees
"""
def turnRight(angleDegrees, clientList):
  global ROVER_ANGLE
  # Adds the degrees | Also eliminates the problem of negative angles by returning it to positive by
  # providing a full revolution of 360 degrees
  ROVER_ANGLE += (angleDegrees + 360)

  # Removes 360 degrees if a full revolution is realized
  ROVER_ANGLE %= 360

  print('Location: ({}, {}), {} degrees'.format(ROVER_X, ROVER_Y, ROVER_ANGLE))

  # Sends an update on the location of the rover/orientation
  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Used for the test cases to test that the PIC commands work properly
# The location of the rover and obstacles don't matter because we are only testing that the rover takes commands properly
# This will be used for all the rover test cases
def roverTestCase(clientList):
  global ROVER_X
  global ROVER_Y
  global ROVER_ANGLE

  # These are distances from the origin (upper left)
  # The rover is midway through the map
  ROVER_X = 18
  ROVER_Y = 18
  ROVER_ANGLE = 90

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Used for testing an obstacle in free space
def obstacleTestCase1(clientList):
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

  obstacleMessage = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x01, 0x06, 0x06)
  sendMessage(obstacleMessage, clientList)

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Used for testing an obstacle on the left side
def obstacleTestCase2(clientList):
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

  obstacleMessage = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x01, 0x06, 0x06)
  sendMessage(obstacleMessage, clientList)

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Used for testing an obstacle on the right side
def obstacleTestCase3(clientList):
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

  obstacleMessage = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x01, 0x06, 0x06)
  sendMessage(obstacleMessage, clientList)

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Used for testing an obstacle on the upper side
def obstacleTestCase4(clientList):
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

  obstacleMessage = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x01, 0x06, 0x06)
  sendMessage(obstacleMessage, clientList)

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Used for testing an obstacle on the lower side
def obstacleTestCase5(clientList):
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

  obstacleMessage = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x01, 0x06, 0x06)
  sendMessage(obstacleMessage, clientList)

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Test case for horizontally moving around an obstacle
def obstacleTestCase6(clientList):
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

  obstacleMessage = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x01, 0x06, 0x06)
  sendMessage(obstacleMessage, clientList)

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Test case for vertically moving around an obstacle
def obstacleTestCase7(clientList):
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

  obstacleMessage = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x01, 0x06, 0x06)
  sendMessage(obstacleMessage, clientList)

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Used for testing rover traversal in an empty map on the upper left corner
def traversalTestCase1(clientList):
  global ROVER_X
  global ROVER_Y
  global ROVER_ANGLE

  ROVER_X = 0
  ROVER_Y = 0
  ROVER_ANGLE = 270

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Used for testing rover traversal in an empty map on the lower left corner
def traversalTestCase2(clientList):
  global ROVER_X
  global ROVER_Y
  global ROVER_ANGLE

  ROVER_X = 0
  ROVER_Y = 33
  ROVER_ANGLE = 270

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Used for testing rover traversal in an empty map on the upper right corner
def traversalTestCase3(clientList):
  global ROVER_X
  global ROVER_Y
  global ROVER_ANGLE

  ROVER_X = 33
  ROVER_Y = 0
  ROVER_ANGLE = 90

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Used for testing rover traversal in an empty map on the lower right corner
def traversalTestCase4(clientList):
  global ROVER_X
  global ROVER_Y
  global ROVER_ANGLE

  ROVER_X = 36
  ROVER_Y = 36
  ROVER_ANGLE = 90

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Used for testing rover traversal in an empty map in the center
def traversalTestCase5(clientList):
  global ROVER_X
  global ROVER_Y
  global ROVER_ANGLE

  ROVER_X = 18
  ROVER_Y = 18
  ROVER_ANGLE = 0

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Used for testing rover traversal in an occupied map on the upper left corner
def traversalTestCase6(clientList):
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

  obstacleMessage1 = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage1, clientList)

  obstacleMessage2 = convertToMessage(OBSTACLE, OBSTACLE_2_X, OBSTACLE_2_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage2, clientList)

  obstacleMessage3 = convertToMessage(OBSTACLE, OBSTACLE_3_X, OBSTACLE_3_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage3, clientList)

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Used for testing rover traversal in an occupied map on the lower left corner
def traversalTestCase7(clientList):
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

  obstacleMessage1 = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage1, clientList)

  obstacleMessage2 = convertToMessage(OBSTACLE, OBSTACLE_2_X, OBSTACLE_2_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage2, clientList)

  obstacleMessage3 = convertToMessage(OBSTACLE, OBSTACLE_3_X, OBSTACLE_3_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage3, clientList)

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Used for testing rover traversal in an occupied map on the upper right corner
def traversalTestCase8(clientList):
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

  obstacleMessage1 = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage1, clientList)

  obstacleMessage2 = convertToMessage(OBSTACLE, OBSTACLE_2_X, OBSTACLE_2_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage2, clientList)

  obstacleMessage3 = convertToMessage(OBSTACLE, OBSTACLE_3_X, OBSTACLE_3_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage3, clientList)

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Used for testing rover traversal in an occupied map on the lower right corner
def traversalTestCase9(clientList):
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

  obstacleMessage1 = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage1, clientList)

  obstacleMessage2 = convertToMessage(OBSTACLE, OBSTACLE_2_X, OBSTACLE_2_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage2, clientList)

  obstacleMessage3 = convertToMessage(OBSTACLE, OBSTACLE_3_X, OBSTACLE_3_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage3, clientList)

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Used for testing rover traversal in an occupied map on the left side
def traversalTestCase10(clientList):
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

  obstacleMessage1 = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage1, clientList)

  obstacleMessage2 = convertToMessage(OBSTACLE, OBSTACLE_2_X, OBSTACLE_2_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage2, clientList)

  obstacleMessage3 = convertToMessage(OBSTACLE, OBSTACLE_3_X, OBSTACLE_3_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage3, clientList)

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Used for testing rover traversal in an occupied map on the right side
def traversalTestCase11(clientList):
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

  obstacleMessage1 = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage1, clientList)

  obstacleMessage2 = convertToMessage(OBSTACLE, OBSTACLE_2_X, OBSTACLE_2_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage2, clientList)

  obstacleMessage3 = convertToMessage(OBSTACLE, OBSTACLE_3_X, OBSTACLE_3_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage3, clientList)

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Used for testing rover traversal in an occupied map on the upper side
def traversalTestCase12(clientList):
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

  obstacleMessage1 = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage1, clientList)

  obstacleMessage2 = convertToMessage(OBSTACLE, OBSTACLE_2_X, OBSTACLE_2_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage2, clientList)

  obstacleMessage3 = convertToMessage(OBSTACLE, OBSTACLE_3_X, OBSTACLE_3_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage3, clientList)

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Used for testing rover traversal in an occupied map on the lower side
def traversalTestCase13(clientList):
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

  obstacleMessage1 = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage1, clientList)

  obstacleMessage2 = convertToMessage(OBSTACLE, OBSTACLE_2_X, OBSTACLE_2_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage2, clientList)

  obstacleMessage3 = convertToMessage(OBSTACLE, OBSTACLE_3_X, OBSTACLE_3_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage3, clientList)

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Used for testing rover traversal in an occupied map in the center
def traversalTestCase14(clientList):
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

  obstacleMessage1 = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage1, clientList)

  obstacleMessage2 = convertToMessage(OBSTACLE, OBSTACLE_2_X, OBSTACLE_2_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage2, clientList)

  obstacleMessage3 = convertToMessage(OBSTACLE, OBSTACLE_3_X, OBSTACLE_3_Y, 0x03, 0x06, 0x06)
  sendMessage(obstacleMessage3, clientList)

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

"""
Test cases for full traversal with multiple obstacles
"""

# Kyle's Test Case
def traversalTestCase15(clientList):
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

  # Sending out all the locations / counts
  obstacleMessage1 = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x04, 0x06, 0x06)
  sendMessage(obstacleMessage1, clientList)

  obstacleMessage2 = convertToMessage(OBSTACLE, OBSTACLE_2_X, OBSTACLE_2_Y, 0x04, 0x06, 0x06)
  sendMessage(obstacleMessage2, clientList)

  obstacleMessage3 = convertToMessage(OBSTACLE, OBSTACLE_3_X, OBSTACLE_3_Y, 0x04, 0x06, 0x06)
  sendMessage(obstacleMessage3, clientList)

  obstacleMessage4 = convertToMessage(OBSTACLE, OBSTACLE_4_X, OBSTACLE_4_Y, 0x04, 0x06, 0x06)
  sendMessage(obstacleMessage4, clientList)

  tokenMessage = convertToMessage(TOKEN, 0x00, 0x00, 0x04, 0x06, 0x06);
  sendMessage(tokenMessage, clientList)

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Test case that is similar to Kyle's test case but has one obstacle blocking the way
# to get to the upper left hand corner
def traversalTestCase16(clientList):
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

  # Sending out all the locations / counts
  obstacleMessage1 = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x04, 0x06, 0x06)
  sendMessage(obstacleMessage1, clientList)

  obstacleMessage2 = convertToMessage(OBSTACLE, OBSTACLE_2_X, OBSTACLE_2_Y, 0x04, 0x06, 0x06)
  sendMessage(obstacleMessage2, clientList)

  obstacleMessage3 = convertToMessage(OBSTACLE, OBSTACLE_3_X, OBSTACLE_3_Y, 0x04, 0x06, 0x06)
  sendMessage(obstacleMessage3, clientList)

  obstacleMessage4 = convertToMessage(OBSTACLE, OBSTACLE_4_X, OBSTACLE_4_Y, 0x04, 0x06, 0x06)
  sendMessage(obstacleMessage4, clientList)

  tokenMessage = convertToMessage(TOKEN, 0x00, 0x00, 0x04, 0x06, 0x06);
  sendMessage(tokenMessage, clientList)

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Test case for having two obstacles at both ends near the corners to test the ability to move around but also
# not miss a single blank space
def traversalTestCase17(clientList):
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

  # Sending out all the locations / counts
  obstacleMessage1 = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x04, 0x06, 0x06)
  sendMessage(obstacleMessage1, clientList)

  obstacleMessage2 = convertToMessage(OBSTACLE, OBSTACLE_2_X, OBSTACLE_2_Y, 0x04, 0x06, 0x06)
  sendMessage(obstacleMessage2, clientList)

  obstacleMessage3 = convertToMessage(OBSTACLE, OBSTACLE_3_X, OBSTACLE_3_Y, 0x04, 0x06, 0x06)
  sendMessage(obstacleMessage3, clientList)

  obstacleMessage4 = convertToMessage(OBSTACLE, OBSTACLE_4_X, OBSTACLE_4_Y, 0x04, 0x06, 0x06)
  sendMessage(obstacleMessage4, clientList)

  tokenMessage = convertToMessage(TOKEN, 0x00, 0x00, 0x04, 0x06, 0x06);
  sendMessage(tokenMessage, clientList)

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Test case for all obstacles in each side
def traversalTestCase18(clientList):
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

  TOKEN_4_X = 21
  TOKEN_4_Y = 33

  # Sending out all the locations / counts
  obstacleMessage1 = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x04, 0x06, 0x06)
  sendMessage(obstacleMessage1, clientList)

  obstacleMessage2 = convertToMessage(OBSTACLE, OBSTACLE_2_X, OBSTACLE_2_Y, 0x04, 0x06, 0x06)
  sendMessage(obstacleMessage2, clientList)

  obstacleMessage3 = convertToMessage(OBSTACLE, OBSTACLE_3_X, OBSTACLE_3_Y, 0x04, 0x06, 0x06)
  sendMessage(obstacleMessage3, clientList)

  obstacleMessage4 = convertToMessage(OBSTACLE, OBSTACLE_4_X, OBSTACLE_4_Y, 0x04, 0x06, 0x06)
  sendMessage(obstacleMessage4, clientList)

  tokenMessage = convertToMessage(TOKEN, 0x00, 0x00, 0x04, 0x06, 0x06);
  sendMessage(tokenMessage, clientList)

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

def timeBasedProcessDistance(clientList):
  global CURRENT_DISTANCE
  global ROVER_X
  global ROVER_Y

  # Break if I've already gone however far I needed to
  if CURRENT_DISTANCE >= TARGET_DISTANCE:
    CURRENT_DISTANCE = 0
    # Sending a message to update the rover location and signify a task is completed
    roverMessage = convertToMessage(ROVER, int(ROVER_X), int(ROVER_Y), int(ROVER_ANGLE), 0x06, 0x06)
    sendMessage(roverMessage, clientList)
    roverMessage = convertToMessage(LEAD_ROVER_UPDATE, 0x00, 0x00, 0x00, 0x06, 0x06)
    sendMessage(roverMessage, clientList)
    # Sending a message if the new location is on top of a token
    if (ROVER_X == TOKEN_1_X) and (ROVER_Y == TOKEN_1_Y):
      tokenMessage1 = InternalMessage(ROUTER, TOKEN_FOUND, bytes([0x00]))
      sendMessage(tokenMessage1, clientList)
    elif (ROVER_X == TOKEN_2_X) and (ROVER_Y == TOKEN_2_Y):
      tokenMessage2 = InternalMessage(ROUTER, TOKEN_FOUND, bytes([0x00]))
      sendMessage(tokenMessage2, clientList)
    elif (ROVER_X == TOKEN_3_X) and (ROVER_Y == TOKEN_3_Y):
      tokenMessage3 = InternalMessage(ROUTER, TOKEN_FOUND, bytes([0x00]))
      sendMessage(tokenMessage3, clientList)
    elif (ROVER_X == TOKEN_4_X) and (ROVER_Y == TOKEN_4_Y):
      tokenMessage4 = InternalMessage(ROUTER, TOKEN_FOUND, bytes([0x00]))
      sendMessage(tokenMessage4, clientList)
    return
  # Increment the current distance
  CURRENT_DISTANCE += (ROVER_SPEED * MEASUREMENT_PERIOD)

  # Increment the rover's actual position by that certain small amount
  ROVER_X += (ROVER_SPEED * cos(radians(ROVER_ANGLE - 90))) * MEASUREMENT_PERIOD
  ROVER_Y += (ROVER_SPEED * sin(radians(ROVER_ANGLE - 90))) * MEASUREMENT_PERIOD

  print('Location: ({}, {}), {} degrees'.format(ROVER_X, ROVER_Y, ROVER_ANGLE))

  roverMessage = convertToMessage(ROVER, int(ROVER_X), int(ROVER_Y), int(ROVER_ANGLE), 0x06, 0x06)
  sendMessage(roverMessage, clientList)

  # Timer to continually increment
  threading.Timer(MEASUREMENT_PERIOD, timeBasedProcessDistance, [listener.clientList]).start()

def timeBasedProcessLeftAngle(clientList):
  global CURRENT_DISTANCE
  global ROVER_ANGLE

  # Break if I've already gone however far I needed to
  if CURRENT_DISTANCE >= TARGET_DISTANCE:
    CURRENT_DISTANCE = 0
    roverMessage = convertToMessage(LEAD_ROVER_UPDATE, 0x00, 0x00, 0x00, 0x06, 0x06)
    sendMessage(roverMessage, clientList)
    return
  # Increment the current distance
  CURRENT_DISTANCE += (ROVER_ANGLE_SPEED * MEASUREMENT_PERIOD)

  # Subtracts the degrees
  ROVER_ANGLE -= (ROVER_ANGLE_SPEED * MEASUREMENT_PERIOD)
  # Removes 360 degrees if a ful revolution is realized
  ROVER_ANGLE %= 360

  print('Location: ({}, {}), {} degrees'.format(ROVER_X, ROVER_Y, ROVER_ANGLE))

  #roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  #sendMessage(roverMessage, clientList)

  # Timer to continually increment
  threading.Timer(MEASUREMENT_PERIOD, timeBasedProcessLeftAngle, [listener.clientList]).start()

def timeBasedProcessRightAngle(clientList):
  global CURRENT_DISTANCE
  global ROVER_ANGLE

  # Break if I've already gone however far I needed to
  if CURRENT_DISTANCE >= TARGET_DISTANCE:
    CURRENT_DISTANCE = 0
    roverMessage = convertToMessage(LEAD_ROVER_UPDATE, 0x00, 0x000, 0x00, 0x06, 0x06)
    sendMessage(roverMessage, clientList)
    return
  # Increment the current distance
  CURRENT_DISTANCE += (ROVER_ANGLE_SPEED * MEASUREMENT_PERIOD)

  # Adds the degrees | Also eliminates the problem of negative angles by returning it to positive by
  # providing a full revolution of 360 degrees
  ROVER_ANGLE += (ROVER_ANGLE_SPEED * MEASUREMENT_PERIOD) + 360
  # Removes 360 degrees if a full revolution is realized
  ROVER_ANGLE %= 360

  print('Location: ({}, {}), {} degrees'.format(ROVER_X, ROVER_Y, ROVER_ANGLE))

  #roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  #sendMessage(roverMessage, clientList)

  #Timer to continually increment
  threading.Timer(MEASUREMENT_PERIOD, timeBasedProcessRightAngle, [listener.clientList]).start()

if __name__ == "__main__":
  # Build the listener object to start communication with the PIC
  listener = Listener()
  #cmdThread = Thread(target=cmdInput, args=[listener.clientList], daemon=True)
  # Start the listener object to start listening for socket connections
  listener.start()
  #cmdThread.start()

  # Infinite loop where the message passing takes place
  while 1: #cmdThread.is_alive():
    try:
      # Attempt to take a message from the queue
      msg = listener.queue.get()

      # Here begins the message parsing to determine what needs to be done

      # Coming from the initialize, output that a client has been connected
      if msg.msgtype == CLIENT_ROLE:
        print("Message Received: {} - {}".format(VAL_TO_MSG[msg.msgtype], msg.msg))

        # Sending the initial data
        #roverTestCase(listener.clientList)

        # Sending rover test cases to traverse through an empty map
        #traversalTestCase1(listener.clientList)
        #traversalTestCase2(listener.clientList)
        #traversalTestCase3(listener.clientList)
        #traversalTestCase4(listener.clientList)
        #traversalTestCase5(listener.clientList)

        #obstacleTestCase1(listener.clientList)
        #obstacleTestCase6(listener.clientList)
        #obstacleTestCase7(listener.clientList)

        # Sending rover test cases to traverse through a nonempty map
        #traversalTestCase6(listener.clientList)
        #traversalTestCase14(listener.clientList)
        #traversalTestCase7(listener.clientList)

        """
        Test cases for obstacle aversion and map traversal
        """
        #traversalTestCase15(listener.clientList)
        #traversalTestCase16(listener.clientList)
        #traversalTestCase17(listener.clientList)
        traversalTestCase18(listener.clientList)

      # Interpreting rover commands
      # From the PIC comm.h file, the makeRoverMove function compacts the distance byte into byte position 1
      elif msg.msgtype == ROVER_MOVE:
          print("Message Received: {} - {}".format(VAL_TO_MSG[msg.msgtype], msg.msg))

          # Changes to the new target distance / angle
          TARGET_DISTANCE = msg.msg[1]

          # Rover moves forward
          if msg.msg[0] == ROVER_FORWARD:
            #processDistance(msg.msg[1], listener.clientList)
            # Processing the change over time rather than jumping from distance / angle
            # to the ideal distance / angle
            timeBasedProcessDistance(listener.clientList)

          # Rover moves backward
          elif msg.msg[0] == ROVER_BACKWARD:
            #processBackwardDistance(msg.msg[1], listener.clientList)
            # Processing the change over time rather than jumping from distance / angle
            # to the ideal distance / angle
            timeBasedProcessDistance(listener.clientList)

          # Turn the rover left
          elif msg.msg[0] == ROVER_LEFT:
            #turnLeft(msg.msg[1], listener.clientList)
            # Processing the change over time rather than jumping from distance / angle
            # to the ideal distance / angle
            timeBasedProcessLeftAngle(listener.clientList)

          # Turn the rover right
          elif msg.msg[0] == ROVER_RIGHT:
            #turnRight(msg.msg[1], listener.clientList)
            # Processing the change over time rather than jumping from distance / angle
            # to the ideal distance / angle
            timeBasedProcessRightAngle(listener.clientList)
      elif msg.msgtype == OBJECT_POS:
        print("Message Received: {} - {}".format(VAL_TO_MSG[msg.msgtype], msg.msg))
      elif msg.msgtype == TOKEN_FOUND:
        print("Message Received: {} - {}".format(VAL_TO_MSG[msg.msgtype], msg.msg))

    except Empty:
      pass
  listener.close()
