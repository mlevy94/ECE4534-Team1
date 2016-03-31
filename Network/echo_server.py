from listener import Listener
from configs import *
from threading import Thread
from queue import Empty
from math import cos,sin,radians

# Global variables for the rover coordinates
ROVER_X = 0
ROVER_Y = 0
ROVER_ANGLE = 0

# Global variables for the obstacle(s) coordinates
OBSTACLE_1_X = 0
OBSTACLE_1_Y = 0

OBSTACLE_2_X = 0
OBSTACLE_2_Y = 0

OBSTACLE_3_X = 0
OBSTACLE_3_Y = 0

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
  Distance in terms of degrees. Increments of 5 degrees.
  North = 0 degrees
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
  global ROVER_X
  global ROVER_Y

  ROVER_X += (distance * cos(radians(ROVER_ANGLE)))
  ROVER_Y += (distance * sin(radians(ROVER_ANGLE)))

  roverInit = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverInit, clientList)

"""
Function to change the coordinates
"""
def processBackwardDistance(distance):
  global ROVER_X
  global ROVER_Y

  ROVER_X += (distance * cos(-radians(ROVER_ANGLE)))
  ROVER_Y += (distance * sin(-radians(ROVER_ANGLE)))

"""
Function to change the angle of the rover. Most likely will be used for correction factoring
North = 0 degrees
"""
def turnLeft(angleDegrees):
  global ROVER_ANGLE
  # Subtracts the degrees
  ROVER_ANGLE -= angleDegrees

  # Removes 360 degrees if a ful revolution is realized
  ROVER_ANGLE %= 360

"""
Function to change the angle of the rover. Most likely will be used for correction factoring
North = 0 degrees
"""
def turnRight(angleDegrees):
  global ROVER_ANGLE
  # Adds the degrees | Also eliminates the problem of negative angles by returning it to positive by
  # providing a full revolution of 360 degrees
  ROVER_ANGLE += (angleDegrees + 360)

  # Removes 360 degrees if a full revolution is realized
  ROVER_ANGLE %= 360

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

  ROVER_X = 0
  ROVER_Y = 36
  ROVER_ANGLE = 0

  OBSTACLE_1_X = 18
  OBSTACLE_1_Y = 18

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

  obstacleMessage = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage, clientList)

# Used for testing an obstacle on the left side
def obstacleTestCase2(clientList):
  global ROVER_X
  global ROVER_Y
  global ROVER_ANGLE
  global OBSTACLE_1_X
  global OBSTACLE_1_Y

  ROVER_X = 18
  ROVER_Y = 18
  ROVER_ANGLE = 270

  OBSTACLE_1_X = 0
  OBSTACLE_1_Y = 18

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

  obstacleMessage = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage, clientList)

# Used for testing an obstacle on the right side
def obstacleTestCase3(clientList):
  global ROVER_X
  global ROVER_Y
  global ROVER_ANGLE
  global OBSTACLE_1_X
  global OBSTACLE_1_Y

  ROVER_X = 18
  ROVER_Y = 18
  ROVER_ANGLE = 270

  OBSTACLE_1_X = 36
  OBSTACLE_1_Y = 18

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

  obstacleMessage = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage, clientList)

# Used for testing an obstacle on the upper side
def obstacleTestCase4(clientList):
  global ROVER_X
  global ROVER_Y
  global ROVER_ANGLE
  global OBSTACLE_1_X
  global OBSTACLE_1_Y

  ROVER_X = 18
  ROVER_Y = 18
  ROVER_ANGLE = 270

  OBSTACLE_1_X = 18
  OBSTACLE_1_Y = 0

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

  obstacleMessage = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage, clientList)

# Used for testing an obstacle on the lower side
def obstacleTestCase5(clientList):
  global ROVER_X
  global ROVER_Y
  global ROVER_ANGLE
  global OBSTACLE_1_X
  global OBSTACLE_1_Y

  ROVER_X = 18
  ROVER_Y = 18
  ROVER_ANGLE = 270

  OBSTACLE_1_X = 18
  OBSTACLE_1_Y = 36

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

  obstacleMessage = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage, clientList)

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
  ROVER_Y = 36
  ROVER_ANGLE = 270

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

# Used for testing rover traversal in an empty map on the upper right corner
def traversalTestCase3(clientList):
  global ROVER_X
  global ROVER_Y
  global ROVER_ANGLE

  ROVER_X = 36
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

  ROVER_X = 0
  ROVER_Y = 0
  ROVER_ANGLE = 0

  OBSTACLE_1_X = 0
  OBSTACLE_1_Y = 18

  OBSTACLE_2_X = 18
  OBSTACLE_2_Y = 18

  OBSTACLE_3_X = 18
  OBSTACLE_3_Y = 36

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

  obstacleMessage1 = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage1, clientList)

  obstacleMessage2 = convertToMessage(OBSTACLE, OBSTACLE_2_X, OBSTACLE_2_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage2, clientList)

  obstacleMessage3 = convertToMessage(OBSTACLE, OBSTACLE_3_X, OBSTACLE_3_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage3, clientList)

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

  ROVER_X = 0
  ROVER_Y = 36
  ROVER_ANGLE = 180

  OBSTACLE_1_X = 0
  OBSTACLE_1_Y = 18

  OBSTACLE_2_X = 18
  OBSTACLE_2_Y = 18

  OBSTACLE_3_X = 18
  OBSTACLE_3_Y = 36

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

  obstacleMessage1 = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage1, clientList)

  obstacleMessage2 = convertToMessage(OBSTACLE, OBSTACLE_2_X, OBSTACLE_2_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage2, clientList)

  obstacleMessage3 = convertToMessage(OBSTACLE, OBSTACLE_3_X, OBSTACLE_3_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage3, clientList)

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

  ROVER_X = 36
  ROVER_Y = 0
  ROVER_ANGLE = 0

  OBSTACLE_1_X = 0
  OBSTACLE_1_Y = 18

  OBSTACLE_2_X = 18
  OBSTACLE_2_Y = 18

  OBSTACLE_3_X = 18
  OBSTACLE_3_Y = 36

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

  obstacleMessage1 = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage1, clientList)

  obstacleMessage2 = convertToMessage(OBSTACLE, OBSTACLE_2_X, OBSTACLE_2_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage2, clientList)

  obstacleMessage3 = convertToMessage(OBSTACLE, OBSTACLE_3_X, OBSTACLE_3_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage3, clientList)

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

  ROVER_X = 36
  ROVER_Y = 36
  ROVER_ANGLE = 180

  OBSTACLE_1_X = 0
  OBSTACLE_1_Y = 18

  OBSTACLE_2_X = 18
  OBSTACLE_2_Y = 18

  OBSTACLE_3_X = 18
  OBSTACLE_3_Y = 36

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

  obstacleMessage1 = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage1, clientList)

  obstacleMessage2 = convertToMessage(OBSTACLE, OBSTACLE_2_X, OBSTACLE_2_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage2, clientList)

  obstacleMessage3 = convertToMessage(OBSTACLE, OBSTACLE_3_X, OBSTACLE_3_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage3, clientList)

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

  ROVER_X = 0
  ROVER_Y = 18
  ROVER_ANGLE = 270

  OBSTACLE_1_X = 36
  OBSTACLE_1_Y = 18

  OBSTACLE_2_X = 18
  OBSTACLE_2_Y = 18

  OBSTACLE_3_X = 18
  OBSTACLE_3_Y = 36

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

  obstacleMessage1 = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage1, clientList)

  obstacleMessage2 = convertToMessage(OBSTACLE, OBSTACLE_2_X, OBSTACLE_2_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage2, clientList)

  obstacleMessage3 = convertToMessage(OBSTACLE, OBSTACLE_3_X, OBSTACLE_3_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage3, clientList)

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

  ROVER_X = 36
  ROVER_Y = 18
  ROVER_ANGLE = 90

  OBSTACLE_1_X = 0
  OBSTACLE_1_Y = 18

  OBSTACLE_2_X = 18
  OBSTACLE_2_Y = 18

  OBSTACLE_3_X = 18
  OBSTACLE_3_Y = 36

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

  obstacleMessage1 = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage1, clientList)

  obstacleMessage2 = convertToMessage(OBSTACLE, OBSTACLE_2_X, OBSTACLE_2_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage2, clientList)

  obstacleMessage3 = convertToMessage(OBSTACLE, OBSTACLE_3_X, OBSTACLE_3_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage3, clientList)

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

  ROVER_X = 18
  ROVER_Y = 0
  ROVER_ANGLE = 0

  OBSTACLE_1_X = 0
  OBSTACLE_1_Y = 18

  OBSTACLE_2_X = 18
  OBSTACLE_2_Y = 18

  OBSTACLE_3_X = 18
  OBSTACLE_3_Y = 36

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

  obstacleMessage1 = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage1, clientList)

  obstacleMessage2 = convertToMessage(OBSTACLE, OBSTACLE_2_X, OBSTACLE_2_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage2, clientList)

  obstacleMessage3 = convertToMessage(OBSTACLE, OBSTACLE_3_X, OBSTACLE_3_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage3, clientList)

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

  ROVER_X = 18
  ROVER_Y = 36
  ROVER_ANGLE = 180


  OBSTACLE_1_X = 0
  OBSTACLE_1_Y = 18

  OBSTACLE_2_X = 18
  OBSTACLE_2_Y = 18

  OBSTACLE_3_X = 18
  OBSTACLE_3_Y = 0

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

  obstacleMessage1 = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage1, clientList)

  obstacleMessage2 = convertToMessage(OBSTACLE, OBSTACLE_2_X, OBSTACLE_2_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage2, clientList)

  obstacleMessage3 = convertToMessage(OBSTACLE, OBSTACLE_3_X, OBSTACLE_3_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage3, clientList)

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

  ROVER_X = 18
  ROVER_Y = 18
  ROVER_ANGLE = 0

  OBSTACLE_1_X = 0
  OBSTACLE_1_Y = 18

  OBSTACLE_2_X = 24
  OBSTACLE_2_Y = 18

  OBSTACLE_3_X = 18
  OBSTACLE_3_Y = 36

  roverMessage = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x06, 0x06)
  sendMessage(roverMessage, clientList)

  obstacleMessage1 = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage1, clientList)

  obstacleMessage2 = convertToMessage(OBSTACLE, OBSTACLE_2_X, OBSTACLE_2_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage2, clientList)

  obstacleMessage3 = convertToMessage(OBSTACLE, OBSTACLE_3_X, OBSTACLE_3_Y, 0x00, 0x06, 0x06)
  sendMessage(obstacleMessage3, clientList)

"""
Function taken from leadrovertest.py for a thread that can send using the I/O port for sending messages.
def cmdInput(clientList):
  try:
    keyin = ""
    while keyin != "shutdown":
      keyin = input().lower()

      if msg is not None:
        for client in clientList:
          print("Message Sent {}: {} - {}".format(client.address, VAL_TO_MSG[msg.msgtype], msg.msg))
          try:
            client.send(msg)
          except (ConnectionResetError, BrokenPipeError):
            clientList.remove(client)
            print("Client Disconnected: {}".format(client.address))
  except KeyboardInterrupt:
    pass
"""
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
        roverTestCase(listener.clientList)
        """
        roverInit1 = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x04, 0x04)
        sendMessage(roverInit1, listener.clientList)

        obstacleInit1 = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x00, 0x04, 0x04)
        sendMessage(obstacleInit1, listener.clientList)

        for client in listener.clientList:
            x = InternalMessage(ROUTER, OBJECT_POS, bytes(0x01))
            print("Message Sent {}: {} - {}".format(client.address, VAL_TO_MSG[x.msgtype], x.msg))
            try:
              roverInit = convertToMessage(ROVER, ROVER_X, ROVER_Y, ROVER_ANGLE, 0x04, 0x04)
              client.send(roverInit)

              obstacleInit = convertToMessage(OBSTACLE, OBSTACLE_1_X, OBSTACLE_1_Y, 0x00, 0x04, 0x04)
              client.send(obstacleInit)
            except (ConnectionResetError, BrokenPipeError):
              listener.clientList.remove(client)
              print("Client Disconnected: {}".format(client.address))
        """

      # Interpreting rover commands
      # From the PIC comm.h file, the makeRoverMove function compacts the distance byte into byte position 1
      elif msg.msgtype == ROVER_MOVE:
          print(msg.msg)

          # Rover moves forward
          if msg.msg[0] == ROVER_FORWARD:
            processDistance(msg.msg[1])

          # Rover moves backward
          elif msg.msg[0] == ROVER_BACKWARD:
            processBackwardDistance(msg.msg[1])

          # Turn the rover left
          elif msg.msg[0] == ROVER_LEFT:
            turnLeft(msg.msg[1])
            print(msg.msg)

          # Turn the rover right
          elif msg.msg[0] == ROVER_RIGHT:
            turnRight(msg.msg[1])
            print(msg.msg)
      elif msg.msgtype == OBJECT_POS:
        print("Message Received: {} - {}".format(VAL_TO_MSG[msg.msgtype], msg.msg))

    except Empty:
      pass
  listener.close()
