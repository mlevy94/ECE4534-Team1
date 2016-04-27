# This file contains the configurations neccessary to operate this in different environments.
from socket import gethostbyname, gethostname
from collections import OrderedDict

DEBUG_ON = False

def getIPAddr():
  return gethostbyname(gethostname())

def getPort():
  return 56677

# sending characters
STARTBYTE = b'\x00'
ENDBYTE = b'\xff'

############# CLIENT ROLES #################

# role values
CLIENT         = 0x01
LEAD_ROVER     = 0x02
FOLLOWER       = 0x04
SENSOR         = 0x08
COORDINATOR    = 0x10
MONITOR        = 0x11
ROUTER         = 0x12
GUI            = 0x14

# value to role conversion
VAL_TO_ROLE = OrderedDict((
  (CLIENT, "CLIENT"),
  (LEAD_ROVER, "LEAD ROVER"),
  (FOLLOWER, "FOLLOWER"),
  (SENSOR, "SENSOR"),
  (COORDINATOR, "COORDINATOR"),
  (MONITOR, "MONITOR"),
  (ROUTER, "ROUTER"),
))

# role to value conversion
ROLE_TO_VAL = OrderedDict((
  ("CLIENT", CLIENT),
  ("LEAD ROVER", LEAD_ROVER),
  ("FOLLOWER", FOLLOWER),
  ("SENSOR", SENSOR),
  ("COORDINATOR", COORDINATOR),
  ("MONITOR", MONITOR),
  ("ROUTER", ROUTER),
))

########## MESSAGE TYPES #################

# type values
DEBUG_MSG            = 0x01
CLIENT_ROLE          = 0x04
INITIALIZE           = 0x08
ROVER_MOVE           = 0x11
OBJECT_POS           = 0x14
TOKEN_FOUND          = 0x18

SENSOR_MODE          = 0x61
CALIBRATE_ROVER      = 0x62
## RESERVED 0x19-0x22
PONG                 = 0x69
PING                 = 0x70
END_GAME             = 0x71
START_GAME           = 0x72
## RESERVED 0x73-0x79

# value to message conversion
VAL_TO_MSG = OrderedDict((
  (DEBUG_MSG, "Debug Message"),
  (CLIENT_ROLE, "Client Role"),
  (INITIALIZE, "Initialize"),
  (ROVER_MOVE, "Rover Move"),
  (OBJECT_POS, "Object Position"),
  (TOKEN_FOUND, "Token Found"),
  (START_GAME, "Start Game"),
  (END_GAME, "End Game"),
  (SENSOR_MODE, "Sensor Mode"),
  (CALIBRATE_ROVER, "Calibrate Rover"),
  (PING, "Ping"),
  (PONG, "Pong"),
))

ROLE_MSG_RECV = OrderedDict((
  (LEAD_ROVER, [ROVER_MOVE, PING, ]),
  (FOLLOWER, []),
  (SENSOR, [PING, ]),
  (COORDINATOR, [ROVER_MOVE, OBJECT_POS, PING, START_GAME, TOKEN_FOUND, ]),
  (MONITOR, [DEBUG_MSG, OBJECT_POS, ROVER_MOVE, TOKEN_FOUND, PONG, START_GAME, END_GAME, ]),
))

########## ROVER MOVE DEFINES #############

ROVER_FORWARD       = 0x01
ROVER_BACKWARD      = 0x02
ROVER_LEFT          = 0x04
ROVER_RIGHT         = 0x08
ROVER_STOP          = 0x10

VAL_TO_ROV = OrderedDict((
  (ROVER_FORWARD, "Rover Forward"),
  (ROVER_BACKWARD, "Rover Backward"),
  (ROVER_LEFT, "Rover Left"),
  (ROVER_RIGHT, "Rover Right"),
  (ROVER_STOP, "Rover Stop"),
))

########## MESSAGE SPECIFIC VALUES ########
MAX_MSG_COUNT = 255
HEADER_SIZE = 5
TAIL_SIZE = 1
INTERNAL_MSG_SIZE = 12
NET_MSG_SIZE = HEADER_SIZE + TAIL_SIZE + INTERNAL_MSG_SIZE


########## MESSAGE STRUCTURES #############
class InternalMessage:

  def __init__(self, client, msgtype, msg, target=None):
    if target is None:
      target = []
    self.client = bytetoval(client)
    self.msgtype = bytetoval(msgtype)
    self.msg = msg
    if not isinstance(target, (list, tuple)):
      self.target = [target]
    else:
      self.target = target

  def __repr__(self):
    if self.msgtype == ROVER_MOVE:
      return "<{}, {}: {}, {}>".format(VAL_TO_MSG[self.msgtype], VAL_TO_ROLE[self.client], VAL_TO_ROV[self.msg[0]], self.msg[1])
    elif self.msgtype == OBJECT_POS:
      return "<{}, {} - Object: {} xPos: {}, yPos: {}, angle: {}, length: {}, width: {}>".format(VAL_TO_MSG[self.msgtype], VAL_TO_ROLE[self.client], *decipherMessage(self.msg))
    else:
      return "<{}, {}: {}>".format(VAL_TO_MSG[self.msgtype], VAL_TO_ROLE[self.client], self.msg)


class NetMessage:

  def __init__(self, source, count, msgtype, msgsize, msg):
    # store values and make sure they are all converted to numbers from bytes
    self.source = bytetoval(source)
    self.count = bytetoval(count)
    self.msgtype = bytetoval(msgtype)
    self.msgsize = bytetoval(msgsize)
    self.msg = msg

  def getMessage(self):
    return InternalMessage(self.source, self.msgtype, self.msg)

  def serialize(self):
    # convert back to bytes
    source = bytes([self.source])
    count = bytes([self.count])
    msgtype = bytes([self.msgtype])
    msgsize = bytes([self.msgsize])

    return STARTBYTE + source + count + msgtype + msgsize + self.msg + ENDBYTE

def bytetoval(val):
    if val == b'':
      return 0
    if isinstance(val, bytes):
      return val[0]
    else:
      return val

# objectType is defined in the message_types
def makeObjPosMsg(objectType, x, y, orientation):
  str = bytes([objectType])
  str += bytes([x >> 8 & 0xff, x & 0xff])
  str += bytes([y >> 8 & 0xff, y & 0xff])
  str += bytes([orientation])
  return InternalMessage(CLIENT, OBJECT_POS, str)

def roverMove(direction, distance, client=CLIENT):
  return InternalMessage(client, ROVER_MOVE, bytes([direction, distance]))

def decipherMessage(message):
  objectType = message[0]
  xPosition = ((message[1] << 8) | message[2]) / 10.0
  yPosition = ((message[3] << 8) | message[4]) / 10.0
  angle = (message[5] << 8) | message[6]
  length = ((message[7] << 8) | message[8]) / 10.0
  width = ((message[9] << 8) | message[10]) / 10.0

  return objectType, xPosition, yPosition, angle, length, width

class peekIter:

  def __init__(self, oglist):
    self.oglist = oglist
    self.idx = 0

  def generate(self):
    self.idx = 0
    for ele in self.oglist:
      self.idx += 1
      yield ele
    raise StopIteration

  def peek(self):
    try:
      return self.oglist[self.idx]
    except IndexError:
      return None
