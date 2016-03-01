# This file contains the configurations neccessary to operate this in different environments.
from socket import gethostbyname, gethostname
from collections import OrderedDict


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

# value to role conversion
VAL_TO_ROLE = OrderedDict((
  (CLIENT, "CLIENT"),
  (LEAD_ROVER, "LEAD_ROVER"),
  (FOLLOWER, "FOLLOWER"),
  (SENSOR, "SENSOR"),
  (COORDINATOR, "COORDINATOR"),
  (MONITOR, "MONITOR"),
  (ROUTER, "ROUTER"),
))

# role to value conversion
ROLE_TO_VAL = OrderedDict((
  ("CLIENT", CLIENT),
  ("LEAD_ROVER", LEAD_ROVER),
  ("FOLLOWER", FOLLOWER),
  ("SENSOR", SENSOR),
  ("COORDINATOR", COORDINATOR),
  ("MONITOR", MONITOR),
  ("ROUTER", ROUTER),
))

########## MESSAGE TYPES #################

# type values
DEBUG_MSG            = 0x01
NET_STAT             = 0x02
CLIENT_ROLE          = 0x04
INITIALIZE           = 0x08
READY_TO_START       = 0x10
MOTOR_MOVE           = 0x11
OBJECT_POS           = 0x14
TOKEN_FOUND          = 0x18

# value to message conversion
VAL_TO_MSG = OrderedDict((
  (DEBUG_MSG, "Debug Message"),
  (NET_STAT, "Network Statistic"),
  (CLIENT_ROLE, "Client Role"),
  (INITIALIZE, "Initialize"),
  (READY_TO_START, "Ready to Start"),
  (MOTOR_MOVE, "Moter Move"),
  (OBJECT_POS, "Object Position"),
  (TOKEN_FOUND, "Token Found"),
))

########## MESSAGE SPECIFIC VALUES ########
MAX_MSG_COUNT = 255
HEADER_SIZE = 5
TAIL_SIZE = 1
INTERNAL_MSG_SIZE = 12
NET_MSG_SIZE = HEADER_SIZE + TAIL_SIZE + INTERNAL_MSG_SIZE


########## MESSAGE STRUCTURES #############
class InternalMessage:

  def __init__(self, client, msgtype, msg, target=[]):
    self.client = bytetoval(client)
    self.msgtype = bytetoval(msgtype)
    self.msg = msg
    if not isinstance(target, (list, tuple)):
      self.target = [target]
    else:
      self.target = target


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
