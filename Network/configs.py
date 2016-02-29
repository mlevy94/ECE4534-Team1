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
BAD_MSG              = 0x02
MSG_REQUEST          = 0x04
CLIENT_ROLE          = 0x08
INITIALIZE           = 0x10
READY_TO_START       = 0x11
LEAD_PO              = 0x12
FOLLOW_PO            = 0x14
OBS_INFO             = 0x18
MOTOR_MOVE           = 0x20
TOKEN_FOUND          = 0x21

# value to message conversion
VAL_TO_MSG = OrderedDict((
  (DEBUG_MSG, "Debug Message"),
  (BAD_MSG, "Bad Message"),
  (MSG_REQUEST, "Message Request"),
  (CLIENT_ROLE, "Client Role"),
  (INITIALIZE, "Initialize"),
  (READY_TO_START, "Ready to Start"),
  (LEAD_PO, "LEAD_PO"),
  (FOLLOW_PO, "FOLLOW_PO"),
  (OBS_INFO, "OBS_INFO"),
  (MOTOR_MOVE, "Moter Move"),
  (TOKEN_FOUND, "Token Found"),
))

########## MESSAGE SPECIFIC VALUES ########
MAX_MSG_COUNT = 128
HEADER_SIZE = 5
TAIL_SIZE = 3
INTERNAL_MSG_SIZE = 12
NET_MSG_SIZE = HEADER_SIZE + TAIL_SIZE + INTERNAL_MSG_SIZE


########## MESSAGE STRUCTURES #############
class InternalMessage:

  def __init__(self, client, msgtype, msg, target=None):
    self.client = bytetoval(client)
    self.msgtype = bytetoval(msgtype)
    self.msg = msg
    self.target = target


class NetMessage:

  def __init__(self, source, count, msgtype, msgsize, msg, checksum=None):
    # store values and make sure they are all converted to numbers from bytes
    self.source = bytetoval(source)
    self.count = bytetoval(count)
    self.msgtype = bytetoval(msgtype)
    self.msgsize = bytetoval(msgsize)
    self.msg = msg
    if checksum is None:
      self.checksum = self.makeChecksum()
    elif isinstance(checksum, bytes):
      self.checksum = (checksum[0] << 8) + checksum[1]
    else:
      self.checksum = checksum

  def makeChecksum(self):
    checksum = self.source + self.count + self.msgtype + self.msgsize
    for byte in self.msg:
      checksum += byte
    return checksum

  def checkMessage(self):
    checksum = self.makeChecksum()
    return self.checksum == checksum and \
           self.msgsize == len(self.msg)

  def getMessage(self):
    return InternalMessage(self.source, self.msgtype, self.msg)

  def serialize(self):
    # convert back to bytes
    source = bytes([self.source])
    count = bytes([self.count])
    msgtype = bytes([self.msgtype])
    msgsize = bytes([self.msgsize])
    checksum = bytes([self.checksum >> 8 & 0xff]) + bytes([self.checksum & 0xff])

    return STARTBYTE + source + count + msgtype + msgsize + self.msg + checksum + ENDBYTE

def bytetoval(val):
    if val == b'':
      return 0
    if isinstance(val, bytes):
      return val[0]
    else:
      return val
