# This file will contain the basic configurations and message types in order to provide proper communication

"""
Modified version of Michael's config.py code
"""

# Start and end bytes to the message frame
START_BYTE = b'\x00'
END_BYTE = b'\xFF'

############# Client Roles #############

# Roles
CLIENT              = 0x01
LEAD_ROVER          = 0x02
FOLLOWER_ROVER      = 0x04
SENSOR              = 0x08
COORDINATOR         = 0x10
MONITOR             = 0x11
ROUTER              = 0x12
VISUALIZATION       = 0x14

# Role conversions
VAL_TO_ROLE = {}
VAL_TO_ROLE[CLIENT] = "CLIENT"
VAL_TO_ROLE[LEAD_ROVER] = "LEAD ROVER"
VAL_TO_ROLE[FOLLOWER_ROVER] = "FOLLOWER ROVER"
VAL_TO_ROLE[SENSOR] = "SENSOR"
VAL_TO_ROLE[COORDINATOR] = "COORDINATOR"
VAL_TO_ROLE[MONITOR] = "MONITOR"
VAL_TO_ROLE[ROUTER] = "ROUTER"
VAL_TO_ROLE[VISUALIZATION] = "VISUALIZATION"

# Role conversions
ROLE_TO_VAL = {}
ROLE_TO_VAL["CLIENT"] = CLIENT
ROLE_TO_VAL["LEAD ROVER"] = LEAD_ROVER
ROLE_TO_VAL["FOLLOWER ROVER"] = FOLLOWER_ROVER
ROLE_TO_VAL["SENSOR"] = SENSOR
ROLE_TO_VAL["COORDINATOR"] = COORDINATOR
ROLE_TO_VAL["MONITOR"] = MONITOR
ROLE_TO_VAL["ROUTER"] = ROUTER
ROLE_TO_VAL["VISUALIZATION"] = VISUALIZATION

########## MESSAGE TYPES #################

# type values
DEBUG_MSG         = 0x01
NET_STAT          = 0x02
CLIENT_ROLE       = 0x04
INITIALIZE        = 0x08
READY_TO_START    = 0x10
ROVER_MOVE        = 0x11
OBJECT_POS        = 0x14
TOKEN_FOUND       = 0x18
TASK_COMPLETED    = 0x20
ALGORITHM_TIME    = 0x21

# OBJECT_STRUCTURE types
ROVER             = 0xA0
OBSTACLE          = 0xA1
LEAD_ROVER_UPDATE = 0xA2
TOKEN             = 0xA3

# Message type conversions
VAL_TO_MSG = {}
VAL_TO_MSG[DEBUG_MSG] = "Debug Message"
VAL_TO_MSG[NET_STAT] = "Network Statistic"
VAL_TO_MSG[CLIENT_ROLE] = "Client Role"
VAL_TO_MSG[INITIALIZE] = "Initialize"
VAL_TO_MSG[READY_TO_START] = "Ready to Start"
VAL_TO_MSG[ROVER_MOVE] = "Rover Move"
VAL_TO_MSG[OBJECT_POS] = "Object Position"
VAL_TO_MSG[TOKEN_FOUND] = "Token Found"
VAL_TO_MSG[TASK_COMPLETED] = "Task Completed - Debug"
VAL_TO_MSG[ALGORITHM_TIME] = "Algorithm time"

# Object type conversions
MSG_TO_OBJECT = {}
MSG_TO_OBJECT[ROVER] = "Rover"
MSG_TO_OBJECT[OBSTACLE] = "Obstacle"
MSG_TO_OBJECT[LEAD_ROVER_UPDATE] = "Lead rover has completed reaching its destination"
MSG_TO_OBJECT[TOKEN] = "Token"

########## ROVER MOVE DEFINES #############

ROVER_FORWARD       = 0x01
ROVER_BACKWARD      = 0x02
ROVER_LEFT          = 0x04
ROVER_RIGHT         = 0x08
ROVER_STOP          = 0x10

# Rover commands conversion
MSG_TO_ROVERMOVE = {}
MSG_TO_ROVERMOVE[ROVER_FORWARD] = "Move rover forward"
MSG_TO_ROVERMOVE[ROVER_BACKWARD] = "Move rover backward"
MSG_TO_ROVERMOVE[ROVER_LEFT] = "Turn rover left"
MSG_TO_ROVERMOVE[ROVER_RIGHT] = "Turn rover right"
MSG_TO_ROVERMOVE[ROVER_STOP] = "Make rover stop"

########## MESSAGE SPECIFIC VALUES ########
MAX_MSG_COUNT = 255
HEADER_SIZE = 5
TAIL_SIZE = 1
INTERNAL_MSG_SIZE = 12
NET_MSG_SIZE = HEADER_SIZE + TAIL_SIZE + INTERNAL_MSG_SIZE

########## MESSAGE STRUCTURES #############
class InternalMessage:
  def __init__(self, client, msgtype, msg):
    self.client = client
    self.msgtype = msgtype
    self.msg = msg