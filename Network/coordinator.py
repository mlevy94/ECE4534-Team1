from configs import *  # too many variables to import explicitly
from testclient import Client

TARGET_IP = getIPAddr()
TARGET_PORT = getPort()
SELF_ROLE = COORDINATOR

def roverMove(direction, distance):
  return InternalMessage(CLIENT, ROVER_MOVE, bytes([direction, distance]))






if __name__ == "__main__":
  client = Client(TARGET_IP, TARGET_PORT, SELF_ROLE)
  client.connect()

  while 1:
    msg = client.get()
    print("Message Received: {} - {}".format(VAL_TO_MSG[msg.msgtype], msg.msg))
