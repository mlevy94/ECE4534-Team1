from configs import *  # too many variables to import explicitly
from testclient import Client
from collections import Counter

TARGET_IP = getIPAddr()
TARGET_PORT = getPort()
MY_ROLE = COORDINATOR

def roverMove(direction, distance):
  return InternalMessage(CLIENT, ROVER_MOVE, bytes([direction, distance]))

def decipherMessage(message):
  objectType = message[0]
  xPosition = ((message[1] << 8) | message[2]) / 10.0
  yPosition = ((message[3] << 8) | message[4]) / 10.0
  angle = (message[5] << 8) | message[6]
  length = ((message[7] << 8) | message[8]) / 10.0
  width = ((message[9] << 8) | message[10]) / 10.0

  return objectType, xPosition, yPosition, angle, length, width

class Rover:

  xMax = 36
  yMax = 36
  xSq = 6
  ySq = 6

  def __init__(self):
    self.xposlist = Counter()
    self.yposlist = Counter()
    self.anglelist = Counter()
    self.roverFace = 0

  def addPos(self, xpos, ypos, angle):
    self.xposlist[xpos] += 1
    self.yposlist[ypos] += 1
    self.anglelist[angle] += 1

  def adjust(self):
    # get useful value
    x = self.xposlist.most_common()[0][0]
    y = self.yposlist.most_common()[0][0]
    a = self.anglelist.most_common()[0][0]

    # get adjustment values
    xadj = round((self.xSq / 2) - (x % self.xSq), 1)
    yadj = round((self.ySq / 2) - (y % self.ySq), 1)
    self.roverFace = round(a / 90) * 90
    aadj = self.roverFace - a


    # clear out lists for next time
    self.xposlist.clear()
    self.yposlist.clear()
    self.anglelist.clear()

    return xadj, yadj, aadj




if __name__ == "__main__":
  client = Client(TARGET_IP, TARGET_PORT, MY_ROLE)
  client.connect()
  rover = Rover()
  counter = 1

  while 1:
    msg = client.get()
    if msg.msgtype == OBJECT_POS:
      obj = decipherMessage(msg.msg)
      if obj[0] == 163:
        print("Message Received: {} - Object: {} xPos: {}, yPos: {}, angle: {}, length: {}, width: {}".format(VAL_TO_MSG[msg.msgtype], *obj))
        rover.addPos(obj[1], obj[2], obj[3])
        if counter % 10 == 0:
          xadj, yadj, aadj = rover.adjust()
          print("Rover Adjust: xPos: {} yPos: {} angle: {}".format(xadj, yadj, aadj))
        counter += 1
    else:
      print("Message Received: {} - {}".format(VAL_TO_MSG[msg.msgtype], msg.msg))
