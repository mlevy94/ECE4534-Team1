from configs import *  # too many variables to import explicitly
from testclient import Client
from collections import Counter
from threading import Thread
from queue import Queue
from time import sleep
from integrationtests import test1, test2, test3, test4, test5, test6, test7


TARGET_IP = getIPAddr()
TARGET_PORT = getPort()
MY_ROLE = COORDINATOR



class Rover:
  angle_buff = 15
  dist_buff = 2
  xSq = 6
  ySq = 6

  def __init__(self, client):
    self.client = client
    self.xposlist = Counter()
    self.yposlist = Counter()
    self.anglelist = Counter()
    self.sequence = []
    self.queue = Queue()
    self.runThread = None

  def addPos(self, xpos, ypos, angle):
    self.xposlist[xpos] += 1
    self.yposlist[ypos] += 1
    self.anglelist[angle] += 1

  def clearPos(self):
    # clear out lists for next time
    self.xposlist.clear()
    self.yposlist.clear()
    self.anglelist.clear()

  def adjust(self):
    adjlist = []
    # get useful values then clear them
    x = self.xposlist.most_common()[0][0]
    y = self.yposlist.most_common()[0][0]
    a = self.anglelist.most_common()[0][0]
    self.clearPos()

    # get adjustment values
    xadj = round((self.xSq / 2) - (x % self.xSq), 1)
    yadj = round((self.ySq / 2) - (y % self.ySq), 1)
    roverFace = round(a / 90) * 90
    aadj = round((roverFace - a / 5)) * 5

    # large angle adjust
    if abs(aadj) >= self.angle_buff:
      if aadj < 0:
        adjlist.append(roverMove(ROVER_LEFT, abs(aadj)))
      else:
        adjlist.append(roverMove(ROVER_RIGHT, aadj))

    if roverFace == 0 or roverFace == 360:
      pass
    elif roverFace == 90:
      pass
    elif roverFace == 180:
      pass
    elif roverFace == 270:
      pass


    return adjlist

  def start(self, sequence):
    self.sequence = sequence
    self.runThread = Thread(target=self.runGame, daemon=True)
    self.runThread.start()

  def runGame(self):
    for move in self.sequence:
      self.client.send(move)
      self.queue.get()
      self.clearPos()
      sleep(1)
      self.queue.task_done()
      self.adjust()
    self.client.send(InternalMessage(MY_ROLE, END_GAME, b'0'))



if __name__ == "__main__":
  client = Client(TARGET_IP, TARGET_PORT, MY_ROLE)
  client.connect()
  rover = Rover(client)

  while 1:
    msg = client.get()
    if msg.msgtype == OBJECT_POS:
      obj = decipherMessage(msg.msg)
      if obj[0] == 163:
        print("Message Received: {} - Object: {} xPos: {}, yPos: {}, angle: {}, length: {}, width: {}".format(VAL_TO_MSG[msg.msgtype], *obj))
        rover.addPos(obj[1], obj[2], obj[3])

    elif msg.msgtype == ROVER_MOVE:
      if msg.msg[0] == ROVER_STOP:
        rover.queue.put(msg)

    elif msg.msgtype == START_GAME:
      client.send(InternalMessage(MY_ROLE, DEBUG_MSG, b'COORD START'))
      testcase = int(msg.msg[0])
      try:
        testseq = eval("test{}()".format(testcase))
        rover.start(testseq)
      except NameError:
        client.send(InternalMessage(COORDINATOR, DEBUG_MSG, b'BAD TEST'))
        print("Received bad test!")

    else:
      print("Message Received: {} - {}".format(VAL_TO_MSG[msg.msgtype], msg.msg))
