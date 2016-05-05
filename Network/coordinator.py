from configs import *  # too many variables to import explicitly
from testclient import Client
from collections import Counter
from threading import Thread
from queue import Queue, Empty
from time import sleep
from integrationtests import calibrationTest, test1, test2, test3, test4, test5, test6, test7 # do not delete


TARGET_IP = getIPAddr()
TARGET_PORT = getPort()
MY_ROLE = COORDINATOR



class Rover:
  angle_buff = 10
  angle_cutoff = 45
  dist_buff = 3
  xSq = 6
  ySq = 6
  bot_bind = 0
  top_bind = 36

  def __init__(self, client):
    self.client = client
    self.xposlist = Counter()
    self.yposlist = Counter()
    self.anglelist = Counter()
    self.sequence = []
    self.queue = Queue()
    self.runThread = None
    self.running = False
    self.angle = 0

  def addPos(self, xpos, ypos, angle):
    if xpos > self.top_bind:
      xpos = self.top_bind
    elif xpos < self.bot_bind:
      xpos = self.bot_bind
    if ypos > self.top_bind:
      ypos = self.top_bind
    elif ypos < self.bot_bind:
      ypos = self.bot_bind
    self.xposlist[xpos] += 1
    self.yposlist[ypos] += 1
    self.anglelist[angle] += 1

  def clearPos(self):
    # clear out lists for next time
    self.xposlist.clear()
    self.yposlist.clear()
    self.anglelist.clear()

  def changeAngle(self, adjAngle):
    self.angle += adjAngle
    if self.angle < 0:
      self.angle += 360
    elif self.angle > 360:
      self.angle -= 360
    print("angle: {}".format(self.angle))

  def adjust(self, nextDir=None):
    adjlist = []
    # make sure enough samples have been collected
    while sum(self.xposlist.values()) < 10:
      sleep(0.1)
    # get useful values then clear them
    x = self.xposlist.most_common()[0][0]
    y = self.yposlist.most_common()[0][0]
    a = self.anglelist.most_common()[0][0]
    print("Adj Vals: X:{} Y:{} A:{}".format(self.xposlist.most_common(), self.yposlist.most_common(), self.anglelist.most_common()))
    self.clearPos()

    # get adjustment values
    xadj = int(round((self.xSq / 2) - (x % self.xSq), 1) * 10)
    yadj = int(round((self.ySq / 2) - (y % self.ySq), 1) * 10)
    roverFace = round(a / 90) * 90
    aadj = int(round(((roverFace - a) / 5)) * 5)
    # large angle adjust
    if abs(aadj) >= self.angle_buff:
      if aadj < 0:
        adjlist.append(roverMove(ROVER_LEFT, abs(aadj)))
      else:
        adjlist.append(roverMove(ROVER_RIGHT, aadj))


    if roverFace == 0 or roverFace == 360:
      # y adjust
      if abs(yadj) > self.dist_buff:
        if yadj < 0:
          adjlist.append(roverMove(ROVER_FORWARD, abs(yadj)))
        else:
          adjlist.append(roverMove(ROVER_BACKWARD, yadj))
      # x adjust
      if nextDir == ROVER_RIGHT:
        adjlist.append(roverMove(ROVER_RIGHT, 90))
        if abs(xadj) > self.dist_buff:
          if xadj > 0:
            adjlist.append(roverMove(ROVER_FORWARD, abs(xadj)))
          else:
            adjlist.append(roverMove(ROVER_BACKWARD, abs(xadj)))
      else:
        if abs(xadj) > self.dist_buff:
          adjlist.append(roverMove(ROVER_LEFT, 90))
          if xadj < 0:
            adjlist.append(roverMove(ROVER_FORWARD, abs(xadj)))
          else:
            adjlist.append(roverMove(ROVER_BACKWARD, abs(xadj)))
          if nextDir != ROVER_LEFT:
            adjlist.append(roverMove(ROVER_RIGHT, 90))
        elif nextDir == ROVER_LEFT:
          adjlist.append(roverMove(ROVER_LEFT, 90))

    elif roverFace == 90:
      # x adjust
      if abs(xadj) > self.dist_buff:
        if xadj < 0:
          adjlist.append(roverMove(ROVER_BACKWARD, abs(xadj)))
        else:
          adjlist.append(roverMove(ROVER_FORWARD, abs(xadj)))
      # y adjust
      if nextDir == ROVER_RIGHT:
        adjlist.append(roverMove(ROVER_RIGHT, 90))
        if abs(yadj) > self.dist_buff:
          if yadj > 0:
            adjlist.append(roverMove(ROVER_FORWARD, abs(yadj)))
          else:
            adjlist.append(roverMove(ROVER_BACKWARD, abs(yadj)))
      else:
        if abs(yadj) > self.dist_buff:
          adjlist.append(roverMove(ROVER_LEFT, 90))
          if yadj < 0:
            adjlist.append(roverMove(ROVER_FORWARD, abs(yadj)))
          else:
            adjlist.append(roverMove(ROVER_BACKWARD, abs(yadj)))
          if nextDir != ROVER_LEFT:
            adjlist.append(roverMove(ROVER_RIGHT, 90))
        elif nextDir == ROVER_LEFT:
          adjlist.append(roverMove(ROVER_LEFT, 90))
    elif roverFace == 180:
      # y adjust
      if abs(yadj) > self.dist_buff:
        if yadj < 0:
          adjlist.append(roverMove(ROVER_BACKWARD, abs(yadj)))
        else:
          adjlist.append(roverMove(ROVER_FORWARD, abs(yadj)))
      # x adjust
      if nextDir == ROVER_RIGHT:
        adjlist.append(roverMove(ROVER_RIGHT, 90))
        if abs(xadj) > self.dist_buff:
          if xadj > 0:
            adjlist.append(roverMove(ROVER_BACKWARD, abs(xadj)))
          else:
            adjlist.append(roverMove(ROVER_FORWARD, abs(xadj)))
      else:
        if abs(xadj) > self.dist_buff:
          adjlist.append(roverMove(ROVER_LEFT, 90))
          if xadj < 0:
            adjlist.append(roverMove(ROVER_BACKWARD, abs(xadj)))
          else:
            adjlist.append(roverMove(ROVER_FORWARD, abs(xadj)))
          if nextDir != ROVER_LEFT:
            adjlist.append(roverMove(ROVER_RIGHT, 90))
        elif nextDir == ROVER_LEFT:
          adjlist.append(roverMove(ROVER_LEFT, 90))
    elif roverFace == 270:
      # x adjust
      if abs(xadj) > self.dist_buff:
        if xadj < 0:
          adjlist.append(roverMove(ROVER_FORWARD, abs(xadj)))
        else:
          adjlist.append(roverMove(ROVER_BACKWARD, abs(xadj)))
      # y adjust
      if nextDir == ROVER_RIGHT:
        adjlist.append(roverMove(ROVER_RIGHT, 90))
        if abs(yadj) > self.dist_buff:
          if yadj > 0:
            adjlist.append(roverMove(ROVER_BACKWARD, abs(yadj)))
          else:
            adjlist.append(roverMove(ROVER_FORWARD, abs(yadj)))
      else:
        if abs(yadj) > self.dist_buff:
          adjlist.append(roverMove(ROVER_LEFT, 90))
          if yadj < 0:
            adjlist.append(roverMove(ROVER_BACKWARD, abs(yadj)))
          else:
            adjlist.append(roverMove(ROVER_FORWARD, abs(yadj)))
          if nextDir != ROVER_LEFT:
            adjlist.append(roverMove(ROVER_RIGHT, 90))
        elif nextDir == ROVER_LEFT:
          adjlist.append((roverMove(ROVER_LEFT, 90)))


    return adjlist

  def start(self, orientation, sequence, adjust=True):
    assert not self.running
    self.running = True
    self.angle = orientation
    self.sequence = sequence
    self.runThread = Thread(target=self.runGame, args= [adjust], daemon=True)
    print("Starting Test")
    self.runThread.start()

  def runGame(self, adjustbool=True):
    # clear queue just in case
    try:
      while True:
        self.queue.get(False) # non-blocking
        self.queue.task_done()
    except Empty:
      pass
    # process moves
    iterator = peekIter(self.sequence)
    generator = iterator.generate()
    for move in generator:
      if not self.running:
        print("Test Aborted!")
        self.client.send(InternalMessage(MY_ROLE, DEBUG_MSG, b'TEST ABORTED'))
        return
      self.client.send(move)
      self.queue.get()
      if bytetoval(move.msg[1]) == 90 and bytetoval(msg.msg[0]) == ROVER_LEFT:
        self.changeAngle(-90)
      elif bytetoval(move.msg[1]) == 90 and bytetoval(msg.msg[0]) == ROVER_RIGHT:
        self.changeAngle(90)
      self.clearPos()
      sleep(1)
      self.queue.task_done()
      if not self.running:
        print("Test Aborted!")
        self.client.send(InternalMessage(MY_ROLE, DEBUG_MSG, b'TEST ABORTED'))
        return
      if adjustbool and bytetoval(move.msg[0]) == ROVER_FORWARD:
        nextMove = iterator.peek()
        if nextMove is not None:
          nextDir = nextMove.msg[0]
          if nextDir == ROVER_LEFT or nextDir == ROVER_RIGHT:
            next(generator) # discard next move
        else:
          nextDir = None
        adjlist = self.adjust(nextDir)
        # make adjustments
        for adj in adjlist:
          if not self.running:
            print("Test Aborted")
            self.client.send(InternalMessage(MY_ROLE, DEBUG_MSG, b'TEST ABORTED'))
            return
          self.client.send(adj)
          self.queue.get()
          if bytetoval(adj.msg[1]) == 90 and bytetoval(adj.msg[0]) == ROVER_LEFT:
            self.changeAngle(-90)
          elif bytetoval(adj.msg[1]) == 90 and bytetoval(adj.msg[0]) == ROVER_RIGHT:
            self.changeAngle(90)
          self.queue.task_done()
    # game over
    print("Test Ended")
    self.client.send(InternalMessage(MY_ROLE, END_GAME, b'0'))
    self.client.send(InternalMessage(MY_ROLE, DEBUG_MSG, b'TEST END'))
    self.running = False



if __name__ == "__main__":
  client = Client(TARGET_IP, TARGET_PORT, MY_ROLE)
  client.connect()
  rover = Rover(client)

  while 1:
    msg = client.get()
    if msg.msgtype == OBJECT_POS:
      obj = decipherMessage(msg.msg)
      if obj[0] == 163:
        # print("Message Received: {} - Object: {} xPos: {}, yPos: {}, angle: {}, length: {}, width: {}".format(VAL_TO_MSG[msg.msgtype], *obj))
        rover.addPos(obj[1], obj[2], obj[3])

    elif msg.msgtype == PING:
      client.send(InternalMessage(MY_ROLE, PONG, b'1'))

    elif msg.msgtype == ROVER_MOVE:
      if msg.msg[0] == ROVER_STOP:
        rover.queue.put(msg) # queue of stop messages

    elif msg.msgtype == START_GAME:
      client.send(InternalMessage(MY_ROLE, DEBUG_MSG, b'TEST START'))
      testcase = int(msg.msg[0])
      testseq = 0
      try:
        orientation, testseq = eval("test{}()".format(testcase))
        rover.start(orientation, testseq)
      except NameError:
        client.send(InternalMessage(MY_ROLE, DEBUG_MSG, b'BAD TEST'))
        print("Received bad test {}!".format(testseq))
      except AssertionError:
        client.send(InternalMessage(MY_ROLE, DEBUG_MSG, b'CANT RUN'))
        print("Test already running!")

    elif msg.msgtype == END_GAME:
      rover.running = False

    elif msg.msgtype == CALIBRATE_ROVER:
      try:
        print("Starting Calibration")
        rover.start(*calibrationTest(), False)
        client.send(InternalMessage(MY_ROLE, DEBUG_MSG, b'ROV CALIB'))
      except AssertionError:
        print("Test currently running!")

    else:
      print("Message Received: {} - {}".format(VAL_TO_MSG[msg.msgtype], msg.msg))
