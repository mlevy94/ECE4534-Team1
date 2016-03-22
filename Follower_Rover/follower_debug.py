# Author: Chris Cox
# Debugging for follower rover
# file: follower_debug.py

from listener import Listener
from configs import *  # too many variables to import explicitly
from threading import Thread
from queue import Empty


MOVE_COUNT_FOLLOWER = 0;
DISTANCE_COUNT_FOLLOWER = 0;

listener = Listener()

def queuePeek(recv_queue_data):
  print("Message : ", recv_queue_data)
  listener.queue.put(recv_queue_data)

# This keeps a total of the rover movements needed
def totalMoves(currentMoveDistance):
  MOVE_COUNT_FOLLOWER = MOVE_COUNT_FOLLOWER + 1
  DISTANCE_COUNT_FOLLOWER = DISTANCE_COUNT_FOLLOWER + currentMoveDistance
  tempMoveCount = tempMoveCount + 1
  tempFive = tempFive + 1
  if tempMoveCount == 50:
    print("Total Moves = ", MOVE_COUNT_FOLLOWER)
    print("currentDistance = ", currentMoveDistance)
    distanceAverage(currentMoveDistance, DISTANCE_COUNT_FOLLOWER)
    tempMoveCount = 0
  elif tempFive == 5:
    print("moving now at 5")
    tempFive = 0

# This prints the average distance the rover has traveled
def distanceAverage(currentDistance, totalDistance):
  avgDistance = currentDistance / totalDistance
  print("Average Distance = ", avgDistance)

# This prints when a token is found by the follower
def followerTokenFND():
  print("Follower Found Token")

# This prints when the servo has finished scanning
def scanServo():
  print("Servo Completed Scan")

# This will eventually be used to write back to the PIC32
def cmdInput(queue):
  while 1:
    msgstring = input().encode()
    if msgstring == b'shutdown':
      break
    while msgstring:
      msg = InternalMessage(ROUTER, DEBUG_MSG, msgstring[:INTERNAL_MSG_SIZE + 1])
      queue.put(msg)
      msgstring = msgstring[INTERNAL_MSG_SIZE + 1:]

def incoming_message_handle(listener):
  while True:
    try:
      recv_queue_data = listener.queue.get(timeout=1)
      print("Message : ", recv_queue_data)
    except Empty:
      pass
    try:
      if listener.queue.empty():
        print("empty")
        pass
      else:
        recv_data = recv_queue_data.msgtype
        message_pack_recv = recv_queue_data.msg
        if recv_data == "Rover Move": # Follower to move FORWARD
          if message_pack_recv == ROVER_FORWARD:
            #totalMoves(distance)
            print ("Received Follower move forward")
          elif message_pack_recv == ROVER_BACKWARD: # Follower to move BACKWARD
            #totalMoves(distance)
            print ("Received Follower move backward")
          elif message_pack_recv == ROVER_LEFT: # Follower to move LEFT
            #totalMoves(distance)
            print ("Received Follower move left")
          elif message_pack_recv == ROVER_RIGHT: # Follower to move RIGHT
            #totalMoves(distance)
            print ("Received Follower move right")
          elif message_pack_recv == ROVER_STOP: # Follower to STOP
            #totalMoves(distance)
            print ("Received Follower STOP")
        elif recv_data == "Follower Distance": # Follower reports distance traveled
          totalMoves(message_pack_recv)
          print ("Received Follower Distance: ", message_pack_recv)
        elif recv_data == "Follower Token Found": # Follower reports found token
          followerTokenFND()
        elif recv_data == "Scanning Follower Servo": # Scanning Follower Servo
          if message_pack_recv == SCAN_STARTED: # Servo Scan Started
            print ("Received Scanning Beginning")
          elif message_pack_recv == LEAD_FOUND: # Scan returned Lead Found
            scanServo()
          elif message_pack_recv == OBJECT_FOUND: # Scan returned Object Found
            scanServo()
        elif recv_data == "Follower Distance To Lead": # Follower reports found token
          print ("Received Follower Distance To Lead: ", message_pack_recv)
        else:
          # Something unexpected happened
          print("Received unexpected data %d" % recv_data)
    except KeyboardInterrupt:
      print("CTRL C Detected")
      return


def main():
  incoming_message_Thread = Thread(target=incoming_message_handle, args=[listener], daemon=True)
  listener.start()
  incoming_message_Thread.start()
  while incoming_message_Thread.is_alive(): print("True")
  
  listener.close()

if __name__ == "__main__":
  main()
