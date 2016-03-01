# Author: Chris Cox
# Debugging for follower rover
# file: follower_debug.py

from listener import Listener
from configs import *  # too many variables to import explicitly

MOVE_COUNT_FOLLOWER = 0;
DISTANCE_COUNT_FOLLOWER = 0;

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

def cmdInput(queue):
  while 1:
    msgstring = input().encode()
    if msgstring == b'shutdown':
      break
    while msgstring:
      msg = InternalMessage(ROUTER, DEBUG_MSG, msgstring[:INTERNAL_MSG_SIZE + 1])
      queue.put(msg)
      msgstring = msgstring[INTERNAL_MSG_SIZE + 1:]

def main():
  listener = Listener()
  listener.start()
  while True:
    recv_queue_data = listener.queue.get()
    recv_data = recv_queue_data.msgtype
    distance = recv_queue_data.msg
    if recv_data == 0x05: # Follower to move forward
      totalMoves(distance)
      print ("Received Follower move forward")
    elif recv_data == 0x06: # Follower to move backward
      totalMoves(distance)
      print ("Received Follower move backward")
    elif recv_data == 0x07: # Follower to move left
      totalMoves(distance)
      print ("Received Follower move left")
    elif recv_data == 0x09: # Follower to move right
      totalMoves(distance)
      print ("Received Follower move right")
    elif recv_data == 0x13: # Follower reports found token
      followerTokenFND()
    elif recv_data == 0x15: # Scanning Follower Servo
      print ("Received Scanning Beginning")
    elif recv_data == 0x16: # Scan returned Lead Found
      scanServo()
    elif recv_data == 0x17: # Scan returned Object Found
      scanServo()
    else:
      # Something unexpected happened
      print("Received unexpected data %d" % recv_data)
  try:
    cmdInput(listener.queue)
  except KeyboardInterrupt:
    pass
  listener.close()

if __name__ == "__main__":
  main()

