from listener import Listener
from configs import *
from threading import Thread
from queue import Empty


def roverMove(direction, distance):
  return InternalMessage(CLIENT, ROVER_MOVE, bytes([direction, distance]))

def sendMsg(msg, clientList):
  if msg is not None:
    for client in clientList:
      try:
        client.send(msg)
        print("Message Sent {}: {} - {}, {}".format(client.address, VAL_TO_MSG[msg.msgtype], VAL_TO_ROV[msg.msg[0]], msg.msg[1]))
      except (ConnectionResetError, BrokenPipeError):
        clientList.remove(client)
        print("Client Disconnected: {}".format(client.address))
        continue

####### BASIC TESTS ######

# move forward one square
def basicMoveTest1(clientList):
  sendMsg(roverMove(ROVER_FORWARD, 6), clientList)

# move forward 4 squares
def basicMoveTest2(clientList):
  for _ in range(4):
    sendMsg(roverMove(ROVER_FORWARD, 6), clientList)

# move backward 1 square
def basicMoveTest3(clientList):
  sendMsg(roverMove(ROVER_BACKWARD, 6), clientList)

# move backward 4 square
def basicMoveTest4(clientList):
  for _ in range(4):
    sendMsg(roverMove(ROVER_BACKWARD, 6), clientList)

# move forward 1 square, then backward 1 square
def basicMoveTest5(clientList):
  sendMsg(roverMove(ROVER_FORWARD, 6), clientList)
  sendMsg(roverMove(ROVER_BACKWARD, 6), clientList)

# move forward 4 suqares, then backward 4 squares
def basicMoveTest6(clientList):
  for _ in range(4):
    sendMsg(roverMove(ROVER_FORWARD, 6), clientList)
  for _ in range(4):
    sendMsg(roverMove(ROVER_BACKWARD, 6), clientList)

# move forward 1 square, then backward 1 square 4 times
def basicMoveTest7(clientList):
  for _ in range(4):
    sendMsg(roverMove(ROVER_FORWARD, 6), clientList)
    sendMsg(roverMove(ROVER_BACKWARD, 6), clientList)



# turn left 90 degrees
def basicTurnTest1(clientList):
  sendMsg(roverMove(ROVER_LEFT, 90), clientList)

# turn left 45 degrees
def basicTurnTest2(clientList):
  sendMsg(roverMove(ROVER_LEFT, 45), clientList)

# turn right 90 degrees
def basicTurnTest3(clientList):
  sendMsg(roverMove(ROVER_RIGHT, 90), clientList)

# turn right 45 degrees
def basicTurnTest4(clientList):
  sendMsg(roverMove(ROVER_RIGHT, 45), clientList)

# turn left 90 degrees then right 90 degrees
def basicTurnTest5(clientList):
  sendMsg(roverMove(ROVER_LEFT, 90), clientList)
  sendMsg(roverMove(ROVER_RIGHT, 90), clientList)

# turn right 90 degrees then left 90 degrees
def basicTurnTest6(clientList):
  sendMsg(roverMove(ROVER_RIGHT, 90), clientList)
  sendMsg(roverMove(ROVER_LEFT, 90), clientList)

# turn left 90 degrees, then right 90 degrees 4 times
def basicTurnTest7(clientList):
  for _ in range(4):
    sendMsg(roverMove(ROVER_LEFT, 90), clientList)
    sendMsg(roverMove(ROVER_RIGHT, 90), clientList)

# turn left a full circle
def basicTurnTest8(clientList):
  for _ in range(4):
    sendMsg(roverMove(ROVER_LEFT, 90), clientList)

# turn right a full circle
def basicTurnTest9(clientList):
  for _ in range(4):
    sendMsg(roverMove(ROVER_RIGHT, 90), clientList)

# turn left half a circle, then right half a circle
def basicTurnTest10(clientList):
  for _ in range(2):
    sendMsg(roverMove(ROVER_LEFT, 90), clientList)
  for _ in range(2):
    sendMsg(roverMove(ROVER_RIGHT, 90), clientList)

###### COMPLEX TESTS #######

# forward then left 4 times in a square
def complexTest1(clientList):
  sendMsg(roverMove(ROVER_FORWARD, 6), clientList)
  sendMsg(roverMove(ROVER_LEFT, 90), clientList)
  sendMsg(roverMove(ROVER_FORWARD, 6), clientList)
  sendMsg(roverMove(ROVER_LEFT, 90), clientList)
  sendMsg(roverMove(ROVER_FORWARD, 6), clientList)
  sendMsg(roverMove(ROVER_LEFT, 90), clientList)
  sendMsg(roverMove(ROVER_FORWARD, 6), clientList)
  sendMsg(roverMove(ROVER_LEFT, 90), clientList)

# forward then right 4 times in a square
def complexTest2(clientList):
  sendMsg(roverMove(ROVER_FORWARD, 6), clientList)
  sendMsg(roverMove(ROVER_RIGHT, 90), clientList)
  sendMsg(roverMove(ROVER_FORWARD, 6), clientList)
  sendMsg(roverMove(ROVER_RIGHT, 90), clientList)
  sendMsg(roverMove(ROVER_FORWARD, 6), clientList)
  sendMsg(roverMove(ROVER_RIGHT, 90), clientList)
  sendMsg(roverMove(ROVER_FORWARD, 6), clientList)
  sendMsg(roverMove(ROVER_RIGHT, 90), clientList)

# move in a figure 8 to the left
def complexTest3(clientList):
  sendMsg(roverMove(ROVER_FORWARD, 6), clientList)
  sendMsg(roverMove(ROVER_LEFT, 90), clientList)
  sendMsg(roverMove(ROVER_FORWARD, 6), clientList)
  sendMsg(roverMove(ROVER_LEFT, 90), clientList)
  sendMsg(roverMove(ROVER_FORWARD, 6), clientList)
  sendMsg(roverMove(ROVER_LEFT, 90), clientList)
  sendMsg(roverMove(ROVER_FORWARD, 6), clientList)
  sendMsg(roverMove(ROVER_LEFT, 90), clientList)
  sendMsg(roverMove(ROVER_FORWARD, 6), clientList)
  sendMsg(roverMove(ROVER_RIGHT, 90), clientList)
  sendMsg(roverMove(ROVER_FORWARD, 6), clientList)
  sendMsg(roverMove(ROVER_RIGHT, 90), clientList)
  sendMsg(roverMove(ROVER_FORWARD, 6), clientList)
  sendMsg(roverMove(ROVER_RIGHT, 90), clientList)
  sendMsg(roverMove(ROVER_FORWARD, 6), clientList)
  sendMsg(roverMove(ROVER_RIGHT, 90), clientList)

# obstacle placed 2 squares in front of rover. move around it.
def complexTest4(clientList):
  sendMsg(roverMove(ROVER_FORWARD, 6), clientList)
  sendMsg(roverMove(ROVER_LEFT, 90), clientList)
  sendMsg(roverMove(ROVER_FORWARD, 6), clientList)
  sendMsg(roverMove(ROVER_RIGHT, 90), clientList)
  sendMsg(roverMove(ROVER_FORWARD, 6), clientList)
  sendMsg(roverMove(ROVER_FORWARD, 6), clientList)
  sendMsg(roverMove(ROVER_RIGHT, 90), clientList)
  sendMsg(roverMove(ROVER_FORWARD, 6), clientList)
  sendMsg(roverMove(ROVER_LEFT, 90), clientList)
  sendMsg(roverMove(ROVER_FORWARD, 6), clientList)

# adjustment test. place rover at bottom right corner of square
def complexTest5(clientList):
  sendMsg(roverMove(ROVER_LEFT, 45), clientList)
  sendMsg(roverMove(ROVER_FORWARD, 2), clientList)
  sendMsg(roverMove(ROVER_RIGHT, 45), clientList)
  sendMsg(roverMove(ROVER_FORWARD, 6), clientList)

###### END COMPLEX TESTS ######

MAIN_MENU = """
"Available Tests:
0 - Go Back
1 - Basic Movement Tests
2 - Basic Turning Tests
3 - Complex Tests
"""

BASIC_MOVE_MENU = """
0 - Go Back
1 - Forward 1
2 - Forward 4
3 - Backward 1
4 - Backward 4
5 - Forward 1, Backward 1
6 - Forward 4, Backward 4
7 - Forward 1, Backward 1 x4
"""

BASIC_TURN_MENU = """
0 - Go Back
1 - Turn Left 90 Degrees
2 - Turn Left 45 Degrees
3 - Turn Right 90 Degrees
4 - Turn Right 45 Degrees
5 - Turn Left 90 Degrees, Turn Right 90 Degrees
6 - Turn Right 90 Degrees, Turn Left 90 Degrees
7 - Turn Left 90 Degrees, Turn Right 90 Degrees x4
8 - Turn Left Full Circle
9 - Turn Right Full Circle
10- Turn Left Half Cirlc, Turn Right Half Circle
"""

COMPLEX_MENU = """
0 - Go Back
1 - Go Around in Square To Left
2 - Go Around in Square To Right
3 - Perform Figure 8, Starting Left
4 - Go Around Obstacle 2 Squares in Front
5 - Adjust To Center, Go Forward. Start at Bottom Right Corner
"""

def cmdInput(clientList):
  try:
    keyin = ""
    msg = None
    while keyin != "shutdown":
      keyin = input().lower()
      if keyin == "shutdown":
        break
      elif keyin == "":
        continue
      elif keyin == "test":
        while keyin != "back":
          keyin = input(MAIN_MENU).lower()
          if keyin == "0":
            break
          elif keyin == "1":
            while keyin != "back":
              keyin = input(BASIC_MOVE_MENU).lower()
              if keyin == "0":
                break
              elif keyin == "1":
                basicMoveTest1(clientList)
              elif keyin == "2":
                basicMoveTest2(clientList)
              elif keyin == "3":
                basicMoveTest3(clientList)
              elif keyin == "4":
                basicMoveTest4(clientList)
              elif keyin == "5":
                basicMoveTest5(clientList)
              elif keyin == "6":
                basicMoveTest6(clientList)
              elif keyin == "7":
                basicMoveTest7(clientList)
          elif keyin == "2":
            while keyin != "back":
              keyin = input(BASIC_TURN_MENU).lower()
              if keyin == "0":
                break
              elif keyin == "1":
                basicTurnTest1(clientList)
              elif keyin == "2":
                basicTurnTest2(clientList)
              elif keyin == "3":
                basicTurnTest3(clientList)
              elif keyin == "4":
                basicTurnTest4(clientList)
              elif keyin == "5":
                basicTurnTest5(clientList)
              elif keyin == "6":
                basicTurnTest6(clientList)
              elif keyin == "7":
                basicTurnTest7(clientList)
              elif keyin == "8":
                basicTurnTest8(clientList)
              elif keyin == "9":
                basicTurnTest9(clientList)
              elif keyin == "10":
                basicTurnTest10(clientList)
          elif keyin == "3":
            while keyin != "back":
              keyin = input(COMPLEX_MENU).lower()
              if keyin == "0":
                break
              elif keyin == "1":
                complexTest1(clientList)
              elif keyin == "2":
                complexTest2(clientList)
              elif keyin == "3":
                complexTest3(clientList)
              elif keyin == "4":
                complexTest4(clientList)
              elif keyin == "5":
                complexTest5(clientList)
      elif keyin[0] == "f":
        try:
          msg = roverMove(ROVER_FORWARD, int(keyin[1:]))
        except (ValueError, IndexError):
          msg = roverMove(ROVER_FORWARD, 0)
      elif keyin[0] == "b":
        try:
          msg = roverMove(ROVER_BACKWARD, int(keyin[1:]))
        except (ValueError, IndexError):
          msg = roverMove(ROVER_BACKWARD, 0)
      elif keyin[0] == "l":
        try:
          msg = roverMove(ROVER_LEFT, int(keyin[1:]))
        except (ValueError, IndexError):
          msg = roverMove(ROVER_LEFT, 0)
      elif keyin[0] == "r":
        try:
          msg = roverMove(ROVER_RIGHT, int(keyin[1:]))
        except (ValueError, IndexError):
          msg = roverMove(ROVER_RIGHT, 0)
      elif keyin == "s":
        msg = roverMove(ROVER_STOP, 0)
      else:
        msg = None
      sendMsg(msg, clientList)
  except KeyboardInterrupt:
    pass

if __name__ == "__main__":
  listener = Listener()
  cmdThread = Thread(target=cmdInput, args=[listener.clientList], daemon=True)
  listener.start()
  cmdThread.start()
  while cmdThread.is_alive():
    try:
      msg = listener.queue.get(timeout=1)
      if msg.msgtype == CLIENT_ROLE:
        print("Message Received: {} - {}".format(VAL_TO_MSG[msg.msgtype], VAL_TO_ROLE[msg.msg[0]]))
      elif msg.msgtype == ROVER_MOVE:
        print("Message Received: {} - {}, {}".format(VAL_TO_MSG[msg.msgtype], VAL_TO_ROV[msg.msg[0]], msg.msg[1]))
      else:
        print("Message Received: {} - {}".format(VAL_TO_MSG[msg.msgtype], msg.msg))
    except Empty:
      pass
  listener.close()
