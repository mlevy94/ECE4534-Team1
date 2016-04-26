from configs import *
from threading import Thread
from queue import Empty
from testclient import Client

MY_ROLE = COORDINATOR
TARGET_IP = getIPAddr()
TARGET_PORT = getPort()

def sendMsg(msg, client):
  client.send(msg)
  if msg.msgtype == ROVER_MOVE:
    print("Message Sent: {} - {}, {}".format(VAL_TO_MSG[msg.msgtype], VAL_TO_ROV[msg.msg[0]], msg.msg[1]))
  else:
    print("Message Sent: {} - {}".format(VAL_TO_MSG[msg.msgtype], msg.msg))

####### BASIC TESTS ######

# move forward one square
def basicMoveTest1(client):
  sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)

# move forward 4 squares
def basicMoveTest2(client):
  for _ in range(4):
    sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)

# move backward 1 square
def basicMoveTest3(client):
  sendMsg(roverMove(ROVER_BACKWARD, 60, MY_ROLE), client)

# move backward 4 square
def basicMoveTest4(client):
  for _ in range(4):
    sendMsg(roverMove(ROVER_BACKWARD, 60, MY_ROLE), client)

# move forward 1 square, then backward 1 square
def basicMoveTest5(client):
  sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)
  sendMsg(roverMove(ROVER_BACKWARD, 60, MY_ROLE), client)

# move forward 4 suqares, then backward 4 squares
def basicMoveTest6(client):
  for _ in range(4):
    sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)
  for _ in range(4):
    sendMsg(roverMove(ROVER_BACKWARD, 60, MY_ROLE), client)

# move forward 1 square, then backward 1 square 4 times
def basicMoveTest7(client):
  for _ in range(4):
    sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)
    sendMsg(roverMove(ROVER_BACKWARD, 60, MY_ROLE), client)



# turn left 90 degrees
def basicTurnTest1(client):
  sendMsg(roverMove(ROVER_LEFT, 90, MY_ROLE), client)

# turn left 45 degrees
def basicTurnTest2(client):
  sendMsg(roverMove(ROVER_LEFT, 45, MY_ROLE), client)

# turn right 90 degrees
def basicTurnTest3(client):
  sendMsg(roverMove(ROVER_RIGHT, 90, MY_ROLE), client)

# turn right 45 degrees
def basicTurnTest4(client):
  sendMsg(roverMove(ROVER_RIGHT, 45, MY_ROLE), client)

# turn left 90 degrees then right 90 degrees
def basicTurnTest5(client):
  sendMsg(roverMove(ROVER_LEFT, 90, MY_ROLE), client)
  sendMsg(roverMove(ROVER_RIGHT, 90, MY_ROLE), client)

# turn right 90 degrees then left 90 degrees
def basicTurnTest6(client):
  sendMsg(roverMove(ROVER_RIGHT, 90, MY_ROLE), client)
  sendMsg(roverMove(ROVER_LEFT, 90, MY_ROLE), client)

# turn left 90 degrees, then right 90 degrees 4 times
def basicTurnTest7(client):
  for _ in range(4):
    sendMsg(roverMove(ROVER_LEFT, 90, MY_ROLE), client)
    sendMsg(roverMove(ROVER_RIGHT, 90, MY_ROLE), client)

# turn left a full circle
def basicTurnTest8(client):
  for _ in range(4):
    sendMsg(roverMove(ROVER_LEFT, 90, MY_ROLE), client)

# turn right a full circle
def basicTurnTest9(client):
  for _ in range(4):
    sendMsg(roverMove(ROVER_RIGHT, 90, MY_ROLE), client)

# turn left half a circle, then right half a circle
def basicTurnTest10(client):
  for _ in range(2):
    sendMsg(roverMove(ROVER_LEFT, 90, MY_ROLE), client)
  for _ in range(2):
    sendMsg(roverMove(ROVER_RIGHT, 90, MY_ROLE), client)

###### COMPLEX TESTS #######

# forward then left 4 times in a square
def complexTest1(client):
  sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)
  sendMsg(roverMove(ROVER_LEFT, 90, MY_ROLE), client)
  sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)
  sendMsg(roverMove(ROVER_LEFT, 90, MY_ROLE), client)
  sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)
  sendMsg(roverMove(ROVER_LEFT, 90, MY_ROLE), client)
  sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)
  sendMsg(roverMove(ROVER_LEFT, 90, MY_ROLE), client)

# forward then right 4 times in a square
def complexTest2(client):
  sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)
  sendMsg(roverMove(ROVER_RIGHT, 90, MY_ROLE), client)
  sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)
  sendMsg(roverMove(ROVER_RIGHT, 90, MY_ROLE), client)
  sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)
  sendMsg(roverMove(ROVER_RIGHT, 90, MY_ROLE), client)
  sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)
  sendMsg(roverMove(ROVER_RIGHT, 90, MY_ROLE), client)

# move in a figure 8 to the left
def complexTest3(client):
  sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)
  sendMsg(roverMove(ROVER_LEFT, 90, MY_ROLE), client)
  sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)
  sendMsg(roverMove(ROVER_LEFT, 90, MY_ROLE), client)
  sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)
  sendMsg(roverMove(ROVER_LEFT, 90, MY_ROLE), client)
  sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)
  sendMsg(roverMove(ROVER_LEFT, 90, MY_ROLE), client)
  sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)
  sendMsg(roverMove(ROVER_RIGHT, 90, MY_ROLE), client)
  sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)
  sendMsg(roverMove(ROVER_RIGHT, 90, MY_ROLE), client)
  sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)
  sendMsg(roverMove(ROVER_RIGHT, 90, MY_ROLE), client)
  sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)
  sendMsg(roverMove(ROVER_RIGHT, 90, MY_ROLE), client)

# obstacle placed 2 squares in front of rover. move around it.
def complexTest4(client):
  sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)
  sendMsg(roverMove(ROVER_LEFT, 90, MY_ROLE), client)
  sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)
  sendMsg(roverMove(ROVER_RIGHT, 90, MY_ROLE), client)
  sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)
  sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)
  sendMsg(roverMove(ROVER_RIGHT, 90, MY_ROLE), client)
  sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)
  sendMsg(roverMove(ROVER_LEFT, 90, MY_ROLE), client)
  sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)

# adjustment test. place rover at bottom right corner of square
def complexTest5(client):
  sendMsg(roverMove(ROVER_LEFT, 45, MY_ROLE), client)
  sendMsg(roverMove(ROVER_FORWARD, 2), client)
  sendMsg(roverMove(ROVER_RIGHT, 45, MY_ROLE), client)
  sendMsg(roverMove(ROVER_FORWARD, 60, MY_ROLE), client)

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

def cmdInput(client):
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
                basicMoveTest1(client)
              elif keyin == "2":
                basicMoveTest2(client)
              elif keyin == "3":
                basicMoveTest3(client)
              elif keyin == "4":
                basicMoveTest4(client)
              elif keyin == "5":
                basicMoveTest5(client)
              elif keyin == "6":
                basicMoveTest6(client)
              elif keyin == "7":
                basicMoveTest7(client)
          elif keyin == "2":
            while keyin != "back":
              keyin = input(BASIC_TURN_MENU).lower()
              if keyin == "0":
                break
              elif keyin == "1":
                basicTurnTest1(client)
              elif keyin == "2":
                basicTurnTest2(client)
              elif keyin == "3":
                basicTurnTest3(client)
              elif keyin == "4":
                basicTurnTest4(client)
              elif keyin == "5":
                basicTurnTest5(client)
              elif keyin == "6":
                basicTurnTest6(client)
              elif keyin == "7":
                basicTurnTest7(client)
              elif keyin == "8":
                basicTurnTest8(client)
              elif keyin == "9":
                basicTurnTest9(client)
              elif keyin == "10":
                basicTurnTest10(client)
          elif keyin == "3":
            while keyin != "back":
              keyin = input(COMPLEX_MENU).lower()
              if keyin == "0":
                break
              elif keyin == "1":
                complexTest1(client)
              elif keyin == "2":
                complexTest2(client)
              elif keyin == "3":
                complexTest3(client)
              elif keyin == "4":
                complexTest4(client)
              elif keyin == "5":
                complexTest5(client)
      elif keyin[0] == "f":
        try:
          msg = roverMove(ROVER_FORWARD, int(keyin[1:]), MY_ROLE)
        except (ValueError, IndexError):
          msg = roverMove(ROVER_FORWARD, 0, MY_ROLE)
      elif keyin[0] == "b":
        try:
          msg = roverMove(ROVER_BACKWARD, int(keyin[1:]), MY_ROLE)
        except (ValueError, IndexError):
          msg = roverMove(ROVER_BACKWARD, 0, MY_ROLE)
      elif keyin[0] == "l":
        try:
          msg = roverMove(ROVER_LEFT, int(keyin[1:]), MY_ROLE)
        except (ValueError, IndexError):
          msg = roverMove(ROVER_LEFT, 0, MY_ROLE)
      elif keyin[0] == "r":
        try:
          msg = roverMove(ROVER_RIGHT, int(keyin[1:]), MY_ROLE)
        except (ValueError, IndexError):
          msg = roverMove(ROVER_RIGHT, 0, MY_ROLE)
      elif keyin == "s":
        msg = roverMove(ROVER_STOP, 0, MY_ROLE)
      else:
        msg = InternalMessage(MY_ROLE, DEBUG_MSG, keyin.encode())
      if msg is not None:
        sendMsg(msg, client)
  except KeyboardInterrupt:
    pass

if __name__ == "__main__":
  client = Client(TARGET_IP, TARGET_PORT, MY_ROLE)
  client.connect()
  cmdThread = Thread(target=cmdInput, args=[client], daemon=True)
  cmdThread.start()
  while cmdThread.is_alive():
    try:
      msg = client.queue.get(timeout=1)
      if msg.msgtype == CLIENT_ROLE:
        print("Message Received: {} - {}".format(VAL_TO_MSG[msg.msgtype], VAL_TO_ROLE[msg.msg[0]]))
      elif msg.msgtype == ROVER_MOVE:
        print("Message Received: {} - {}, {}".format(VAL_TO_MSG[msg.msgtype], VAL_TO_ROV[msg.msg[0]], msg.msg[1]))
      else:
        print("Message Received: {} - {}".format(VAL_TO_MSG[msg.msgtype], msg.msg))
    except Empty:
      pass
