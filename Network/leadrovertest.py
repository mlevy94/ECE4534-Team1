from listener import Listener
from configs import *
from threading import Thread
from queue import Empty


def roverMove(direction, distance):
  return InternalMessage(CLIENT, ROVER_MOVE, bytes([direction, distance]))


def cmdInput(clientList):
  try:
    keyin = ""
    while keyin != "shutdown":
      keyin = input().lower()
      if keyin == "shutdown":
        break
      elif keyin == "test":
        msg = None
      elif keyin == "f":
        msg = roverMove(ROVER_FORWARD, 30)
      elif keyin == "b":
        msg = roverMove(ROVER_BACKWARD, 30)
      elif keyin == "l":
        msg = roverMove(ROVER_LEFT, 90)
      elif keyin == "r":
        msg = roverMove(ROVER_RIGHT, 90)
      elif keyin == "s":
        msg = roverMove(ROVER_STOP, 0)
      else:
        msg = None
      if msg is not None:
        for client in clientList:
          print("Message Sent {}: {} - {}".format(client.address, VAL_TO_MSG[msg.msgtype], msg.msg))
          try:
            client.send(msg)
          except (ConnectionResetError, BrokenPipeError):
            clientList.remove(client)
            print("Client Disconnected: {}".format(client.address))
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
      print("Message Received: {} - {}".format(VAL_TO_MSG[msg.msgtype], msg.msg))
    except Empty:
      pass
  listener.close()
