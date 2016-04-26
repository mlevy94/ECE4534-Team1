from listener import Listener
from outbound import OutboundWorker
from configs import *  # too many variables to import explicitly


def cmdInput(listener):
  while 1:
    msgstring = input().lower()

    if msgstring == "shutdown":
      break

    elif msgstring == "clients":
      print("Connected Clients:")
      if listener.clientList:
        for client in listener.clientList:
          print("  Client: {}".format(client.address))
      else:
        print("  None")
      print("Connected Roles:")
      if listener.clientDict:
        for role, client in listener.clientDict.items():
          print("  {}: {}".format(VAL_TO_ROLE[role].title(), client.address))
      else:
        print("  None")

    elif msgstring == "start":
      try:
        testcase = int(input("Select Test Case:"))
        assert testcase in range(1, 8)
      except (TypeError, AssertionError):
        print("Test case does not exist")
        continue
      listener.queue.put(InternalMessage(ROUTER, START_GAME, bytes([testcase])))

    elif msgstring == "end":
      listener.queue.put(InternalMessage(ROUTER, END_GAME, bytes([0])))

    else:
      msgstring = msgstring.encode()
      while msgstring:
        msg = InternalMessage(ROUTER, DEBUG_MSG, msgstring[:INTERNAL_MSG_SIZE + 1])
        listener.queue.put(msg)
        msgstring = msgstring[INTERNAL_MSG_SIZE + 1:]

if __name__ == "__main__":
  listener = Listener()
  outWorker = OutboundWorker(listener.queue, listener.clientList, listener.clientDict)
  listener.start()
  outWorker.start()
  try:
    cmdInput(listener)
  except KeyboardInterrupt:
    pass
  listener.close()

