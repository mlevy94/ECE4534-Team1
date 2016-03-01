

from listener import Listener
from outbound import OutboundWorker
from configs import *  # too many variables to import explicitly

def cmdInput(queue):
  while 1:
    msgstring = input().encode()
    if msgstring == b'shutdown':
      break
    while msgstring:
      msg = InternalMessage(ROUTER, DEBUG_MSG, msgstring[:INTERNAL_MSG_SIZE + 1])
      queue.put(msg)
      msgstring = msgstring[INTERNAL_MSG_SIZE + 1:]

if __name__ == "__main__":
  listener = Listener()
  outWorker = OutboundWorker(listener.queue, listener.clientList, listener.clientDict)
  listener.start()
  outWorker.start()
  try:
    cmdInput(listener.queue)
  except KeyboardInterrupt:
    pass
  listener.close()

