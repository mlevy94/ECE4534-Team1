from queue import Queue
import threading




class OutboundWorker:

  def __init__(self, clientList, clientDict= None):
    if not isinstance(clientList, list):
      clientList = [clientList]
    self.clientList = clientList
    self.clientDict = clientDict
    self.queue = Queue()
    self.thread = None

  def start(self):
    self.thread = threading.Thread(target=self._commThread, daemon=True)
    self.thread.start()

  def _commThread(self):
    while 1:
      msg = self.queue.get()
      for client in self.clientList:
          client.send(msg)
          print("Send {}: {}".format(client.address, msg))


