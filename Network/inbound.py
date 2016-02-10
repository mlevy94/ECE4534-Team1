import threading

class InboundWorker:

  def __init__(self, client, outQueue, address="client", clientList = None):
    self.outQueue = outQueue
    self.client = client
    self.thread = None
    self.address = address
    self.clientList = clientList

  def start(self):
    self.thread = threading.Thread(target=self._clientRecv, daemon=True)
    self.thread.start()

  def _clientRecv(self):
    try:
      inMsg = b''
      while 1:
        inMsg += self.client.recv(4096)
        nullLocation = inMsg.find(b'\0')
        while nullLocation != -1:
          msg = inMsg[:nullLocation]
          inMsg = inMsg[nullLocation + 1:]
          print("Recv {}: {}".format(self.address, msg))
          self.outQueue.put(msg)
          nullLocation = inMsg.find(b'\0')
    except:
      if self.clientList is not None:
        self.clientList.remove(self)
      print("Client Disconnected: {}".format(self.address))

  def send(self, msg):
    self.client.sendall(msg)
