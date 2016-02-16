from configs import STARTBYTE, ENDBYTE
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
        start = inMsg.find(STARTBYTE)
        end = inMsg.find(ENDBYTE)

        while start != -1 and end != -1 and start < end:
          msg = inMsg[:end]
          inMsg = inMsg[end + 1:]
          print("Recv {}: {}".format(self.address, msg))
          self.outQueue.put(msg)
          end = inMsg.find(b'\0')
    except:
      if self.clientList is not None:
        self.clientList.remove(self)
      print("Client Disconnected: {}".format(self.address))

  def send(self, msg):
    self.client.sendall(msg)
