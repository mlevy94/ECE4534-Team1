from configs import *
from socket import SHUT_RDWR
import threading




class OutboundWorker:

  def __init__(self, queue, clientList, clientDict={}):
    if not isinstance(clientList, list):
      clientList = [clientList]
    self.clientList = clientList
    self.clientDict = clientDict
    self.queue = queue
    self.thread = None

  def start(self):
    self.thread = threading.Thread(target=self._commThread, daemon=True)
    self.thread.start()

  def _commThread(self):
    while 1:
      msg = self.queue.get()
      # check for predefined target
      if msg.target:
        if msg.msgtype == CLIENT_ROLE:
          if msg.msg[0] != CLIENT:
            self.clientList.remove(msg.target)
          self.clientDict[msg.msg[0]] = msg.target
          print("Role Assigned: {} - {}".format(VAL_TO_ROLE[msg.msg[0]], msg.target.address))
        else:
          for client in msg.target:
            if client.send(msg):
              print("Sent {}: {} - {}".format(client.address, VAL_TO_MSG[msg.msgtype], msg.msg))
            else:
              self.clientDisconnect(client)
        self.queue.task_done()
        continue
      # send to client list
      for client in self.clientList:
        try:
          if client.send(msg):
            print("Sent {}: {} - {}".format(client.address, VAL_TO_MSG[msg.msgtype], msg.msg))
          else:
            self.clientDisconnect(client)
        except (ConnectionResetError, BrokenPipeError):
          self.clientDisconnect(client)

      self.queue.task_done()

  def clientDisconnect(self, client):
    try:
      client.client.shutdown(SHUT_RDWR)
      client.client.close()
    except:
      pass
    self.clientList.remove(client)
    print("Client Disconnected: {}".format(client.address))
