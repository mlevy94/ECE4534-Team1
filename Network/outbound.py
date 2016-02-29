from queue import Queue
from configs import *
import threading




class OutboundWorker:

  def __init__(self, clientList, clientDict={}):
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
      # check for predefined target
      if msg.target is not None:
        if msg.msgtype == CLIENT_ROLE:
          if msg.msg[0] != CLIENT:
            self.clientList.remove(msg.target)
          self.clientDict[msg.msg[0]] = msg.target
          print("Role Assigned: {} - {}".format(VAL_TO_ROLE[msg.msg[0]], msg.target.name()))
        elif isinstance(msg.target, list):
          for target in msg.target:
            target.send(msg)
        else:
          msg.target.send(msg)
        self.queue.task_done()
        continue
      # send to client list
      for client in self.clientList:
        try:
          client.send(msg)
          print("Sent {}: {} - {}".format(client.address, VAL_TO_MSG[msg.msgtype], msg.msg))
        except ConnectionResetError:
          self.clientList.remove(client)
          print("Client Disconnected: {}".format(client.address))
      self.queue.task_done()

