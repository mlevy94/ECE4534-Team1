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
        # CLIENT_ROLE messages handled here
        if msg.msgtype == CLIENT_ROLE:
          if msg.msg[0] != CLIENT:
            self.clientList.remove(msg.target)
            self.clientDict[msg.msg[0]] = msg.target
            print("Role Assigned: {} - {}".format(VAL_TO_ROLE[msg.msg[0]], msg.target.address))
        else:
          for client in msg.target:
            if client.send(msg):
              if DEBUG_ON:
                print("Sent {}: {} - {}".format(client.address, VAL_TO_MSG[msg.msgtype], msg.msg))
            else:
              self.clientDisconnect(client)
        self.queue.task_done()
        continue
      # send to client list
      for client in self.clientList:
        try:
          if client.send(msg):
            if DEBUG_ON:
              print("Sent {}: {} - {}".format(client.address, VAL_TO_MSG[msg.msgtype], msg.msg))
          else:
            self.clientDisconnect(client)
        except ConnectionError:
          self.clientDisconnect(client)
      # send to client roles
      for role, client in self.clientDict.items():
        # Check if client is connected for that role
        if client is None or role == msg.client:
          continue
        try:
          if msg.msgtype in ROLE_MSG_RECV[role] or (DEBUG_ON and msg.msgtype == DEBUG_MSG):
            if client.send(msg):
              if DEBUG_ON:
                print("Sent {}: {} - {}".format(VAL_TO_ROLE[role], VAL_TO_MSG[msg.msgtype], msg.msg))
        except ConnectionError:
          self.clientDisconnect(client)

      self.queue.task_done()

  def clientDisconnect(self, client):
    try:
      client.client.shutdown(SHUT_RDWR)
      client.client.close()
    except:
      pass
    try:
      self.clientList.remove(client)
    except ValueError:
      for role, roleclient in self.clientDict.items():
        if client is roleclient:
          self.clientDict[role] = None
    print("Client Disconnected: {}".format(client.address))
