from socket import socket
from configs import getIPAddr, getPort
from outbound import OutboundWorker
from inbound import InboundWorker
from threading import Thread

class Connector:

  def __init__(self):
    self.clientDict = None
    self.clientList = []
    self.outbound = OutboundWorker(self.clientList, self.clientDict)
    self.queue = self.outbound.queue

  def start(self):
    self.outbound.start()
    listener = socket()
    listenThread = Thread(target=self.listener, args=[listener], daemon=True)
    listenThread.start()
    cmdString = ""
    try:
      while cmdString.lower() != "shutdown":
        cmdString = input()
        self.queue.put(cmdString.encode() + b'\0')
    finally:
      listener.close()

  def listener(self, listener=None):
    if listener is None:
      listener = socket()
    try:
      listener.bind((getIPAddr(), getPort()))
      listener.listen()
      print("Listening on {}:{}".format(getIPAddr(),getPort()))
      while 1:
        newSocket, addr = listener.accept()
        newClient = InboundWorker(newSocket, self.queue, addr, self.clientList)
        newClient.start()
        self.clientList.append(newClient)
        print("Client connected from {}".format(addr))
    finally:
      listener.close()

mainObj = Connector()
mainObj.start()

