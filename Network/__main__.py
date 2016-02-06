from socket import socket
from configs import getIPAddr, getPort
from outbound import OutboundWorker
from inbound import InboundWorker

class Connector:

  def __init__(self):
    self.clientDict = None
    self.clientList = []
    self.outbound = OutboundWorker(self.clientList, self.clientDict)
    self.queue = self.outbound.queue

  def start(self):
    self.outbound.start()
    self.listener()

  def listener(self):
    listener = socket()
    listener.bind((getIPAddr(), getPort()))
    listener.listen()
    print("Listening on {}:{}".format(getIPAddr(),getPort()))
    while 1:
      newSocket, addr = listener.accept()
      newClient = InboundWorker(newSocket, self.queue, addr)
      newClient.start()
      self.clientList.append(newClient)
      print("Client connected from {}".format(addr))

mainObj = Connector()
mainObj.start()

