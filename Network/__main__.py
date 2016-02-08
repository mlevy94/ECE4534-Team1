from socket import socket
from configs import getIPAddr, getPort
from outbound import OutboundWorker
from inbound import InboundWorker
from threading import Thread
from time import sleep

class Connector:

  def __init__(self):
    self.clientDict = None
    self.clientList = []
    self.outbound = OutboundWorker(self.clientList, self.clientDict)
    self.queue = self.outbound.queue

  def start(self):
    self.outbound.start()
    listenThread = Thread(target=self.listener, daemon=True)
    listenThread.start()
    cmdString = ""
    sleep(0.5)
    try:
      while cmdString.lower() != "shutdown":
        cmdString = input("Message: ")
        self.queue.put(cmdString.encode() + b'\0')
    except KeyboardInterrupt:
      pass

  def listener(self):
    listener = socket()
    listener.bind((getIPAddr(), getPort()))
    listener.listen()
    print("Listening on {}:{}".format(getIPAddr(),getPort()))
    try:
      while 1:
        newSocket, addr = listener.accept()
        newClient = InboundWorker(newSocket, self.queue, addr)
        newClient.start()
        self.clientList.append(newClient)
        print("Client connected from {}".format(addr))
    except KeyboardInterrupt:
      listener.close()

mainObj = Connector()
mainObj.start()

