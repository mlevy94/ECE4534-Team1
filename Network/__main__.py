from socket import socket
from configs import *  # too many variables to import explicitly
from outbound import OutboundWorker
from client import ClientWorker
from threading import Thread
from time import sleep

class Connector:

  def __init__(self):
    self.clientList = []
    self.clientDict = {}
    self.outbound = OutboundWorker(self.clientList, self.clientDict)
    self.queue = self.outbound.queue

  def start(self):
    self.outbound.start()
    listener = socket()
    listenThread = Thread(target=self.listener, args=[listener], daemon=True)
    listenThread.start()
    cmdString = ""
    sleep(0.5)
    try:
      while 1:
        msgstring = input().encode()
        if msgstring == b'shutdown':
          break
        while msgstring:
          msg = InternalMessage(ROUTER, DEBUG_MSG, msgstring[:INTERNAL_MSG_SIZE + 1])
          self.queue.put(msg)
          msgstring = msgstring[INTERNAL_MSG_SIZE + 1:]
    finally:
      listener.close()

  def listener(self, listener=None):
    if listener is None:
      listener = socket()
    try:
      listener.bind((getIPAddr(), getPort()))
      listener.listen()
      print("Listening on {}:{}".format(getIPAddr(), getPort()))
      while 1:
        newSocket, addr = listener.accept()
        newClient = ClientWorker(newSocket, self.outbound, addr)
        newClient.start()
        self.clientList.append(newClient)
        print("Client connected from {}".format(addr))
    finally:
      listener.close()

mainObj = Connector()
mainObj.start()

