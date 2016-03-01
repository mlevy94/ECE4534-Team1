from socket import socket, SHUT_RDWR
from queue import Queue
from threading import Thread

from client import ClientWorker
from configs import *  # too many variables to import explicitly



class Listener:

  def __init__(self, queue=None):
    self.clientList = []
    self.clientDict = {}
    if queue is None:
      self.queue = Queue()
    self.listener = None

  def start(self):
    listenThread = Thread(target=self.listenThread, daemon=True)
    listenThread.start()

  def close(self):
    if self.listener is not None:
      self.listener.close()


  def listenThread(self):
    if self.listener is None:
      self.listener = socket()
    try:
      self.listener.bind((getIPAddr(), getPort()))
      self.listener.listen()
      print("Listening on {}:{}".format(getIPAddr(), getPort()))
      while 1:
        newSocket, addr = self.listener.accept()
        newClient = ClientWorker(newSocket, self.queue, addr)
        newClient.start()
        self.clientList.append(newClient)
        print("Client connected from {}".format(addr))
    finally:
      self.listener.shutdown(SHUT_RDWR)
      self.listener.close()
