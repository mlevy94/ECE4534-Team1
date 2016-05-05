from configs import *  # too many variables to import explicitly
from socket import socket
from queue import Queue
from threading import Thread

class Client:

  def __init__(self, targetIP, targetPort, role=None):
    self.targetIP = targetIP
    self.targetPort = targetPort
    self.client = socket()
    self.sentmsgcount = 0
    self.recvmsgcount = 0
    self.role = role
    self.queue = Queue()
    self.recvThread = Thread(target=self.recv, daemon=True)

  def connect(self):
    print("Connecting to ({}:{})...".format(self.targetIP, self.targetPort))
    self.client.connect((self.targetIP, self.targetPort))
    print("Established Connection.")
    self.recvThread.start()
    self.send(InternalMessage(self.role, CLIENT_ROLE, bytes([self.role])))

  def _put(self, msg):
    self.queue.put(msg)

  def get(self, timeout=None):
    return self.queue.get(timeout=timeout)

  def send(self, intmsg):
    if self.sentmsgcount < MAX_MSG_COUNT:
      self.sentmsgcount += 1
    else:
      self.sentmsgcount = 0
    netmsg = NetMessage(
        source= self.role,
        count= self.sentmsgcount,
        msgtype= intmsg.msgtype,
        msgsize= len(intmsg.msg),
        msg= intmsg.msg,
      )
    self.client.sendall(netmsg.serialize())

  def recv(self):
    try:
      serialstream = b''
      while 1:
        serialstream += self.client.recv(4096)
        while len(serialstream) > HEADER_SIZE:
          startbyte = serialstream.find(STARTBYTE)
          if startbyte < 0 or len(serialstream[startbyte:]) < HEADER_SIZE:
            # keep looking for start byte or wait for more data
            break
          else:
            # trim extra bytes in beginning
            serialstream = serialstream[startbyte:]
          # get header values
          ((startbyte, source, count, msgtype, msgsize), serialstream) = (serialstream[:HEADER_SIZE], serialstream[HEADER_SIZE:])
          # get message
          msg, serialstream = serialstream[:msgsize], serialstream[msgsize:]
          netmsg = NetMessage(
            source= source,
            count = count,
            msgtype= msgtype,
            msgsize = msgsize,
            msg=msg,
          )
          endbyte, serialstream = serialstream[0], serialstream[1:]
          if endbyte != ENDBYTE[0]:
            print("Bad End Byte: {}".format(endbyte))
            continue
          # place in outbound queue
          msg = netmsg.getMessage()
          if DEBUG_ON:
            print("Received Message: {} - {}".format(VAL_TO_MSG[msg.msgtype], msg.msg))
          self._put(msg)
          self.recvmsgcount += 1
    except ConnectionError:
      pass


if __name__ == "__main__":
  client = Client(getIPAddr(), getPort(), CLIENT)
  client.connect()
  while 1:
    msg = client.get()
    print("Message Received: {} - {}".format(VAL_TO_MSG[msg.msgtype], msg.msg))
