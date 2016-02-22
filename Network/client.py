from configs import *  # too many variables to import explicitly
import threading


class ClientWorker:

  def __init__(self, client, outWorker, address=None):
    if address is None:
      self.address = client.getsockname()
    self.outWorker = outWorker
    self.client = client
    self.thread = None
    self.address = address
    self.msgcount = 0
    # initialize the message cache
    self.sentlist = [None for _ in range(MAX_MSG_COUNT)]

  def start(self):
    self.thread = threading.Thread(target=self._clientRecv, daemon=True)
    self.thread.start()

  def name(self):
    return self.client.getsockname()

  def _clientRecv(self):
    try:
      serialstream = b''
      while 1:
        serialstream += self.client.recv(4096)
        msgstart = serialstream.find(STARTBYTE)
        # make sure we at least have the entire header before starting
        while msgstart > -1 and len(serialstream) - msgstart >= HEADER_SIZE:
          serialstream = serialstream[msgstart:] # discard bytes before start of message
          msgsize = serialstream[4]
          # check for bad message length
          if msgsize > INTERNAL_MSG_SIZE:
            serialstream = serialstream[1:] # discard start character
            msgstart = serialstream.find(STARTBYTE)
            print("Invald Message: {}".format(serialstream))
            continue
          netmsgsize = HEADER_SIZE + TAIL_SIZE + msgsize
          # wait until the entire message arrives
          while len(serialstream) < netmsgsize:
            serialstream += self.client.recv(4096)
          # check end byte presence
          if serialstream[netmsgsize - 1] == ENDBYTE[0]:
            netmsg = NetMessage(
              source= serialstream[1],
              count = serialstream[2],
              msgtype= serialstream[3],
              msgsize = msgsize,
              msg=serialstream[5: 5 + msgsize],
              checksum = serialstream[msgsize + HEADER_SIZE: msgsize + HEADER_SIZE + 2],
            )
            # check message validity, send if it's okay.
            if netmsg.checkMessage():
              # request all missing messages first
              while self.msgcount < netmsg.count:
                self._put(InternalMessage(ROUTER, MSG_REQUEST, bytes([self.msgcount]), self))
                self.msgcount += 1
              # send
              msg = netmsg.getMessage()
              if netmsg.msgtype == CLIENT_ROLE:
                msg.target = self
              print("Received Message {}: {} - {}".format(self.name(), VAL_TO_MSG[msg.msgtype], msg.msg))
              self._put(msg)
            else:
              print("Invalid Message {}: {}".format(self.name(), serialstream[:netmsgsize]))
          else:
            print("Corrupt Stream {}: {}".format(self.name(), serialstream[:netmsgsize]))
          serialstream = serialstream[netmsgsize:]
          msgstart = serialstream.find(STARTBYTE)
    except ConnectionResetError:
      pass
    finally:
      if self.outWorker.clientList:
        self.outWorker.clientList.remove(self)
      print("Client Disconnected: {}".format(self.address))

  def _put(self, msg):
    self.outWorker.queue.put(msg)

  def send(self, intmsg):
    if intmsg.msgtype == MSG_REQUEST:
      if self.sentlist[bytetoval(intmsg.msg)] is None:
        print("Bad Message Request {}: Count - {}".format(self.name(), bytetoval(intmsg.msg)))
        return
      else:
        intmsg = self.sentlist[bytetoval(intmsg.msg)].getMessage()
    if self.msgcount == MAX_MSG_COUNT:
      self.msgcount = 0
    else:
      self.msgcount += 1
    netmsg = NetMessage(
      source= intmsg.client,
      count= self.msgcount,
      msgtype= intmsg.msgtype,
      msgsize= len(intmsg.msg),
      msg= intmsg.msg,
    )
    # add message to message cache
    self.sentlist[netmsg.count] = netmsg
    self.client.sendall(netmsg.serialize())
