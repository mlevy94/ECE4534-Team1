from configs import *  # too many variables to import explicitly
import threading


class ClientWorker:

  def __init__(self, client, queue, address=None, writefunc=None, readfunc=None):
    self.client = client
    if writefunc is None:
      self.writefunc = self.client.sendall
    else:
      self.writefunc = writefunc
    if readfunc is None:
        self.readfunc = self.client.recv
    else:
      self.readfunc = readfunc
    if address is None:
      self.address = client.getsockname()
    self.queue = queue
    self.thread = None
    self.address = address
    self.recvmsgcount = 0
    self.sentmsgcount = 0
    self.clientConnected = False
    self.lock = threading.Lock()

  def start(self):
    self.writefunc(bytes([0x50 for _ in range(30)]))
    self.clientConnected = True
    self.send(InternalMessage(ROUTER, INITIALIZE, b'111111111111', self))
    self.thread = threading.Thread(target=self._clientRecv, daemon=True)
    self.thread.start()


  def _clientRecv(self):
    try:
      serialstream = b''
      while 1:
        serialstream += self.readfunc(4096)
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
          while len(serialstream) < msgsize + 1:
            serialstream += self.readfunc(4096)
          msg, serialstream = serialstream[:msgsize], serialstream[msgsize:]
          netmsg = NetMessage(
            source= source,
            count = count,
            msgtype= msgtype,
            msgsize = msgsize,
            msg=msg,
          )
          if len(serialstream) > 1:
            endbyte, serialstream = serialstream[0], serialstream[1:]
          else:
            endbyte = serialstream[0]
            serialstream = b''
          if endbyte != ENDBYTE[0]:
            print("Bad End Byte: {}".format(self.address))
            continue
          # place in outbound queue
          msg = netmsg.getMessage()
          if netmsg.msgtype == CLIENT_ROLE:
            msg.target = self
          if DEBUG_ON:
            print("Received Message {}: {} - {}".format(self.address, VAL_TO_MSG[msg.msgtype], msg.msg))
          self._put(msg)
          if self.recvmsgcount < MAX_MSG_COUNT:
            self.recvmsgcount += 1
          else:
            self.recvmsgcount = 0
    except (ConnectionError, TimeoutError):
      pass

  def _put(self, msg):
    self.queue.put(msg)

  def send(self, intmsg):
    with self.lock:
      if not self.clientConnected:
        return False
      if self.sentmsgcount < MAX_MSG_COUNT:
        self.sentmsgcount += 1
      else:
        self.sentmsgcount = 0
      if intmsg.count is not None:
        msgcount = intmsg.count
      else:
        msgcount = self.sentmsgcount
      netmsg = NetMessage(
        source= intmsg.client,
        count= msgcount,
        msgtype= intmsg.msgtype,
        msgsize= len(intmsg.msg),
        msg= intmsg.msg,
      )
      self.writefunc(netmsg.serialize())
      return True
