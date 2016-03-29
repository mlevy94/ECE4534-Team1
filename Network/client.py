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
    self.rcvmsgcount = 0
    self.sentmsgcount = 0
    self.clientConnected = False
    self.lock = threading.Lock()

  def start(self):
    self.writefunc(bytes([0x50 for _ in range(30)]))
    self.clientConnected = True
    self.send(InternalMessage(ROUTER, INITIALIZE, b'1', self))
    self.thread = threading.Thread(target=self._clientRecv, daemon=True)
    self.thread.start()


  def _clientRecv(self):
    try:
      serialstream = b''
      while 1:
        serialstream += self.readfunc(4096)
        while len(serialstream) > HEADER_SIZE:
          startbyte = serialstream.find(STARTBYTE)
          if startbyte < 0 or len(serialstream) < HEADER_SIZE:
            # keep looking for start byte or wait for more data
            break
          else:
            # trim extra bytes in beginning
            serialstream = serialstream[startbyte:]
          # get header values
          (startbyte, source, count, msgtype, msgsize), serialstream = (serialstream[:HEADER_SIZE], serialstream[HEADER_SIZE:])
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
            print("Bad End Byte: {}".format(self.address))
            continue
          # place in outbound queue
          msg = netmsg.getMessage()
          if netmsg.msgtype == CLIENT_ROLE:
            msg.target = self
          #print("Received Message {}: {} - {}".format(self.address, VAL_TO_MSG[msg.msgtype], msg.msg))
          self._put(msg)
          self.rcvmsgcount += 1
    except ConnectionResetError:
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
      netmsg = NetMessage(
        source= intmsg.client,
        count= self.sentmsgcount,
        msgtype= intmsg.msgtype,
        msgsize= len(intmsg.msg),
        msg= intmsg.msg,
      )
      self.writefunc(netmsg.serialize())
      return True
