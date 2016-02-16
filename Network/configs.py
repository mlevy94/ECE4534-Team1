# This file contains the configurations neccessary to operate this in different environments.
from socket import gethostbyname, gethostname;


def getIPAddr():
  return gethostbyname(gethostname())

def getPort():
  return 56677

STARTBYTE = b'\SOH'
ENDBYTE = b'\ACK'
