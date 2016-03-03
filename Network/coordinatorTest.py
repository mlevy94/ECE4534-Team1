#!/usr/bin/env python
import serial
from client import ClientWorker
from queue import Queue
from threading import Thread
from rover import rover
from configs import *

# Opening USB serial communication to port "COM5"
# The Baud Rate is 57600 bits / second
# No timeout occurs to provide continual blocking until the correct amount of bytes are received
# Updated: Timeout will occur after 3 seconds to account for a readline (not knowing the size of the message)
usb = serial.Serial(port='COM9', baudrate=57600)

# Create a message queue for the serial communication
usbQ = Queue()

# Instantiating a rover object
#simulation = rover()

# Instantiation of a whole message
#message = []

# Print out what port is in use
print('Port: ' + usb.name)
# Building a new client worker object
usbClient = ClientWorker(usb, usbQ, usb.name, usb.write, usb.readline)

print('Client built')
# Start the client worker object thread
usbClient.start()

print('Client thread started')
# Look to receive a message <- Blocking
outmsg = InternalMessage(CLIENT, DEBUG_MSG, b'hello')
def outfunc():
    try:
        while True:
            usbClient.send(outmsg)
    except ConnectionError:
        pass
outthread = Thread(target=outfunc, daemon=True)
outthread.start()
while True:
    msg = usbQ.get()
    print(msg.msg)


# Close port
usb.close()