#!/usr/bin/env python

"""
A simple echo server
Taken from Brigham Young University
"""

import socket, select
import time
from threading import Thread
from common import *

clientList = []
data = b''  # <- data is used to be the bytes string of a full message
type = 0x00
objectType = 0x00
roverCommandType = 0x00
distance = 0
xPos = 0
yPos = 0
angleDegrees = 0
objectLength = 0
objectWidth = 0

# The counter for the number of messages sent and received
SEND_COUNT = 0
RECEIVE_COUNT = 0

"""
Message Structure:
        Bytes:                                              Content:
_______________________________________________________________________________________________
        0                                                   Start Byte
        1                                                   Sender
        2                                                   Message Number
        3                                                   Message Type
        4                                                   Message Size
        5-16                                                Message
17 (or anything after the last byte of Message)             End Byte
_______________________________________________________________________________________________
"""

# Converts a byte message into an internal message that can be used to send out a message
# Really this is just a fancy way of extracting data I want to keep a hold of in a class
def convertFromMessage(message):
    global RECEIVE_COUNT

    # Increment the send counter
    RECEIVE_COUNT += 1
    # The sender / client of the message
    client = message[1]
    # The message number for the number of messages this client has received
    RECEIVE_COUNT = message[2]
    # Message type
    messagetype = message[3]
    # Message size
    messagesize = message[4]
    msg = []
    # Takes the message as a bytes string
    for x in range (0, messagesize):
        try:
            msg.append(message[x + 5])
        except:
            pass

    # Creates an internal message for easy data extraction
    convertedMessage = InternalMessage(client, messagetype, msg)
    return convertedMessage

# Converts to a message that can be sent wirelessly
def convertToMessage(messageType, message):
    global SEND_COUNT

    # Client / Message sender
    sender = VISUALIZATION
    # The message number for the number of messages this client has sent
    messageNo = SEND_COUNT
    # Message type
    type = messageType
    # Message size
    msgSize = len(message)
    # Message without the start and end bytes
    incompleteMessage = bytes([sender, messageNo, type, msgSize]) + message

    # Increment the send counter
    SEND_COUNT += 1

    # Return the full message
    return START_BYTE + incompleteMessage + END_BYTE

# Function to convert a message to a list of data used for the rover command
# Returns in the following format: roverCommand, distance
# e.g. roverCommand = ROVER_LEFT, distance 120 <- degrees
# distance will always be a 16-bit value and roverCommand will always be an 8-bit value
def fromRoverMoveMessage(message):
    # Get the type of movement the rover needs to do
    roverCommand = message[0]

    # Get the actual distance / angle change that must be performed
    # Bit manipulation to get the correct distance
    # Distance on the PIC end is the size of a char
    distance = message[1]

    return roverCommand, distance

def fromObjectPositionMessage(message):
    # Picks up the whole message to return the data as a list
    messagesize = len(message)
    msg = []
    for x in range (0, messagesize):
        msg.append(message[x])

    objectType = msg[0]
    x = ((msg[1]) << 8) | msg[2]
    y = ((msg[3]) << 8) | msg[4]
    angle = ((msg[5]) << 8) | msg[6]
    length = ((msg[7]) << 8) | msg[8]
    width = ((msg[9]) << 8) | msg[10]

    return objectType, x, y, angle, length, width

# Function to convert a command for the rover to move into a bytes string to be
# available for immediate sending to the socket
def toRoverMoveMessage(roverCommand, distance):
    """
    Rover command is different from the message type 'ROVER_MOVE'
    Rover command contains one of the following types:
    __________________________________________________________
    ROVER_FORWARD       0x01
    ROVER_BACKWARD      0x02
    ROVER_LEFT          0x04
    ROVER_RIGHT         0x08
    ROVER_STOP          0x10
    """
    distanceHigh = (distance & 0xFF00) >> 8
    distanceLow = (distance & 0x00FF)
    return bytes([roverCommand, distanceHigh, distanceLow])

# Function to convert an object position to a bytes string to be available
# for immediate sending to the socket
def toObjectPositionMessage(objectType, x, y, angle, length, width):
    """
    For tokens and obstacles, the angle represents the total number of those
    objects because the angle does not matter
    """
    xHigh = (x & 0xFF00) >> 8
    xLow = (x & 0x00FF)
    yHigh = (y & 0xFF00) >> 8
    yLow = (y & 0x00FF)
    angleHigh = (angle & 0xFF00) >> 8
    angleLow = (angle & 0x00FF)
    lengthHigh = (length & 0xFF00) >> 8
    lengthLow = (length & 0x00FF)
    widthHigh = (width & 0xFF00) >> 8
    widthLow = (width & 0x00FF)
    return bytes([objectType, xHigh, xLow, yHigh, yLow, angleHigh,
                  angleLow, lengthHigh, lengthLow, widthHigh, widthLow])

def listening():
    global data
    global clientList
    global type
    global distance
    global objectType
    global roverCommandType
    global xPos
    global yPos
    global angleDegrees
    global objectLength
    global objectWidth

    # Socket parameters
    host = ''
    port = 59784
    backlog = 10
    size = 1024

    # Socket object
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Attempt to bind and listen to an IP address and port number
    s.bind((host,port))
    s.listen(backlog)
    clientList.append(s)
    while 1:
        # Get the list sockets which are ready to be read through select
        read_sockets,write_sockets,error_sockets = select.select(clientList,[],[])
        for sock in read_sockets:
            if sock == s:
                client, address = s.accept()

                # Checking if you connected to a new client
                clientList.append(client)
                print('New client at {}'.format(address))
            else:
                # Checking for a new message
                data = sock.recv(size)

                if data:
                    if data == b'*HELLO*':
                        pass
                    else:
                        # Need to deal with multiple messages in one string
                        intMessage = convertFromMessage(data)

                        # Printing out the messages in a readable format
                        if intMessage.msgtype == ROVER_MOVE:
                            roverCommandType, distance = fromRoverMoveMessage(intMessage.msg)
                            print('Receive: {} - {} : {} - {}'.format(VAL_TO_ROLE[intMessage.client],
                                                                      VAL_TO_MSG[intMessage.msgtype],
                                                                      MSG_TO_ROVERMOVE[roverCommandType],
                                                                      distance))
                        elif intMessage.msgtype == OBJECT_POS:
                            objectType, xPos, yPos,angleDegrees, objectLength, objectWidth = fromObjectPositionMessage(intMessage.msg)
                            print('Receive: {} - {} : {} - ({}, {}), {} degrees, length: {}, width: {}'.format(VAL_TO_ROLE[intMessage.client],
                                                                                                             VAL_TO_MSG[intMessage.msgtype],
                                                                                                             MSG_TO_OBJECT[objectType],
                                                                                                             xPos,
                                                                                                             yPos,
                                                                                                             angleDegrees,
                                                                                                             objectLength,
                                                                                                             objectWidth))
                        elif intMessage.msgtype == TOKEN_FOUND:
                            objectType, xPos, yPos, count, objectLength, objectWidth = fromObjectPositionMessage(intMessage.msg)
                            print('Receive: {} - {}, ({}, {}), count: {}, current token: {}, length/width: {}'.format(VAL_TO_ROLE[intMessage.client],
                                                                                                                      VAL_TO_MSG[intMessage.msgtype],
                                                                                                                      xPos,
                                                                                                                      yPos,
                                                                                                                      count,
                                                                                                                      objectLength,
                                                                                                                      objectWidth))

                        #Do not send the message to master socket and the client who has sent us the message
                        for sockets in clientList:
                            if sockets != s and sockets != sock :
                                try :
                                    sockets.send(data)
                                except :
                                    # broken socket connection may be, chat client pressed ctrl+c for example
                                    sockets.close()
                                    clientList.remove(sockets)

                """# A new message was received and hopefully the resource can be shared from
                # the listener thread almost simultaneously
                    print('Received ', data)

                    intMessage = convertFromMessage(data)
                    print(intMessage.client)
                    print(intMessage.msgtype)
                    print(intMessage.msg)

                    # Transferring the data to the appropriate global variables
                    if intMessage.msgtype == ROVER_MOVE:
                        roverCommandType, distance = fromRoverMoveMessage(intMessage.msg)
                        print(roverCommandType)
                        print(distance)
                    elif intMessage.msgtype == OBJECT_POS:
                        objectType, xPos, yPos,angleDegrees, objectLength, objectWidth = fromObjectPositionMessage(intMessage.msg)
                        print(objectType)
                        print(xPos)
                        print(yPos)
                        print(angleDegrees)
                        print(objectLength)
                        print(objectWidth)"""

def main():
    global data
    global clientList
    while 1:

        """# A new message was received and hopefully the resource can be shared from
        # the listener thread almost simultaneously
        if data:
            print('Received ', data)

            intMessage = convertFromMessage(data)
            print(intMessage.client)
            print(intMessage.msgtype)
            print(intMessage.msg)

            # Transferring the data to the appropriate global variables
            if intMessage.msgtype == ROVER_MOVE:
                roverCommandType, distance = fromRoverMoveMessage(intMessage.msg)
                print(roverCommandType)
                print(distance)
            elif intMessage.msgtype == OBJECT_POS:
                objectType, xPos, yPos,angleDegrees, objectLength, objectWidth = fromObjectPositionMessage(intMessage.msg)
                print(objectType)
                print(xPos)
                print(yPos)
                print(angleDegrees)
                print(objectLength)
                print(objectWidth)

            try:
                # Loops through all the clients to send the message
                for clients in clientList:
                    clients.send(data)
            except:
                pass"""

if __name__ == '__main__':
    # Starting the listening thread
    listeningThread = Thread(target=listening, daemon=True)
    listeningThread.start()

    # Starting the main function
    main()