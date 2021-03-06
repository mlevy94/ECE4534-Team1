#!/usr/bin/env python
import serial
import time

"""
Scan for serial ports.

Part of pySerial (http://pyserial.sf.net)
(C) 2002-2003 <cliechti@gmx.net>

The scan function of this module tries to open each port number
from 0 to 255 and it builds a list of those ports where this was
successful.
"""
def scan():
    """scan for available ports. return a list of tuples (num, name)"""
    available = []
    for i in range(256):
        try:
            s = serial.Serial('COM' + str(i))
            available.append( (i, s.name))
            s.close()   # explicit close 'cause of delayed GC in java
        except serial.SerialException:
            pass
    return available

if __name__=='__main__':
    print("Found ports:")
    for n,s in scan():
        print("(%d) %s" % (n,s))

    print('Current Port being used: ')
    print(s)
    # A clear assumption will be made that the processor manufacturer is Intel(R)
    usb = serial.Serial(s, baudrate=57600, timeout=5)

    usb.write(b'$$$')
    print('Setting to CMD mode')
    time.sleep(3)

    # Factory resetting the WiFly module
    print('Creating a factory reset')
    usb.write(b'factory RESET\r')
    time.sleep(3)
    print('Saving the Factory Reset')
    usb.write(b'save\r')
    time.sleep(3)
    print('Rebooting the device')
    usb.write(b'reboot\r')
    time.sleep(5)
    # Close to reconnect at the new Baud rate
    usb.close()
    print('Reopening the port at a Baud rate of 9600 bits/second')
    usb = serial.Serial(s, baudrate=9600, timeout=5)
    time.sleep(3)

    # Return to CMD mode
    print('Returning to CMD mode')
    usb.write(b'$$$')
    time.sleep(3)

    # Changing the Baud rate
    print('Setting the Baud rate to 57600 bits/second')
    usb.write(b'set uart baud 57600\r')
    time.sleep(3)
    print('Saving settings for new Baud rate')
    usb.write(b'save\r')
    time.sleep(3)
    print('Rebooting the device')
    usb.write(b'reboot\r')
    time.sleep(5)

    # Reconnecting the higher Baud rate
    usb.close()
    print('Reopening the port at the Baud rate of 57600 bits/second')
    usb = serial.Serial(s, baudrate = 57600, timeout=5)

    # Return to CMD mode
    print('Returning to CMD mode')
    usb.write(b'$$$')
    time.sleep(3)

    # Complete reconfiguration
    print('Enabling AP mode')
    usb.write(b'set wlan join 7\r')
    time.sleep(3)
    print('Setting the SSID to "4534_team1"')
    usb.write(b'set apmode ssid 4534_team1\r')
    time.sleep(3)
    print('Setting the pass phrase to be "team1rpi"')
    usb.write(b'set apmode passphrase team1rpi\r')
    time.sleep(3)
    print('Setting up open authentication mode')
    usb.write(b'set wlan channel 0\r')
    time.sleep(3)
    print('Setting the IP address to 192.168.0.1')
    usb.write(b'set ip address 192.168.0.1\r')
    time.sleep(3)
    print('Setting the IP gateway to 192.168.0.1')
    usb.write(b'set ip gateway 192.168.0.1\r')
    time.sleep(3)
    print('Reconfiguring the IP gateway to 255.255.255.0')
    usb.write(b'set ip gateway 255.255.255.0\r')
    time.sleep(3)
    print('Enabling the DHCP server')
    usb.write(b'set ip dhcp 4\r')
    time.sleep(3)
    print('Setting the IP port to 56677')
    usb.write(b'set ip localport 56677\r')
    time.sleep(3)
    print('Setting up the device id to be "team1_ap"')
    usb.write(b'set opt deviceid team1_ap\r')
    time.sleep(3)
    print('Saving the settings')
    usb.write(b'save\r')
    time.sleep(3)
    print('Rebooting the device')
    usb.write(b'reboot\r')