#!/usr/bin/env python
 
import sys
import serial

port='/dev/serial/by-id/usb-Intel_GENUINO_101_AE6771SQ60900BE-if00'
if sys.argv[1:]:
    port=sys.argv[1]

ser = serial.Serial() #open serial connection
ser.port=port #setting the port accordingly
 
#in case your usual baudrate isn't 9600 reset will not work, therefore we will open a resetable connection
#thanks to mattvenn.net for suggesting to add this step!
ser.baudrate=9600
ser.open(); ser.close()
 
ser.baudrate=1200 # set the reset baudrate
ser.open(); ser.close()
#don't forget to sleep some reset time
