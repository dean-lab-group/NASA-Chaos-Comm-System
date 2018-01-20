#!/usr/bin/evn python
import serial
# The following line will vary between OS and environment
ser = serial.Serial('/dev/tty.usbmodem1413', 9600, timeout=1)

while 1: # go forever
     line = ser.readline() # read serial data until you encounter a newline.
     print line
     if(line != ''):       # make sure there is something read from the buffer.
         mylist = line.split(',') # create an array from line splitting every comma.
         print mylist # print array
     #### end if
#### end while

##stupid Windows!
# Markus concurs!
