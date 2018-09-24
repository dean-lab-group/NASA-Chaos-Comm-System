from __future__ import print_function

import datetime
import random
import string
import time

from serial_setup import SuperSerial

# Set these up with the correct ports, e.g. COM2, COM3
# s: sender
#s = SuperSerial(port='/dev/tty.usbmodem1423')
s = SuperSerial(port='COM6')

# r: receiver
#r = SuperSerial(port='/dev/tty.usbmodem1413')
r = SuperSerial(port='COM8')
temperature = SuperSerial(port='COM10', baudrate=115200)

fh = open('BER_Test.csv', 'w')
fh.write("Date, Sent Letter, Received Letter (escaped), Status, Temperature")
total = 1
success = 0
while True:
    date_now = datetime.datetime.now()
    my_temp = temperature.readline().strip()
    s_letter = random.choice(string.letters)
    s_binary = ' '.join(format(ord(x), 'b') for x in s_letter)
    s.write(s_letter)
    s.write('\n')
    time.sleep(0.001)
    r_letter = r.read()
    r_binary = ' '.join(format(ord(x), 'b') for x in r_letter)
    if r_letter == s_letter:
        status = 1
        success = success + 1
    else:
        status = 0
    send_string = '%s, %s, %s, %d, %s' % (date_now, s_letter, repr(r_letter), status, my_temp)
    print(send_string)
    print("Rate: %0.2f%%" % float(100*success/total))
    fh.write(send_string)
    fh.write('\n')
    fh.flush()
    total = total+1
    time.sleep(0.5)
