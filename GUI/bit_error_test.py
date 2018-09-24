from __future__ import print_function

import datetime
import random
import string
import time

from serial_setup import SuperSerial

# Set these up with the correct ports, e.g. COM2, COM3
# s: sender
s = SuperSerial(port='/dev/tty.usbmodem1423')

# r: receiver
r = SuperSerial(port='/dev/tty.usbmodem1413')

fh = open('BER_Test.csv', 'w')
fh.write("Date, Sent Letter, Received Letter (escaped), Status")
while True:
    s_letter = random.choice(string.letters)
    fh.write(str(datetime.datetime.now()) + ", ")
    fh.write(s_letter + ", ")
    s.write(s_letter)
    s.write('\n')
    r_letter = r.read()
    print(s_letter, ' '.join(format(ord(x), 'b') for x in s_letter))
    print(r_letter, ' '.join(format(ord(x), 'b') for x in r_letter))
    print()
    fh.write(repr(r_letter) + ", ")
    if r_letter == s_letter:
        fh.write("0, ")
    else:
        fh.write("1, ")
    fh.write('\n')
    fh.flush()
    time.sleep(1)
