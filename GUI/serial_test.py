from __future__ import print_function

from serial_setup import SuperSerial

s = SuperSerial()

try:
    while True:
        char = s.read()
        print(char, end='')
except KeyboardInterrupt:
    exit(0)
