import string

import sys

from parse_frame import Frame
from serial_setup import SuperSerial

if sys.platform == 'darwin':
    MY_PORT = '/dev/tty.usbmodem1423'
else:
    MY_PORT = 'COM4'

receiver = SuperSerial(port=MY_PORT)
#fh = open('test_data_received.txt', 'w')
fr = Frame(receiver)

fr.DATA_DELIM = "%s%c%s" % ('\n', 0, '\n')
fr.FRAME_END = bytearray("%c%s%c%s" % (0, '\n', 0, '\n'))
while True:
    try:
        frame_array = fr.frame_data_array
        if frame_array:
            sys.stdout.write(frame_array)
        else:
            sys.stdout.write('*')
        #fh.write(recv)
        #fh.flush()
    except KeyboardInterrupt:
        print "Closing files"
        break

receiver.close()
#fh.close()


