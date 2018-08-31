import sys
from parse_frame import Frame
from serial_setup import SuperSerial

if sys.platform == 'darwin':
    MY_PORT = '/dev/tty.usbmodem1413'
else:
    MY_PORT = 'COM5'

receiver = SuperSerial(port=MY_PORT)
fh = open(sys.argv[1], 'w')
fr = Frame(receiver)

while True:
    try:
        frame_array = fr.frame_data_array
        if frame_array:
            sys.stdout.write(frame_array)
        else:
            sys.stdout.write('*')
        fh.write(recv)
        fh.flush()
    except KeyboardInterrupt:
        print "Closing files"
        break

receiver.close()
fh.close()


