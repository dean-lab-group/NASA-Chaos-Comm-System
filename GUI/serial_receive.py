import string

from parse_frame import Frame
from serial_setup import SuperSerial

receiver = SuperSerial(port='COM4')
#fh = open('test_data_received.txt', 'w')
#fr = Frame(receiver)
while True:
    try:
        for i in xrange(20):
            my_in = receiver.read()
            if my_in in string.printable:
                print my_in,
            else:
                print repr(my_in),
        print
        #frame_array = fr.frame_data_array
        #print frame_array
        #print repr(recv)
        #fh.write(recv)
        #fh.flush()
    except KeyboardInterrupt:
        print "Closing files"
        break

receiver.close()
#fh.close()


