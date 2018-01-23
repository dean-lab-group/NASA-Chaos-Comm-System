import serial

receiver = serial.Serial(port='COM7')
fh = open('test_data_received.txt', 'w')

while True:
    try:
        recv = receiver.read(1)
        print recv
        fh.write(recv)
        fh.flush()
    except KeyboardInterrupt:
        print "Closing files"
        break

receiver.close()
fh.close()


