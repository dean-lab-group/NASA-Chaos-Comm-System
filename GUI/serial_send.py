from time import sleep
import serial
import string
import random
from util import string2bits

sender = serial.Serial(port='COM5')

# chars = string.ascii_letters + string.digits
chars = string.printable

fh = open('test_data_sent.txt', 'w')


def send():
    letter = random.choice(chars)
    print "Sending:  %s - %s" % (letter, ''.join(string2bits(letter)))
    fh.write(letter)
    fh.flush()
    sender.write(letter)
    sender.write('\n')
    sleep(1)


try:
    while True:
        send()
except KeyboardInterrupt:
    fh.close()
    sender.close()
    exit(0)
