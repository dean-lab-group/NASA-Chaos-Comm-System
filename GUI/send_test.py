import sys
from file_sender import FileSender
from serial_setup import SuperSerial
filename = 'shakespeare.txt'
sender = SuperSerial(port='/dev/tty.usbmodem1411')
fs = FileSender(sender)
while True:
    fs.send_file(filename)
