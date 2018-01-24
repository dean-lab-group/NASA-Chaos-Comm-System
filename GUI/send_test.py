import sys
from file_sender import FileSender
from serial_setup import SuperSerial
filename = 'alphanumeric.txt'
sender = SuperSerial(port='COM5')
fs = FileSender(sender)
while True:
    fs.send_file(filename)
