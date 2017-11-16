#!/usr/bin/env python
# coding=utf-8
from zlib import crc32
import easygui as eg
import serial
import settings
from cobs import cobs

try:
    # Python2
    from Tkinter import *
    from tkFileDialog import askopenfilename, asksaveasfilename
except ImportError:
    # Python3
    from tkinter import *


class CommGui(object):
    def __init__(self, serial_port=None, *args, **kwargs):
        self.set = settings
        if serial_port:
            self.serial_port = serial_port
        else:
            self.serial_port = self.set.serial_port
        # The following line will vary between OS and environment. Set it as
        # appropriate.
        try:
            if 'serial_port_rate' in kwargs:
                self.serial_port_rate = kwargs['serial_port_rate']
                self.ser = serial.Serial(self.serial_port, self.serial_port_rate, timeout=1)
            else:
                self.ser = serial.Serial(self.serial_port, timeout=1)
        except serial.serialutil.SerialException as e:
            msg = "Couldn't connect to serial device!"
            print(msg)
            # eg.msgbox(msg)
            exit(-1)

        while True:
            reply = self.entry()
            if reply == self.set.components["send"]:
                file_path = askopenfilename()
                print(file_path)
                self.send_file(file_path)
            elif reply == self.set.components['receive']:
                self.receive_file()
            elif reply == self.set.components['chat']:
                self.chat()
            else:
                exit()

    def entry(self):
        # Entry function will loop forever unless it it is quit.
        image = settings.logo
        return eg.buttonbox(image=image, title=self.set.title, choices=self.set.components.values())

    def chat(self):
        import ChaTTY
        ChaTTY(self.serial_port)

    def send_file(self, file_path):
        # Compile string and send data.
        print "Sending ", file_path
        with open(file_path, 'rb') as fh:
            file_data = fh.read()
            checksum = crc32(file_data)
            cobs_encoded = cobs.encode(file_data + self.set.data_delim + checksum + self.set.data_delim * 2)

            # I'm not sure if Aaron's uC code will be able to handle this much data. I may need to only send
            # him x number of bytes at a time.
            self.ser.write(cobs_encoded)
            eg.msgbox(msg='File sent', title='Data Sent', ok_button='(OK)')

    def receive_file(self):
        # Puts program in receive mode, waiting for data on serial line.
        encoded_data = self.ser.read_until(terminator=self.set.data_delim * 2).rstrip(self.set.data_delim * 2)
        # We split the incoming data into two parts. The '1' below is the maxsplit parameter which means we're splitting
        # it into 2.
        file_data = cobs.decode(encoded_data).split(self.set.data_delim, 1)
        checksum = crc32(file_data[self.set.FILE_DATA_POS])
        if checksum != file_data[self.set.FILE_CHECKSUM_POS]:
            print('Data was corrupted.')
            return
        print("Received file.")
        file_path = asksaveasfilename()
        print file_path
        with open(file_path, 'w') as fh:
            fh.write(file_data[0])


if __name__ == '__main__':
    gui = CommGui()
