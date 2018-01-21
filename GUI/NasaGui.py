#!/usr/bin/env python
# coding=utf-8
from zlib import crc32
from cobs import cobs

from ChaTTY import ChaTTY
from settings import Settings
from serial_setup import SuperSerial

import easygui as eg
try:
    # Python2
    from Tkinter import *
    from tkFileDialog import askopenfilename, asksaveasfilename
except ImportError:
    # Python3
    from tkinter import *


class CommGui(object):
    def __init__(self, serial_port=None, **kwargs):
        self.settings = Settings()
        self.serial_port = serial_port
        self.connected = False
        # The following line will vary between OS and environment. Set it as
        # appropriate.
        if 'serial_port_rate' in kwargs:
            self.ser.baudrate = kwargs['serial_port_rate']

        while True:
            reply = self.entry()
            if reply == self.settings.components["send"]:
                file_path = askopenfilename()
                print(file_path)
                self.send_file(file_path)
            elif reply == self.settings.components['receive']:
                self.receive_file()
            elif reply == self.settings.components['chat']:
                self.chat()
            elif reply == self.settings.components['connect']:
                self._connect()
            else:
                self._disconnect()
                exit()

    def _disconnect(self):
        self.ser.close()
        self.connected = False

    def _connect(self):
        try:
            if self.serial_port:
                self.ser = SuperSerial(port=self.serial_port)
            else:
                self.ser = SuperSerial()
            self.ser.open()
            self.connected = True
        except Exception as e:
            self.connected = False
            msg = "Couldn't connect to serial device!"
            print(msg)

    def entry(self):
        # Entry function will loop forever unless it it is quit.
        image = self.settings.logo
        return eg.buttonbox(image=image, title=self.settings.title, choices=self.settings.components.values())

    def chat(self):
        if not self.connected:
            self._connect()
        ChaTTY(self.ser)

    def send_file(self, file_path):
        if not self.connected:
            self._connect()
        # Compile string and send data.
        print "Sending ", file_path
        with open(file_path, 'rb') as fh:
            file_data = fh.read()
            checksum = crc32(file_data)
            data = file_data + self.settings.data_delim + str(checksum) + self.settings.data_delim * 2
            cobs_encoded = cobs.encode(data)

            # I'm not sure if Aaron's uC code will be able to handle this much data. I may need to only send
            # him x number of bytes at a time.
            self.ser.write(cobs_encoded)
            eg.msgbox(msg='File sent', title='Data Sent', ok_button='(OK)')

    def receive_file(self):
        if not self.connected:
            self._connect()
        # Puts program in receive mode, waiting for data on serial line.
        encoded_data = self.ser.read_until(terminator=self.settings.data_delim * 2).rstrip(self.settings.data_delim * 2)
        # We split the incoming data into two parts. The '1' below is the maxsplit parameter which means we're splitting
        # it into 2.
        file_data = cobs.decode(encoded_data).split(self.settings.data_delim, 1)
        checksum = crc32(file_data[self.settings.FILE_DATA_POS])
        if checksum != file_data[self.settings.FILE_CHECKSUM_POS]:
            print('Data was corrupted.')
            return
        print("Received file.")
        file_path = asksaveasfilename()
        print file_path
        with open(file_path, 'w') as fh:
            fh.write(file_data[0])


if __name__ == '__main__':
    gui = CommGui()
