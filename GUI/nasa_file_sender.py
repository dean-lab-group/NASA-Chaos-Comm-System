# coding=utf-8
from zlib import crc32
import easygui as eg
import serial
import settings
from cobs import cobs

class NasaFileSender(object):
    def __init__(self, serial_obj):
        self.serial_obj = serial_obj


    def encode(self):
        pass

    def decode(self):
        pass

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

