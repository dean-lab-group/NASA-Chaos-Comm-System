# coding=utf-8
from zlib import crc32
from cobs import cobs

from serial_setup import SuperSerial
from settings import Settings


class FileSender(object):
    def __init__(self, serial_obj):
        self.connected = False
        self.settings = Settings()
        self.ser = serial_obj

    def send_file(self, file_path):
        if not self.ser.is_open():
            self.ser.open()
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

    def receive_file(self, file_path):
        # if not self.ser.is_open():
        #     self.ser.open()
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
        print file_path
        with open(file_path, 'w') as fh:
            fh.write(file_data[0])


if __name__ == '__main__':
    import sys
    sender = SuperSerial(port='/dev/tty.usbmodem1411')
    receiver = SuperSerial(port='/dev/tty.usbserial-FTG96HDJ')
    mode = sys.argv[1]
    filename = sys.argv[2]
    if mode == '--receive':
        fs = FileSender(receiver)
        fs.send_file(filename)
    elif mode == '--send':
        fs = FileSender(sender)
        fs.receive_file(filename)
    else:
        print sys.argv[0] + "--send|--receive [file name]"
        exit(-1)
