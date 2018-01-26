# coding=utf-8
from time import sleep
from zlib import crc32
from cobs import cobs
from tqdm import tqdm
from serial_setup import SuperSerial
from settings import Settings

class SoftFrame(object):


class FileSender(object):
    def __init__(self, serial_obj):
        self.SEND_DELAY = 0.5
        self.connected = False
        self.settings = Settings()
        self.ser = serial_obj
        self.progress = True

    def send_file(self, file_path):
        if not self.ser.is_open:
            self.ser.open()
        # Compile string and send data.
        print "Sending ", file_path
        with open(file_path, 'rb') as fh:
            file_data = fh.read()
            #checksum = crc32(file_data)
            #data = file_data + self.settings.data_delim + str(checksum) + self.settings.data_delim * 2
            #cobs_encoded = cobs.encode(data)
            #cobs_newlined = [x+'\n' for x in cobs_encoded]
            #data_size = len(cobs_encoded)
            data_size = len(file_data)
            #print "File size:", data_size, "bytes"
            #for my_byte in tqdm(cobs_encoded, total=data_size, unit='bytes'):
            if not self.progress:
                print "Disabling progress meter."
                tqdm.disable = True
            for my_byte in tqdm(file_data, total=data_size, unit='bytes'):
                self.ser.write(my_byte)
                self.ser.write('\n')
                sleep(self.SEND_DELAY)
            #eg.msgbox(msg='File sent', title='Data Sent', ok_button='(OK)')
            print

    def receive_file(self, file_path):
        if not self.ser.is_open:
            self.ser.open()
        # Puts program in receive mode, waiting for data on serial line.
        print("Listening")
        terminator = self.settings.data_delim * 2
        line = bytearray()
        lenterm = len(terminator)
        print "Terminator:", [ord(x) for x in terminator]
        while True:
            c = self.ser.read(1)
            print(c+",",)
            if c:
                line += c
                if line[-lenterm:] == terminator:
                    break
                #if size is not None and len(line) >= size:
                #    break
            else:
                break
            #if timeout.expired():
            #    break
        #return bytes(line)
        #
        exit()
        serial_data = self.ser.read_until(terminator=self.settings.data_delim * 2)
        print("Got data!")
        encoded_data = serial_data.rstrip(self.settings.data_delim * 2)
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
    mode = sys.argv[1]
    filename = sys.argv[2]
    if mode == '--receive':
        receiver = SuperSerial() #port='/dev/tty.usbserial-FTG96HDJ')
        fs = FileSender(receiver)
        fs.receive_file(filename)
    elif mode == '--send':
        sender = SuperSerial(port='COM5') #port='/dev/tty.usbmodem1413')
        fs = FileSender(sender)
        fs.send_file(filename)
    else:
        print sys.argv[0] + "(--send|--receive) [file name]"
        exit(-1)
