from __future__ import print_function
import glob
import re

import serial
import sys

from settings import Settings


class SuperSerial(serial.Serial):
    def __init__(self, **kwargs):
        self.settings = Settings()
        super(SuperSerial, self).__init__(**kwargs)
        if 'baudrate' not in kwargs:
            self.baudrate = self.settings.serial_port_rate
        if 'port' not in kwargs:
            try:
                self.port = self.ports[-1]
            except IndexError:
                print("Could not find a suitable serial port.")
                raise
        #self.open()

    @property
    def ports(self):
        if sys.platform == 'darwin':
            temp_list = glob.glob('/dev/tty.[A-Za-z]*')
        elif sys.platform == 'linux':
            temp_list = glob.glob('/dev/tty.[A-Za-z]*')
        else:
            # TODO: How do we find available Windows COM ports?
            temp_list = None

        result = []
        for a_port in temp_list:
            try:
                if re.search(self.settings.serial_ignore_list, a_port, re.IGNORECASE):
                    #print("Ignoring" + a_port)
                    continue

                s = serial.Serial(a_port)
                if s.readable():
                    s.close()
                    result.append(a_port)
            except serial.SerialException:
                pass
        return result


if __name__ == '__main__':
    s = SuperSerial()
