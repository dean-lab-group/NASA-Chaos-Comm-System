from __future__ import print_function
import glob
import re

import serial
import sys
import serial.tools.list_ports
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
        result = []
        for n, (portname, desc, hwid) in enumerate(sorted(serial.tools.list_ports.comports())):
            if re.search(self.settings.serial_ignore_list, portname, re.IGNORECASE):
                #print("Ignoring" + a_port)
                continue
            else:
                result.append(portname)
        return result

if __name__ == '__main__':
    s = SuperSerial()
