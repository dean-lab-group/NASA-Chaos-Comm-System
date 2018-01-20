from __future__ import print_function
import glob
import serial
from settings import Settings


class SerialSetup(serial.SerialBase):
    def __init__(self, **kwargs):
        super(SerialSetup, self).__init__(**kwargs)
        self.settings = Settings()
        if 'port' not in kwargs:
            self.port = self.list_ports()[0]
        if 'baudrate' not in kwargs:
            self.baudrate = self.settings.serial_port_rate

    def list_ports(self):
        temp_list = glob.glob('/dev/tty[A-Za-z]*')
        result = []
        for a_port in temp_list:
            try:
                s = serial.Serial(a_port)
                s.close()
                result.append(a_port)
            except serial.SerialException:
                pass
        return result


if __name__ == '__main__':
    s = SerialSetup()
    print(s.list_ports())
    print(s.port)
    print(s.baudrate)
    print(s.is_open)
    #print(s.read(), end='')
