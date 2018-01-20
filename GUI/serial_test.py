from __future__ import print_function
import glob
import serial


def list_ports():
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


ports = list_ports()
print("We found these ports:", ", ".join(ports))
print("Selecting first port:", ports[0])

serial_device = ports[0]
serial_baud = 9600

s = serial.Serial(port=serial_device, baudrate=serial_baud)
try:
    while True:
        char = s.read()
        print(char, end='')
except KeyboardInterrupt:
    exit(0)
