
import serial
serial_device = "/dev/ttyACM0"
serial_baud = 9600

s = serial.Serial(port=serial_device, baudrate=serial_baud)
try:
    while True:
        print s.readline()
except KeyboardInterrupt:
    exit(0)


