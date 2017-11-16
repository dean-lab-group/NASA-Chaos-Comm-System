# coding=utf-8
from threading import Thread
from serial import Serial


class Receiver(Thread):
    def __init__(self, serial_port):
        Thread.__init__(self)
        self.serialPort = serial_port

    def run(self):
        text = ""
        while (text != "exit\n"):
            text = serialPort.readline()
            print ("\n Machine 1: " + text)
        self.serialPort.close()


class Sender(Thread):
    def __init__(self, serial_port):
        Thread.__init__(self)
        self.serialPort = serial_port

    def run(self):
        text = ""
        while (text != "exit\n"):
            text = raw_input(">>") + "\n"
            self.serialPort.write(text)
        self.serialPort.close()


class ChaTTY(object):
    def __init__(self, serial_port, *args, **kwargs):
        self.serial_port = serial_port
        try:
            self.serial_port = Serial(serial_port)
        except Exception as e:
            print e
            exit(-1)

        send = Sender(self.serial_port)
        receive = Receiver(self.serial_port)
        send.start()
        receive.start()



