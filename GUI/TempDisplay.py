# coding=utf-8

import Tkinter
from Tkinter import Tk, BOTH, Label
import threading
import tkFont

import serial

from GUI import settings

temperature = "asdf"
top = Tk()
top.geometry("600x600")
top.title("Sensor Temperature")
myLabel = Label(top, text=temperature)
myLabel.pack()


class Looping(object):
    def __init__(self):
        #my_font = tkFont.Font(family='Helvetica', size=100, weight='bold')
        self.B_start = Tkinter.Button(top, text="Start", command=self.button_start)
        self.B_start.pack(fill=BOTH, expand=0)
        self.B_stop = Tkinter.Button(top, text="Stop", command=self.button_stop)
        self.B_stop.pack(fill=BOTH, expand=0)
        self.isRunning = True
        self.ser = serial.Serial(settings.serial_port, 9600, timeout=1)

    def isConnected(self):
        return self.ser.is_open()

    @staticmethod
    def button_start():
        l.isRunning = True
        t = threading.Thread(target=l.get_temperature)
        t.start()

    def get_temperature(self):
        global temperature
        while self.isConnected:
            temperature = self.ser.readline()
            try:
                temperature = int(temperature.strip())
                temperature = (3.3 * temperature/255.0) / 0.01
                myLabel.config(text = "%0.2f deg C" % temperature)
            except ValueError as e:
                pass
                #print ">>"+temperature+"<<", e
        else:
            print "Not connected to the communications system."

    def button_stop(self):
        l.isRunning = False
        exit()


l = Looping()
top.mainloop()
