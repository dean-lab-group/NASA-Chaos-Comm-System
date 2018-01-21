# coding=utf-8

import Tkinter
from Tkinter import Tk, BOTH, Label
import threading
import tkFont
from serial_setup import SuperSerial

from settings import Settings
my_set = Settings()

temperature = my_set.init_temp

top = Tk()
top.geometry(my_set.window_geometry)
top.title(my_set.window_title)
myLabel = Label(top, text=temperature)
myLabel.pack()


class Looping(object):
    def __init__(self):
        # my_font = tkFont.Font(family='Helvetica', size=100, weight='bold')
        self.B_start = Tkinter.Button(top, text="Start", command=self.button_start)
        self.B_start.pack(fill=BOTH, expand=0)
        self.B_stop = Tkinter.Button(top, text="Stop", command=self.button_stop)
        self.B_stop.pack(fill=BOTH, expand=0)
        self.isRunning = True
        self.s = SuperSerial()
        self.temperature = None
        self.alpha = 0.5


    def isConnected(self):
        return self.ser.is_open()

    @staticmethod
    def button_start():
        l.isRunning = True
        t = threading.Thread(target=l.get_temperature)
        t.start()

    def ma(self, new_temp):
        ma_temp = self.alpha * new_temp + (1 - self.alpha) * self.temperature
        print ma_temp
        return ma_temp

    def convert_temp(self, temperature_int):
        print "temp_int", temperature_int
        temp_c = (3.3 * temperature_int / 255.0) / 0.01
        print temp_c
        return temp_c

    def get_temperature(self):
        global temperature
        while self.isConnected:
            temperature = self.ser.readline()
            try:
                temperature = int(temperature.strip())
                temperature = self.convert_temp(temperature)

                # Added this to intialize the average temperature close to the value its reporting so that it will
                # take less time to stablilize.
                if not self.temperature:
                    self.temperature = temperature
                temperature = self.ma(temperature)
                self.temperature = temperature
                myLabel.config(text="%0.2f deg C" % temperature)
            except ValueError as e:
                pass
                # print ">>"+temperature+"<<", e
        else:
            print "Not connected to the communications system."

    def button_stop(self):
        l.isRunning = False
        exit()


l = Looping()
top.mainloop()
