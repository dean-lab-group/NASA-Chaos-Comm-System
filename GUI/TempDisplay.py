# coding=utf-8

import Tkinter
from Tkinter import Tk, BOTH, Label
import threading
import tkFont
from serial_setup import SuperSerial
from parse_frame import Frame
from settings import Settings

my_set = Settings()
temperature = my_set.init_temp

top = Tk()
top.geometry(my_set.window_geometry)
top.title(my_set.window_title)
myLabel = Label(top, text=temperature)
myLabel.pack()

global_counter = 0
success = 0


class Looping(object):
    def __init__(self):
        # my_font = tkFont.Font(family='Helvetica', size=100, weight='bold')
        self.stop_threads = threading.Event()
        self.t = None
        self.B_start = Tkinter.Button(top, text="Start", command=self.button_start)
        self.B_start.pack(fill=BOTH, expand=0)
        self.B_stop = Tkinter.Button(top, text="Stop", command=self.button_stop)
        self.B_stop.pack(fill=BOTH, expand=0)
        self.isRunning = True
        self.s = SuperSerial()
        self.frame = Frame(self.s)
        self.temperature = None
        self.alpha = 0.5

    def is_connected(self):
        return self.s.is_open()

    def button_start(self):
        l.isRunning = True
        self.t = threading.Thread(target=l.loop_temps)
        self.t.start()

    def loop_temps(self):
        while not self.stop_threads.is_set():
            self._get_temperature()

    def _moving_average(self, new_temp):
        # Moving Average function
        ma_temp = self.alpha * new_temp + (1 - self.alpha) * self.temperature
        return ma_temp

    @staticmethod
    def _convert_temp(temperature_int):
        temp_c = (3.3 * temperature_int / 255.0) / 0.01
        return temp_c

    def _get_temperature(self):
        global temperature
        global global_counter
        global success
        global_counter += 1
        if not self.s.is_open:
            self.s.open()
        try:
            temperature = self.frame.frame_data_array
        except Exception:
            return None
        print "Received data:", repr(temperature)
        self.s.close()
        try:
            temperature = int(temperature)
            temperature = self._convert_temp(temperature)

            # Added this to initialize the average temperature close to the value its reporting so that it will
            # take less time to stabilize.
            if not self.temperature:
                self.temperature = temperature
                temperature = self._moving_average(temperature)
            self.temperature = temperature

            myLabel.config(text="%0.2f deg C" % temperature)
            success += 1
            print "(%0.2f%%) %0.2f deg C" % (100*success/float(global_counter), temperature)
        except ValueError as e:
            print ">>" + temperature + "<<", e
        except Exception as e:
            print e

    def button_stop(self):
        self.stop_threads.set()
        l.isRunning = False
        self.t.join(timeout=1)
        self.t = None
        exit()


l = Looping()
l.button_start()
top.mainloop()
