# coding=utf-8
import serial
from tkinter import *
import random
import tkinter.font


class Application(Frame):
    def __init__(self, master=None):
        print "App __init__"
        Frame.__init__(self, master)
        self.pack()
        self.temp_label = Label(root, text="NONE", fg="black", bg="white", font="36")
        self.temp_label.grid(row=0, column=0)
        self.temp_label.pack()

        self.QUIT = Button(self)
        self.QUIT['text'] = "EXIT"
        self.QUIT['command'] = self.quit
        self.QUIT.pack()
        self.after(1000, self.update_temperature)

    def update_temperature(self):
        print "update_temperature"
        temp = self.get_temperature() * 9 / 5.0 + 32
        self.temp_label.configure(text=str(temp) + " deg C")
        self.after(1000, self.update_temperature)

    def get_temperature(self):
        try:
            # Replace with appropriate serial port.
            self.serial_port = "/dev/tty.usbmodem1423"
            self.ser = serial.Serial(self.serial_port, timeout=1)

        except Exception as e:
            pass

        print "Getting temp"
        return random.random()


root = Tk()
root.title("Temperature")
app = Application(master=root)
app.mainloop()
root.destroy()
