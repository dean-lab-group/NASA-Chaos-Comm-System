#!/usr/bin/env python
try:
    # Python2
    from Tkinter import *
except ImportError:
    # Python3
    from tkinter import *


def key(event):
    """shows key or tk code for the key"""
    if event.keysym == 'Escape':
        root.destroy()
    if event.char == event.keysym:
        # normal number and letter characters
        print('Normal Key %r' % event.char)
    elif len(event.char) == 1:
        # charcters like []/.,><#$ also Return and ctrl/key
        print('Punctuation Key %r (%r)' % (event.keysym, event.char))
    else:
        # f1 to f12, shift keys, caps lock, Home, End, Delete ...
        print('Special Key %r' % event.keysym)


class Application(Frame):
    def say_hi(self):
        print "Manual Steering Control"

    def createWidgets(self):
        self.QUIT = Button(self)
        self.QUIT["text"] = "QUIT"
        # self.QUIT["fg"]   = "red"
        self.QUIT["command"] = self.quit

        self.QUIT.pack({"side": "left"})

        self.hi_there = Button(self)
        self.hi_there["text"] = "Steer",
        self.hi_there["command"] = self.say_hi

        self.hi_there.pack({"side": "left"})

    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.pack()
        self.createWidgets()


root = Tk()
root.bind_all('<Key>', key)
app = Application(master=root)
app.mainloop()
root.destroy()
