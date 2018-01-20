# coding=utf-8

class Settings(object):
    def __init__(self):
        self.window_title = "Sensor Temperature"
        self.window_geometry = "600x600"
        self.init_temp = 'NA'
        self.logo = 'NASA-logo.gif'
        self.serial_port_rate = 9600
        self.data_delim = '\x00'
        self.title = "Chaos Communication GUI"

        self.components = dict(send="Send File",
                               receive="Receive File",
                               chat="Open Chat",
                               exit="Exit"
                               )

        self.FILE_DATA_POS = 0
        self.FILE_CHECKSUM_POS = 1
