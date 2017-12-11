# coding=utf-8
logo = 'NASA-logo.gif'
# serial_port = '/dev/tty.usbserial-A9014QOJ'
serial_port = '/dev/tty.usbmodem1413'
serial_port_rate = 9600
data_delim = '\x00'
title = "Chaos Communication GUI"
components = dict(send="Send File",
                       receive="Receive File",
                       chat = "Open Chat",
                       exit="Exit"
                       )

FILE_DATA_POS = 0
FILE_CHECKSUM_POS = 1


