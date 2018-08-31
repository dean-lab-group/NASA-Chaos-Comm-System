import struct
from collections import OrderedDict

from serial_setup import SuperSerial


class Frame(object):
    def __init__(self, serial_buffer_obj):
        self.s = serial_buffer_obj
        self.FRAME_END = '\0\0'
        self.DATA_DELIM = '\0'
        # The data in the frame starts at 0 but ends at the next to last entry in the array. So -1 will specifies that.
        self.DATA_BEGIN = 0
        self.DATA_END = -1
        # Frame Info:
        # Regex: '\d+\0\d+\0\d+0\\0\0'
        # Sample Data: '18\0\18\018\0\0'

    def _decode_frame(self):
        buf = self.s.read_until(self.FRAME_END)
        frame = buf.split(self.DATA_DELIM)[self.DATA_BEGIN:self.DATA_END]
        #frame = [x.strip() for x in frame] # Strips newlines
        # We should only receive three elements in each received array. More than three indicates corrupt data.
        if len(frame) == 4:
            # We remove duplicates from the array of three. If all three are not the same, the data is corrupt.
            if len(set(OrderedDict.fromkeys(frame))) == 2:
                return frame[0]
            else:
                print "Inconsistent data", repr(buf), frame
        else:
            print "Corrupt data", repr(buf), frame

    @property
    def frame_data_array(self):
        return self._decode_frame()

    def __str__(self):
        return str(self._decode_frame())


class FrameException(BaseException):
    """
    Frame is corrupt
    """


if __name__ == '__main__':
    ser = SuperSerial()  # port='COM4')
    ser.open()
    while True:
        fr = Frame(ser)
        print fr.frame_data_array
