from difflib import SequenceMatcher
from time import sleep

file1 = "test_data_received.txt"
file2 = "test_data_sent.txt"

while True:
    text1 = open(file1).read()
    text2 = open(file2).read()
    m = SequenceMatcher(None, text1, text2)
    print m.ratio()
    sleep(10)
