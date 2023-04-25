import serial
import Essentials
import time
ser = serial.Serial(Essentials.Essentials.get_comport(),115200)

while True:


    print(ser.readline())

