import serial
import pandas as pd
import time

print("Initalizing Connection...")
ser = serial.Serial("COM7", 115200)

for timestep in range(0,100000):
    response = ser.readline().decode().strip()  # Decode bytes to string and remove trailing newline
    if response.isalpha():
        print("got:", response)
    else:
        print("data:", response)

ser.close()