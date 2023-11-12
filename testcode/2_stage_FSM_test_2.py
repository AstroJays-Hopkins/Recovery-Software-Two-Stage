import serial
import pandas as pd
import time

print("Initalizing Connection...")
ser = serial.Serial("COM4", 115200)

print("Loading Data..")
# setup dataframe
df = pd.read_csv("Strugglebus_June2022_FAR.csv")

print("Dataset Ready")


def prep_next(df, time_step):
    full_data = df.iloc[time_step]
    return int(full_data["state"]), int(full_data["acceleration"]), int(full_data["height"])

timestep = 0
for timestep in range(0,100):
    state, acceleration, height = prep_next(df,timestep)
    print("sent: " + str(height))
    ser.write(str(height).encode())
    response = ser.readline()
    print("got: " + str(response.decode()))
    


ser.close()