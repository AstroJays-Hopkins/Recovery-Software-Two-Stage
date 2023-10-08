import serial
import time
import pandas as pd
import struct


print("Setting Up...")
# settings
DELAY = 0
time_step = 0


# corresponding class to the IMU struct in the statemachines
class IMU:
    def __init__(self, xGyro, yGyro, zGyro, xAcc, yAcc, zAcc):
        self.xGyro = xGyro
        self.yGyro = yGyro
        self.zGyro = zGyro
        self.xAcc = xAcc
        self.yAcc = yAcc
        self.zAcc = zAcc


# corresponding class to the Packet struct in the statemachines
class Packet:
    def __init__(self, header, stage, latitude, longitude, altitude, alt_trend, imuPacket):
        self.header = header
        self.padding0 = 0
        self.stage = stage
        self.padding1 = 0
        self.latitude = latitude
        self.padding2 = 0
        self.longitude = longitude
        self.padding3 = 0
        self.altitude = altitude
        self.padding4 = 0
        self.alt_trend = alt_trend
        self.padding5 = 0
        self.imuPacket = imuPacket


# creates a struct of our data in the proper format for the arduino from our packet object
def pack_data(packet):
    # Define the format string for struct.pack
    format_string = "BxBxlxlxlxlxllllll"

    # Pack the data using the format string
    packed_data = struct.pack(
        format_string,
        packet.header,
        packet.stage,
        packet.latitude,
        packet.longitude,
        packet.altitude,
        packet.alt_trend,
        packet.imuPacket.xGyro,
        packet.imuPacket.yGyro,
        packet.imuPacket.zGyro,
        packet.imuPacket.xAcc,
        packet.imuPacket.yAcc,
        packet.imuPacket.zAcc,
    )

    return packed_data


# user facing function to get the next datapoint to send to teh rocket
# FOR TESTING: only sends z.acc state and altitude (do not have gyro, long, lat, x.acc or y.acc)
# NOTEL actually using height(from ground) NOT altitude (from sea level)
def prep_next(df, time_step):
    full_data = df.iloc[time_step]
    time_step += 1
    imu_data = IMU(0, 0, 0, 0, 0, int(full_data["acceleration"]))
    packet_data_class = Packet(0x55, full_data["state"], 0, 0, int(full_data["height"]), 0, imu_data)
    struct_data = pack_data(packet_data_class)
    print("Packed Data:", struct_data)
    return struct_data, time_step


print("Initalizing Connection...")
# setup serial to arduino
# TODO: fix port
port = serial.Serial("COM3")

print("Loading Data..")
# setup dataframe
df = pd.read_csv("Strugglebus_June2022_FAR.csv")

print("Dataset Ready")


# Example usage:
# imu_data = IMU(100, 200, 300, 400, 500, 600)
# packet_data = Packet(0x55, 1, 123456, 789012, 345, imu_data)

# # Pack the data and print the result
# packed_data = pack_data(packet_data)
# print("Packed Data:", packed_data)


print("Begining send every " + str(DELAY) + " seconds")

while df.shape[0] > time_step:
    data, time_step = prep_next(df, time_step)
    port.write(data)
    res = port.readline()
    print(res)
    time.sleep(DELAY)
    print(time_step)
