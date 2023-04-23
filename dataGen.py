import csv
from csv import reader
import random
import serial
import time

with open('launch1.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    field = ["theta", "phi", "xAcc", "yAcc", "zAcc", "latitude", "longitude", "altitude", "altTrend"]
    writer.writerow(field)

    theta = phi = xAcc = yAcc = zAcc = latitude = longitude = alt_index = alt_trend = 0
    variability = 0

    #pre launch
    for i in range(10):
        writer.writerow([theta, phi, xAcc, yAcc, zAcc, latitude, longitude, alt_index, alt_trend])

    zAcc = 1000
    alt_trend = 1
    #until acceleration becomes negative
    for i in range(100):
        #variability = random.uniform(1, 3)
        alt_index += 15 + variability
        zAcc -= 11 - variability
        
        writer.writerow([theta, phi, xAcc, yAcc, zAcc, latitude, longitude, alt_index, alt_trend])
    
    #sustainer fire, acceleration grows super fast
    for i in range(15):
        #variability = random.uniform(1, 3)
        alt_index += 20 + variability
        zAcc += 100 - variability
        
        writer.writerow([theta, phi, xAcc, yAcc, zAcc, latitude, longitude, alt_index, alt_trend])

    #until apogee
    for i in range(50):
        #ariability = random.uniform(1, 3)
        alt_index += 15 + variability
        zAcc -= 11 - variability
        
        writer.writerow([theta, phi, xAcc, yAcc, zAcc, latitude, longitude, alt_index, alt_trend])

    alt_trend = -1
    while(alt_index > 0):
        #variability = random.uniform(1, 3)
        alt_index -= 15 + variability
        zAcc -= 4 - variability

        writer.writerow([theta, phi, xAcc, yAcc, zAcc, latitude, longitude, alt_index, alt_trend])

    theta = phi = xAcc = yAcc = zAcc = latitude = longitude = alt_index = alt_trend = 0
    for i in range(10):
        writer.writerow([theta, phi, xAcc, yAcc, zAcc, latitude, longitude, alt_index, alt_trend])

ser = serial.Serial("COM4")

dataRowIndex = 0

# skip the first line(the header)
with open('launch1.csv', 'r') as file:
    file_csv = reader(file)
    head = next(file_csv)
    
    # check if the file is empty or not
    if head is not None:
        # Iterate over each row
        for row in file_csv:
            # print to serial
            output = ",".join(str(x) for x in row) + "\n"
            ser.write(output.encode('ascii')) 
        
            # wait for acknowledge and/or change in state
            while ser.in_waiting:
                data = ser.readline().decode('ascii')
                #checks first character to see if it's an acknowledge (%). if not,
                #represents change in state. print to console
                if(data[0] != '%'):
                    print(data)

ser.close()