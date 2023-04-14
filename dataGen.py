import csv
import random

alt_index = 0
zAcc = 1000

with open('launch1.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    field = ["zAcc", "altTrend", "altitude"]
    writer.writerow(field)

    #pre launch
    for i in range(10):
        writer.writerow(["0.0", "0", "0.0"])

    #until acceleration becomes negative
    for i in range(100):
        alt_index += 15
        zAcc -= 11
        variability = random.uniform(1, 3)
        writer.writerow([zAcc - variability, 1, alt_index + variability])

    #sustainer fire, acceleration grows super fast
    for i in range(15):
        alt_index += 20
        zAcc += 100
        variability = random.uniform(1, 3)
        writer.writerow([zAcc - variability, 1, alt_index + variability])

    #until apogee
    for i in range(50):
        alt_index += 15
        zAcc -= 11
        variability = random.uniform(1, 3)
        writer.writerow([zAcc - variability, 1, alt_index + variability])

    while(alt_index > 0):
        alt_index -= 15
        variability = random.uniform(1, 3)
        zAcc -= 4
        writer.writerow([zAcc - variability, -1, alt_index + variability])

    for i in range(10):
        writer.writerow(["0.0", "0", "0.0"])