import csv
from csv import reader
import random
import time

with open('launch1.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    field = ["state","theta", "phi", "acceleration", "height"]
    writer.writerow(field)

    theta = phi = acc = alt = 0
    variance = 1

    #pre launch
    state = 0
    for i in range(10):
        writer.writerow([state, round(theta, 2), round(phi, 2),round(acc, 2) , round(alt, 2) ])

    acc = 600
    state = 1
    #booster ignition
    while acc > -10:
        #variability = random.uniform(1, 3)
        alt += acc/10 + random.gauss(0, variance)
        acc = acc*.995 - 2*random.gauss(1, variance)
        theta = random.gauss(0, variance)
        phi = random.gauss(0, variance)
        
        writer.writerow([state, round(theta, 2), round(phi, 2),round(acc, 2) , round(alt, 2) ])
    
    acc = 600
    state = 2
    #sustainer ignition
    while acc > -10:
        #variability = random.uniform(1, 3)
        alt += acc/10 + random.gauss(0, variance)
        acc = acc*.995 - 2*random.gauss(1, variance)
        theta = random.gauss(0, variance)
        phi = random.gauss(0, variance)
        
        writer.writerow([state, round(theta, 2), round(phi, 2),round(acc, 2) , round(alt, 2)])

    #descent
    state = 3
    while(alt > 0):
        alt -= 3 + random.gauss(0, variance)
        acc -= 0 - random.gauss(0, variance)
        theta = random.gauss(0, 2*variance)
        phi = random.gauss(0, 2*variance)

        writer.writerow([state, round(theta, 2), round(phi, 2),round(acc, 2) , round(alt, 2) ])

    #on ground
    state = 4
    theta = phi = acc = alt= 0
    for i in range(10):
        writer.writerow([state, round(theta, 2), round(phi, 2),round(acc, 2) , round(alt, 2) ])

