import matplotlib.pyplot as plt
import csv
import numpy as np
import scipy.integrate as it

# READ DATA
file = open('acc-data-20ft-cal2')
csv_reader = csv.reader(file)
next(csv_reader)

timestamp = []
accX = []
accY = []
accZ = []

empty = ''  # find invalid rows
for row in csv_reader:
    timestamp.append(float(row[0]) / 1000.0)  # seconds
    accX.append(float(row[1]) * 3.281)  # feet
    accY.append(float(row[2]) * 3.281)  # feet
    accZ.append(float(row[3]) * 3.281)  # feet

# take samples from desired range
# timestamp = timestamp[864:1000]
# accX = accX[864:1000]
# accY = accY[864:1000]
# accZ = accZ[864:1000]

# CALCULATE X
velocityX = it.cumtrapz(accX, timestamp, initial=0)
locationX = it.cumtrapz(velocityX, timestamp, initial=0)

# CALCULATE Y
velocityY = it.cumtrapz(accY, timestamp, initial=0)
locationY = it.cumtrapz(velocityY, timestamp, initial=0)

# CALCULATE Z
velocityZ = it.cumtrapz(accZ, timestamp, initial=0)
locationZ = it.cumtrapz(velocityZ, timestamp, initial=0)

# plot location data by time
plt.plot(timestamp, locationX)
plt.plot(timestamp, locationY)
plt.plot(timestamp, locationZ)
plt.ylabel("DISTANCE TRAVELED, FEET")
plt.xlabel("TIME, SECONDS")
plt.plot(timestamp, np.zeros(len(timestamp)), "--", color='black')
plt.legend(["X", "Y", "Z", "ZERO"])
plt.show()

# bird's eye
plt.plot(locationX, locationY)
plt.ylabel("Y")
plt.xlabel("X")
plt.title("BIRDS EYE VIEW")
plt.show()
