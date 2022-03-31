import matplotlib.pyplot as plt
import csv
import numpy as np

# READ DATA
file = open('displacement.csv')
csv_reader = csv.reader(file)
next(csv_reader)

timestamp = []
dispX = []
dispY = []
dispZ = []

empty = ''  # find invalid rows
for row in csv_reader:
    timestamp.append(float(row[0]))  # seconds
    dispX.append(float(row[1]) * 3.281)  # feet
    dispY.append(float(row[2]) * 3.281)  # feet
    dispZ.append(float(row[3]) * 3.281)  # feet

# plot location data by time
plt.plot(timestamp, dispX)
plt.plot(timestamp, dispY)
plt.plot(timestamp, dispZ)
plt.ylabel("DISTANCE TRAVELED, FEET")
plt.xlabel("TIME, SECONDS")
plt.plot(timestamp, np.zeros(len(timestamp)), "--", color='black')
plt.legend(["ALTITUDE", "Y", "X", "ZERO"])

plt.show()
