# Name: UMass Rocket Team
# Language: Python
# Project: Resolve Grid Coordinate of Launch Vehicle

import matplotlib.pyplot as plt
import csv
import scipy.integrate as it
import numpy as np
import math

# READ DATA
file = open('DATAIMU1.csv')
csv_reader = csv.reader(file)
next(csv_reader)

timestamp = []
accX = []
accY = []
accZ = []

empty = ''  # find invalid rows
for i, row in enumerate(csv_reader):
    if empty not in row:
        timestamp.append(float(row[0])/1000)  # seconds
        accX.append(float(row[4])*3.281)  # feet
        accY.append(float(row[5])*3.281)  # feet
        accZ.append(float(row[6])*3.281)  # feet
    if i >= 1500:
        break

# take samples from desired range
timestamp = timestamp[875:1050]
accX = accX[875:1050]
accY = accY[875:1050]
accZ = accZ[875:1050]

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
plt.legend(["ALTITUDE", "Y", "X", "ZERO"])


# PLOT DATA
x = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T"]
y = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]

# find starting point
startX = ord(input("ENTER STARTING POINT X: ")) - 96
startY = int(input("ENTER STARTING POINT Y: "))
start = chr(startX + 96)+str(startY)
start = start.upper()

# find landing point
endX = startX - locationZ[-1] / 250
endY = startY - locationY[-1] / 250
end = chr(round(endX) + 96)+str(round(endY))
end = end.upper()

# actual landing point
actualX = startX + 2.6
actualY = startY - 2.6
actual = chr(round(actualX) + 96)+str(round(actualY))
actual = actual.upper()

# distance between start and end
d = math.sqrt(((startX - endX)**2) + ((startY - endY)**2))

# draw circle
circle = plt.Circle((startX, startY), d, color='blue', fill=False)

fig = plt.figure()
ax = fig.add_axes([0.1, 0.1, 0.8, 0.8])  # main axes
ax.set_aspect('equal', adjustable='box')

# plot functions
plt.plot(startX, startY, color='red', marker="o", markersize=12, markeredgecolor="red", markerfacecolor="red")
plt.plot(endX, endY, color='blue', marker="o", markersize=12, markeredgecolor="blue", markerfacecolor="blue")
plt.plot(actualX, actualY, color='purple', marker="o", markersize=12, markeredgecolor="purple", markerfacecolor="purple")
ax.add_artist(circle)
plt.legend(["START: " + start, "END: " + end, "ACTUAL: " + actual])

# image background
img = plt.imread("map.png")
ax.imshow(img, extent=[0, 20, 0, 20])

ax.set_title('VERMONT LAUNCH SITE')
ax.set_xticks([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20])
ax.set_xticklabels(["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T"])
ax.set_yticks([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20])

plt.grid(color='#888888')  # color the grid

plt.show()
