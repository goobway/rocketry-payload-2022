# Name: Calista Greenway
# Language: Python
# Project: Payload - Graphing Live BNO055 Data
# Date: 2/5/2022

import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import math as m
from drawnow import *

SERIALPORT = '/dev/ttyACM0'
BAUDRATE = 115200

# data for plotting
x = []
y = []
z = []

# enable interactive mode to plot live data
plt.ion()
cnt = 0

# create serial object named 'arduinoData'
arduinoData = serial.Serial(SERIALPORT, BAUDRATE)


# plot
def makeGraph():
    plt.title('BNO055 Live Sensor Data')
    plt.axes(projection='3d')
    # plt.xlabel("Pitch: x-axis of the Euler angle vector")
    # plt.ylabel("Yaw: y-axis of the Euler angle vector")
    # "Roll: z-axis of the Euler angle vector"
    plt.grid(True)  # turn grid ON
    line = plt.plot(x, y, z, 'b-o')
    plt.show()


while True:
    while arduinoData.inWaiting() == 0:
        pass
    arduinoString = arduinoData.readline()  # read line from serial port
    dataArray = arduinoString.split(b',')
    xCoordinate = float(dataArray[0])  # X coordinate from data stream
    yCoordinate = float(dataArray[1])  # Y coordinate from data stream
    zCoordinate = float(dataArray[2])  # Z coordinate from data stream
    x.append(xCoordinate)
    y.append(yCoordinate)
    z.append(zCoordinate)
    drawnow(makeGraph)
    plt.pause(.000001)  # pause briefly so drawnow does not crash
    cnt = cnt + 1
    # hold no more than 50 points in each array
    if cnt > 50:
        x.pop(0)
        y.pop(0)
        z.pop(0)
