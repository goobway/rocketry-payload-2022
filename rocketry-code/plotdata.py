# Name: Calista Greenway
# Language: Python
# Project: Payload - Graphing Live BNO055 Data
# Date: 2/5/2022

# %%

import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from drawnow import *

SERIALPORT = '/dev/ttyACM0'
# SERIALPORT.encode('UTF-8')
BAUDRATE = 115200

x = []
y = []
# z = []

# enable interactive mode to plot live data
plt.ion()
cnt = 0

# create serial object named 'arduinoData'
arduinoData = serial.Serial(SERIALPORT, BAUDRATE)


# plot]
def makeGraph():
    plt.title('BNO055 Live Sensor Data')
    plt.grid(True)  # turn grid ON
    plt.plot(x, y)
    plt.show()


while True:
    while arduinoData.inWaiting() == 0:
        pass
    arduinoString = arduinoData.readline()  # read line from serial port
    dataArray = arduinoString.split(b',')
    xCoordinate = float(dataArray[0])  # X coordinate from data stream
    yCoordinate = float(dataArray[1])  # Y coordinate from data stream
    # zCoordinate = float(dataArray[2])     # Z coordinate from data stream
    x.append(xCoordinate)
    y.append(yCoordinate)
    # z.append(zCoordinate)
    drawnow(makeGraph)
    plt.pause(.000001)  # pause briefly so drawnow does not crash
    cnt = cnt + 1
    # hold no more than 50 points in each array
    if cnt > 50:
        x.pop(0)
        y.pop(0)
        # z.pop(0)

# %%
