# Name: Calista Greenway
# Language: Python
# Project: Payload - Graphing Live BNO055 Data
# Date: 2/7/2022

import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import math as m
from drawnow import *

SERIALPORT = '/dev/ttyACM0'
BAUDRATE = 115200

# data for plotting
time = []
pitch = []  # x
# yaw = []    # y
roll = []   # z

# enable interactive mode to plot live data
plt.ion()
cnt = 0

# create serial object named 'arduinoData'
arduinoData = serial.Serial(SERIALPORT, BAUDRATE)


# plot
def makeGraph():
    plt.suptitle('BNO055 Live Sensor Data')
    plt.subplot(211)
    line1 = plt.plot(time, pitch)
    plt.grid(True)  # turn grid ON
    plt.ylim([-60, 60])
    plt.subplot(212)
    line2 = plt.plot(time, roll)
    plt.ylim([-60, 60])
    # plt.axes(projection='3d')
    # plt.xlabel("Pitch: x-axis of the Euler angle vector")
    # plt.ylabel("Yaw: y-axis of the Euler angle vector")
    # "Roll: z-axis of the Euler angle vector"
    plt.grid(True)  # turn grid ON
    plt.show()


while True:
    while arduinoData.inWaiting() == 0:
        pass
    arduinoString = arduinoData.readline()  # read line from serial port
    dataArray = arduinoString.split(b',')
    timeData = float(dataArray[0])
    pitchData = float(dataArray[1])
    # yawData = float(dataArray[1])
    rollData = float(dataArray[2])
    time.append(timeData)
    pitch.append(pitchData)
    # yaw.append(yawData)
    roll.append(rollData)
    drawnow(makeGraph)
    plt.pause(.000001)  # pause briefly so drawnow does not crash
    cnt = cnt + 1
    # hold no more than 100 points in each array
    if cnt > 100:
        pitch.pop(0)
        roll.pop(0)
