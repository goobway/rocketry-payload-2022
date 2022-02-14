import numpy as np
import matplotlib.pyplot as plt
import serial
from drawnow import *

SERIALPORT = '/dev/ttyACM0'
BAUDRATE = 115200

# enable interactive mode to plot live data
plt.ion()
cnt = 0

# create serial object named 'arduinoData'
arduinoData = serial.Serial(SERIALPORT, BAUDRATE)

bar_colors = ['#333333', '#444444', '#555555', '#666666', '#777777', '#888888', '#999999', '#AA0000']
num_obs = len(bar_colors)

# data from BNO055
yaw = []


# determine cardinal coordinate
def degrees_to_cardinal(d):
    direction = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                 "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
    ix = int((d + 11.25) / 22.5)
    print(ix)
    return direction[ix % len(direction)]


def makeGraph():
    # polar plotting
    fig = plt.figure(figsize=(3, 3))  # size
    ax = plt.subplot(111, polar=True)  # create subplot
    plt.grid(color='#888888')  # color the grid
    ax.set_theta_zero_location('N')  # set zero to North

    ax.vlines(yaw, 0, 0, colors=bar_colors, zorder=3)

    fig.show()


while True:
    while arduinoData.inWaiting() == 0:
        pass
    arduinoString = arduinoData.readline()  # read line from serial port
    dataArray = arduinoString.split(b' ')
    yawData = float(dataArray[0])
    yaw.append(yawData)
    # drawnow(makeGraph)
    plt.pause(.000001)  # pause briefly so drawnow does not crash
    cnt = cnt + 1
    print(yawData)

    # print cardinal coordinate
    print(degrees_to_cardinal(yaw[0]))

    if cnt > 0:
        yaw.pop(0)
