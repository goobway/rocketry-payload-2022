# Name: Calista Greenway
# Language: Python
# Project: Payload Python Attempt

import serial
from drawnow import *

SERIALPORT = '/dev/ttyACM0'
BAUDRATE = 115200

# enable interactive mode to plot live data
plt.ion()
cnt = 0

# create serial object named 'arduinoData'
arduinoData = serial.Serial(SERIALPORT, BAUDRATE)

# data from BNO055
timestamp = []
posX = []
posY = []
posZ = []


# plot
def makeGraph():
    # labels
    plt.suptitle('BNO055 LIVE SENSOR DATA')
    plt.xlabel("TIME, SECONDS")
    plt.ylabel("DISTANCE TRAVELED, FEET")
    # plot x, y, and z
    plt.plot(timestamp, posX)
    plt.plot(timestamp, posY)
    plt.plot(timestamp, posZ)

    plt.ylim([-20, 20])
    # plt.axes(projection='3d')
    plt.grid(True)  # turn grid ON
    plt.show()


while True:
    while arduinoData.inWaiting() == 0:
        pass
    arduinoString = arduinoData.readline()  # read line from serial port
    dataArray = arduinoString.split(b',')
    if len(dataArray) == 4:  # skip invalid rows
        # parse incoming data array
        timeData = float(dataArray[0])
        xData = float(dataArray[1])
        yData = float(dataArray[2])
        zData = float(dataArray[3])
        # append data to corresponding array
        timestamp.append(timeData)
        posX.append(xData)
        posY.append(yData)
        posZ.append(zData)
        # draw live plot
        drawnow(makeGraph)
        plt.pause(.000001)  # pause briefly so drawnow does not crash
        cnt = cnt + 1
        # hold no more than 1000 points in each array
        if cnt > 1000:
            posX.pop(0)
            posY.pop(0)
            posZ.pop(0)
