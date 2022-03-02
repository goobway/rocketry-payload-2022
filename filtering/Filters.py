

import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt


def readAllData(accName, gyroName, magName):
    accData = readData(accName)
    gyroData = readData(gyroName)
    magData = readData(magName)

    return accData, gyroData, magData


def readData(fname):
    f = open(fname, "r")
    contents = f.readlines()
    del contents[0]
    L = len(contents)
    # initialize lists of values
    x_vals = np.zeros(L, dtype=float)
    y_vals = np.zeros(L, dtype=float)
    z_vals = np.zeros(L, dtype=float)

    for i in range(len(contents)):
        strippedVal = contents[i].strip("\n") # remove line break between time instances
        xyzSplitter = strippedVal.split("\t") ### switch back to ,
        xVal = float(xyzSplitter[0])
        yVal = float(xyzSplitter[1])
        zVal = float(xyzSplitter[2])
        x_vals[i] = xVal
        y_vals[i] = yVal
        z_vals[i] = zVal
    ################
    startTime = 875  # default 0
    endTime = 1000  # default L
    ################
    data = {"X": x_vals[startTime:endTime], "Y": y_vals[startTime:endTime], "Z": z_vals[startTime:endTime]}
    return data

def moving_average(input, n=10):
    array_meanA = np.zeros(n)
    for x in range(n):
        #getSensorData()  # Get some data.eg tempPiFloat
        #array_meanA[n] = tempPiFloat  # tempPiFloat slots into end of array.
        for h in range(n):
            array_meanA[h] = array_meanA[(h + 1)]  # Shift the values in the array to the left
        meanA = 0
        for h in range(n):
            meanA = array_meanA[h] + meanA  # Calculate the mean, no weights.
        meanA = meanA / n

def energySections(sig):
    energyArray = []
    print(sig)
    print(len(sig))
    for i in range(sig.size):
        energyArray.append(sig[i])
    print(len(energyArray))
    plt.plot(range(len(energyArray)), energyArray)
    plt.show()

def rect_LPF(input, c, w):
    tMax = input.size
    W = tMax * w
    C = tMax * c

    freqSignal = np.fft.fft(input)

    # create window in freq domain
    freqRect = np.zeros(tMax, dtype=float)
    for i in range(freqSignal.size):
        if (i < (C-w)) or (i > ((tMax-C)+w)):
            freqRect[i] = 1
        elif ((C + w) < i) and (i < ((tMax-C)-w)):
            freqRect[i] = 1
    convolvedSignal = np.multiply(freqSignal, freqRect)
    reconstructedSignal = np.fft.ifft(convolvedSignal)

    output = np.array([input, freqSignal, freqRect, reconstructedSignal])
    return output

def butter_LPF(input):
    #import numpy as np
    #from scipy.signal import butter, filtfilt
    # Filter requirements.
    T = 5.0  # Sample Period
    fs = 30.0  # sample rate, Hz
    cutoff = 2  # desired cutoff frequency of the filter, Hz ,      slightly higher than actual 1.2 Hz
    nyq = 0.5 * fs  # Nyquist Frequency
    order = 2  # sin wave can be approx represented as quadratic
    n = int(T * fs)  # total number of samples

    normal_cutoff = cutoff / nyq
    # Get the filter coefficients
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, input)
    out = np.array([np.zeros((1, len(y))), np.zeros((1, len(y))), np.zeros((1, len(y))), y])
    return out

def filters(accName, gyroName, magName):
    accData, gyroData, magData = readAllData(accName, gyroName, magName)

    #energySections(accData["X"])

    accX = rect_LPF(accData["X"], 0.25, 15)  # X-direction linear acceleration
    accY = rect_LPF(accData["Y"], 0.3, 10)  # Y-direction linear acceleration
    accZ = rect_LPF(accData["Z"], 0.3, 10)  # Z-direction linear acceleration
    graphThreeAxis(accX, accY, accZ, "Linear Acceleration")

    gyroX = rect_LPF(gyroData["X"], 0.4, 20)  # X-direction angular velocity
    gyroY = rect_LPF(gyroData["Y"], 0.3, 10)  # Y-direction angular velocity
    gyroZ = rect_LPF(gyroData["Z"], 0.3, 10)  # Z-direction angular velocity
    graphThreeAxis(gyroX, gyroY, gyroZ, "Gyroscope")

    magX = rect_LPF(magData["X"], 0.3, 10)  # X-direction magnetic bearing
    magY = rect_LPF(magData["Y"], 0.3, 10)  # Y-direction magnetic bearing
    magZ = rect_LPF(magData["Z"], 0.3, 10)  # Z-direction magnetic bearing
    graphThreeAxis(magX, magY, magZ, "Euler coordinates")


def graphThreeAxis(listX, listY, listZ, title):

    fig, ax = plt.subplots(2, 3)

    bigTitle = plt.suptitle(title)
    bigTitle.set_size(20)
    bigTitle.set_weight("bold")

    inputs = [listX, listY, listZ]
    t = np.arange(0, listX[0].size)
    f = np.arange(0, listX[1].size)
    colCounter = 0
    for val in inputs:

        # frequency domain (original signal)
        axRect = ax[0, colCounter].twinx()
        ax[0, colCounter].plot(f, abs(val[1]), color="tab:orange")
        axRect.plot(f, abs(val[2]), color="tab:purple", linewidth=3)
        plt.ylim([-0.1, 1.1])

        # time domain (filtered signal)
        ax[1, colCounter].plot(t, val[0], color='tab:blue')
        ax[1, colCounter].plot(t, val[3], color='k')

        colCounter = colCounter + 1

    plt.show()


def main():
    filters("FullScaleLinAcc.txt", "FullScaleGyro.txt", "FullScaleEuler.txt")


if __name__ == "__main__":
    main()

