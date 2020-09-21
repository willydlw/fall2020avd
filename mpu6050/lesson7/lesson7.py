# Plot 3 signals

import serial
import numpy as np
from time import sleep 
from matplotlib import pyplot as plt
from matplotlib import animation


def serialConnect(portName, baudRate):
    try: 
        ser = serial.Serial(portName, baudRate)
        print("opened port " + ser.name + '\n')
        # give Arduino time to reset
        sleep(2)
        # flush input buffer, discarding all contents
        ser.reset_input_buffer()
        return ser 
    except serial.SerialException:
        raise IOError("problem connecting to " + portName)


def init():
    # draw a clear frame with empty Line2d objects
    graphX.set_data([], [])
    graphY.set_data([], [])
    graphZ.set_data([], [])
    return graphX, graphY, graphZ

def animate(i):
    global t, accelx, accely, accelz

    while (ser.inWaiting() == 0):
        pass

    arduinoString = ser.readline().decode("utf-8")
    dataArray = arduinoString.split(',')

    # add scaled data points to end of array
    accelx.append(float(dataArray[0])/(32767/2))    
    accely.append(float(dataArray[1])/(32767/2))    
    accelz.append(float(dataArray[2])/(32767/2))
    
    # remove oldest data point from front of array
    accelx.pop(0)
    accely.pop(0)
    accelz.pop(0)

    # add data to each graph set for plotting
    graphX.set_data(t, accelx)
    graphY.set_data(t, accely)
    graphZ.set_data(t, accelz)

    # return list of iterable artists
    return graphX, graphY, graphZ

   

if __name__ == '__main__':

    portName = "/dev/ttyACM0"
    ser = serialConnect(portName,38400)
    sleep(2)                                        # give Arduino time to reset

    # flush input buffer, discarding all contents
    ser.reset_input_buffer()

    numPoints = 201                                 # number of data points
    fig = plt.figure(figsize=(12, 6))               # create figure window
    ax = plt.axes(xlim=(0,numPoints-1), ylim=(-2, 2))    # specify axis limits

    plt.title('Real-time sensor data')
    plt.xlabel('Data points')
    plt.ylabel('Acceleration [G]')
    ax.grid(True)

    # specify line color, labeling
    graphX, = ax.plot([], [], 'b', label = 'X')
    graphY, = ax.plot([], [], 'r', label = 'Y')
    graphZ, = ax.plot([], [], 'g', label = 'Z')

    # specify legend location
    ax.legend(loc='upper right')
    ax.legend(loc='upper right')
    ax.legend(loc='upper right')

    # data list initialization
    t = list(range(0, numPoints))     # create list of integers in range [0, numPoints]
    accelx = []                       # declare 3 empty lists
    accely = []
    accelz = []

    # fill lists with zero values
    for i in range(0, numPoints):
        accelx.append(0)
        accely.append(0)
        accelz.append(0)


    delay = 20
    anim = animation.FuncAnimation(fig, animate, init_func=init,
                               interval=delay, blit=True)

    plt.show() 

