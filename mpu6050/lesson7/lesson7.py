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

    accelx.append(float(dataArray[0])/(32767/2))    
    accely.append(float(dataArray[1])/(32767/2))    
    accelz.append(float(dataArray[2])/(32767/2))
    accelx.pop(0)
    accely.pop(0)
    accelz.pop(0)

    graphX.set_data(t, accelx)
    graphY.set_data(t, accely)
    graphZ.set_data(t, accelz)

    return graphX, graphY, graphZ

   

if __name__ == '__main__':

    portName = "/dev/ttyACM0"
    ser = serialConnect(portName,115200)
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

    graphX, = ax.plot([], [], 'b', label = 'X')
    graphY, = ax.plot([], [], 'r', label = 'Y')
    graphZ, = ax.plot([], [], 'g', label = 'Z')
    ax.legend(loc='upper right')
    ax.legend(loc='upper right')
    ax.legend(loc='upper right')

    t = list(range(0, numPoints))
    accelx = []
    accely = []
    accelz = []

    for i in range(0, numPoints):
        accelx.append(0)
        accely.append(0)
        accelz.append(0)


    delay = 20
    anim = animation.FuncAnimation(fig, animate, init_func=init,
                               interval=delay, blit=True)

    plt.show() 

