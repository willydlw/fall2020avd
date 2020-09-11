import serial
from signal import signal, SIGINT
import time
import struct

keepRunning = True 

def handler(signal, frame):
    global keepRunning 
    print('SIGINT or CTRL+C detected, setting keepRunning to False')
    keepRunning = False


def serialConnect(portName):
    try: 
        ser = serial.Serial(portName)
        print("opened port " + ser.name + '\n')
        # give Arduino time to reset
        time.sleep(2)
        # flush input buffer, discarding all contents
        ser.reset_input_buffer()
        return ser 
    except serial.SerialException:
        raise IOError("problem connecting to " + portName)


def getSignalFrequency():
    freq = -1.0
    while(freq < 1.0 or freq > 250.0):
        # input returns a string, convert it to a float
        freq = float(input('enter signal frequency in range [1.0,250.0] '))
    return freq
    

def sendSignalFrequency(ser, freq):
    # convert float to 4 byte, bytes object
    freqBytes = struct.pack('f',freq)

    # not necessary to convert to bytearray type
    # but can do so with the code below
    #freqBytes = bytearray(struct.pack('f',freq))

    print("Debug: sendSignalFrequency(ser,freq) ")
    print("freqBytes: ", end='')
    print([ "0x%02x" % b for b in freqBytes])
    if len(freqBytes) == 4:
        ser.write(freqBytes)
        return True 
    else:
        print("error: len(freqBytes) " + str(len(freqBytes)))
        print("Arduino expects 4 bytes")
        return False 
    


if __name__ == '__main__':

    #register the signal handler
    signal(SIGINT, handler)

    portName = "/dev/ttyACM1"
    ser = serialConnect(portName)

    signalFreq = getSignalFrequency()

    if sendSignalFrequency(ser, signalFreq) == True:
        print("transmitted signal frequency")
    else:
        print("failed to transmit signal frequency")
        ser.close()
        exit(1)

    print("Waiting for ack")

    start = time.monotonic()
    while(time.monotonic() - start) < 1./10.:
        pass

    print(time.monotonic() - start)

    '''
    count = 0
    while count < 4:
        if ser.in_waiting > 0:
            print("byte waiting")
            val = ser.read(1)
            count += 1
            print("val: ", end='')
            print(val)
            print("count: ", end='')
            print (count) 
    '''


    
    '''
    ser = waitForReadySignal(ser)
    ser.reset_input_buffer()
    print("ready signal received")
    ser = sendAck(ser)
    print("sent ack")
    ''' 

    while keepRunning == True:
        if ser.in_waiting > 0:
            val = ser.read(1).decode('utf-8')
            print(val)
        else:
            print("main keepRunning loop")
            sendSignalFrequency(ser,signalFreq)
        # kill a bit of time before running through loop again
        # Arduino program transmitting every 2 sec
        time.sleep(100/1000)

    print('while loop terminated')
    ser.close()
    print("closed port")