import serial
from signal import signal, SIGINT
from time import sleep

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
        sleep(2)
        # flush input buffer, discarding all contents
        ser.reset_input_buffer()
        return ser 
    except serial.SerialException:
        raise IOError("problem connecting to " + portName)


def waitForReadySignal(ser):
    readyMsg = 'READY'
    global keepRunning 
    readyReceived = False  

    while readyReceived  == False and keepRunning == True:
        while ser.in_waiting < len(readyMsg):
            pass
        
        '''
        read_until(expected=LF,size=None)

        read until an expected sequence is found('\n' by default),
        the size is exceeded or until timeout occurs. With no timeout
        it will block until the requested number of bytes is read
        '''
        print("ready to read")
        bytesRead = ser.read_until()
        print("bytesRead: ", end='')
        print(bytesRead)

        # convert byte string to unicode string, remove leading and trailing whitespace
        receivedMsg = bytesRead.decode().strip()   # decode default is 'utf-8'
        print(receivedMsg)
        if receivedMsg == readyMsg:
            readyReceived = True
            print("received ready message: " + receivedMsg)
        else:
            print("expected ready message, received: " + str(bytesRead))

        return ser


def sendAck(ser):
    ackMessage = 'ACK\n'
    ser.write(ackMessage.encode())
    return ser



if __name__ == '__main__':

    #register the signal handler
    signal(SIGINT, handler)

    portName = "/dev/ttyACM0"
    ser = serialConnect(portName)
    
    ser = waitForReadySignal(ser)
    ser.reset_input_buffer()
    print("ready signal received")
    ser = sendAck(ser)
    print("sent ack")

    while keepRunning == True:
        print("main keepRunning loop")
        # kill a bit of time before running through loop again
        # Arduino program transmitting every 2 sec
        sleep(2)

    print('while loop terminated')
    ser.close()
    print("closed port")