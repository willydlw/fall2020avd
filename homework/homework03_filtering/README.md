# Homework 3 - Digital Filtering, Serial Comm, State

This module contains an example approach to help you begin solving the [homework](./homework03_serial_plotting_filtering.docx) problem. The solution approach breaks the problem statement into separate parts, solves those parts individually, and then then them into a workable solution.

Note that we strive to solve the sub-problems, not optimally with the first attempt, simply implementing a working solution. After a workable, well-tested solution is created, optimization, if needed, is a later step.<br><br>

## General Problem Description

The Arduino will generate a noisy periodic signal, filter it with a first-order low pass digital filter and serially transmit both the noisy signal and filtered signal values to a python program. The python program will write the signal data to files and plot the signal data.<br><br>

## Homework Concepts
- Serial communication, Arduino to python, python to Arduino
- Simulate a noisy sensor
    - Signal Generation
    - Sampling frequency, sampling interval
    - Nyquist sampling criteria
- Digital low pass filter
- Finite state machine behavior
<br><br>

## Finite State Behavior

States for both the Arduino and python programs are specified in the assignment. Successive read-through of the control flow resulted in the following:

| Arduino States | python states |
| --- | --- |
| 1. Initialize serial | 1. Initialize serial |
| 2. Send ready signal until acknowledgment received | 2. Wait for ready signal |
| 3. Ack received | 3. Acknowledge ready received |
| 4. Wait for frequency | 4. Send frequency |
| 5. Send ack | 5. Wait for ack |
| 6. Transmit Signal Data | 6. Receive Signal Data |
| 7. Transmit end of signal | 7. Receive end of signal |
| 8. Return to state 4 | 8. Process signal data |
|     | 9. Return to state 4 |

Each of these states represents a sub-problem to be solved. We start with initializing the serial hardware connections.
<br><br>

## Initialize Serial

Initializing the serial port requires configuration decisions regarding 
- baud rate
- start bit, stop bits, data bits, parity bit
- read and write timeouts

Let's start with the easy one: 1 start bit, 1 stop bit, 8 data bits, no parity bit. This is the Arduino and python default serial configuration which meets our requirements.

Baud rate depends on the speed needed to transmit the signal data from the Arduino to python. 
- What is the highest possible sampling frequency? 
- How much data is transmitted in that interval? 

For read and write timeouts, we need to know if blocking behavior will meet our requirements. We have to examine the states in which there are Serial read and write operations and determine if a blocking read and blocking write will work with the program control flow requirements. A blocking read/write means that operation does not return until the read/write operation finishes. 

The problem specification indicates that both the Arduino and python states have a sequential flow, transitioning from one state to another upon completing tasks and receiving acknowledgments. There are no behaviors requiring the Arduino to stop in the middle of a serial read/write to execute another task.

The only interrupt to be handled is a SIGINT on the python side. That interrupt is designed to cause the python program to shut down gracefully, saving data, and releasing system resources by closing connections. Waiting for a blocking serial read or write might result in the interrupt not working as quickly as expected. We have not yet studied how to handle serial functions with timeouts in python, so we will allow read and write to block.

The Arduino defaults for read and write timeouts will meet our needs as well.
<br><br><br>

## Ready-Acknowledge Implementation

After the serial initialization state, the next Arduino/python states are the ready-acknowledge sequence. Here, we need to decide on the ready signal and acknowledge signal. The character string "READY" and character string "ACK" will serve as these signals.
<br>

**Arduino Ready-Acknowledge Behavior**

- Arduino serially transmits a ready signal to the python program, once per second. 
- Continue transmitting the ready signal until an acknowledgement from the python program is received.
<br><br>

**python Ready-Acknowledge Behavior**

- Wait to receive ready signal from Arduino
- transmit acknowledgement to Arduino in response to receiving ready signal.
<br><br>

**Handling Character Strings**

Character strings require multiple byte transmission and reception. Terminating these strings with a '\n' character is a convenient way to indicate the end of message condition. Arduino and python both have serial functions designed to handle these messages.

Arduino's Serial.println transmits ASCII character data and appends the '\n' character to the end. python's read_until function can be used to read all bytes until the '\n' character is detected. That will handle Arduino's ready transmission and python's reception of that message.

Arduino's Serial.readBytesUntil function will be used to read "ACK" until the '\n' character is found. Python's serial write function will transmit the "ACK" message.

See the [lesson7.ino](../../serial_comm/lesson7/lesson7.ino) example and [README.md](../../serial_comm/lesson7/README.md )to experiment with readBytesUntil.

The following videos explain how the code is developed and tested:
- https://youtu.be/b52ksbX6J94
- https://youtu.be/pcqIim0fAj4 
<br><br><br>

## Frequency - Acknowledge Implementation

After completing the ready-acknowlege, the next Arduino/python states are the frequency-acknowledge sequence. Here, we need to decide how python will transmit the frequency. Before integrating this into the final programming solution, write separate programs to implement and test this sub-problem.
<br>

**Arduino Frequency-Acknowledge Behavior**

- Arduino waits to receive the signal frequency from the python program
- After receiving the signal frequency, send acknowledgment
<br><br>

**python Ready-Acknowledge Behavior**

Python program transmits a signal frequency in units of Hz to Arduino 10 times per second until Arduino acknowledges receipt of signal frequency.

- Transmit signal frequency in units of Hz to Arduino 
- Wait for acknowledgment
- Retransmit if ACK not recieved within 1/10 second.
<br><br>


**Python Send Frequency Test Program**

Frequency Input - Frequency is a user input, with a maximum of 250 Hz. The assignment does not specify if this is an integer or floating point data type. Since we have worked with integer data in previous serial communication examples, let's work with the floating point data type.

The input function reads information from the keyboard and returns it as a string. Simply passing that string to the float function will convert it to floating point. (Note: Assumes user inputs numeric data. Implement the simplest basic, working case first. Add other forms of validation later, if there is time.)

Another consideration is the minimum frequency. Do we want to allow DC, 0 Hz? The assignment requires generating a periodic signal. Zero Hz, DC, will be excluded. For the example code below, the chosen minimum signal frequency is 1 Hz.


```
def getSignalFrequency():
    freq = -1.0
    while(freq < 1.0 or freq > 250.0):
        # input returns a string, convert it to a float
        freq = float(input('enter signal frequency in range [1.0,250.0] '))
    return freq
```
<br>

**Transmitting Signal Frequency**

The floating point frequency value must be converted to type bytes or bytearray for the python serial write function. The struct library is used to interpret bytes as packed binary data. The struct.pack function is used to convert the float variable freq to a bytes object.

`struct.pack(format, v1, v2, ...)`

Data is packed according to the format string parameter. https://docs.python.org/3.8/library/struct.html#format-characters

The format character 'f' is used for the standard size C and python 4 byte float type. `struct.pack('f',freq)`

Why is the byte size of 4 significant? The Arduino double data type is 4 bytes. The trig functions we will use later in Arduino use double data types. Here we insure that the number of bytes transmitted from python matches Arduino data requirements.

Study and run the [inputFreq.py](./frequencyAck/inputFreq.py) example. The output for the frequency input 64.0 is shown below. When developing and testing, this type of confirmation about data types, lengths, and memory content helps with the debugging process.

```
enter signal frequency in range [1.0,250.0] 64.0
freq data type: <class 'float'>
freqBytes data type: <class 'bytearray'>
freqBytes: bytearray(b'\x00\x00\x80B')
length of bytes object: 4
['0x00', '0x00', '0x80', '0x42']
```

The number of decimal places have not been restricted, but you may wish to do so. The round function can be used before converting the floating point value to a bytes object.

`freq = round(freq,2)`

Video tutorial: https://youtu.be/jXO2oevBSFE 

Now, we are ready to transmit the frequency from python to Arduino.

Debugging python to Arduino float transmission: https://youtu.be/pvRMq7sQlGQ

<br><br>

## Summary

We have broken down the homework problem into sub-problems and solved two major sub-problems. Next, write separate programs to implement Arduino generating and transmitting the signal data to python. Write the corresponding python program to receive the data. Writing these programs using functions will make integration into the final working program easier in the long run. Use functions and avoid global variables.
