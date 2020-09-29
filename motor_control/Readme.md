# Motor Control

## Introduction

Now we are ready to make our robot move around in the world. It is not a simple matter of writing a line of code that specifies direction and speed.

Our first milestone is establishing balance control, maintaining the robot in an upright position. Angular position data from the MPU6050 is the sensory input to a closed loop control system that actuates motor speed and direction to keep the robot upright, within a certain angular error tolerance. The control system should also be robust to minor disturbances to the robot's angular orientation.

The unit starts by presenting an overview of a PID control system, low-level motor control, and a self-balancing algorithm. 



## Lessons

Lesson 1 presents an overview of a PID control system and some examples illustrating the effects of the Kp and Kd parameters.

Lesson 2 presents low-level motor control functions using the TB6612FNG motor driver.

Lesson 3 discusses the PID code algorithm and two possible methods of tuning the control system.

Lesson 4 introduces motor encoders.

Lesson 5 discusses estimating velocity from encoder data.
