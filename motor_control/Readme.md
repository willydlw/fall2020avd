# Motor Control

## Introduction

Now we are ready to make our robot move around in the world. We start by learning to use the motor driver to specify DC motor direction and speed. After that, we learn how to use Arduino interrupts to read enoder data.

Our first milestone is establishing balance control, maintaining the robot in an upright position. Angular position data from the MPU6050 is the sensory input to a closed loop control system that actuates motor speed and direction to keep the robot upright, within a certain angular error tolerance. The control system should also be robust to minor disturbances to the robot's angular orientation.

The unit starts by presenting an overview of a low-level motor control, PID control systems, and a self-balancing algorithm.</br></br>

## Lessons

Lesson 1 presents low-level motor control functions using the TB6612FNG motor driver.

Lesson 2 introduces Arduino hardware interrupts.

Lesson 3 introduces motor encoders.

Lesson 4 discusses estimating velocity from encoder data.

Lesson B presents an overview of a PID control system and some examples illustrating the effects of the Kp and Kd parameters.

Lesson C discusses the PID code algorithm and two possible methods of tuning the control system.
</br></br>
