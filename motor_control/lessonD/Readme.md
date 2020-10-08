# Lesson 3: Self-Balancing Control System Using Gyroscope & Accelerometer

Control - The Inverted Pendulum Problem

Balancing robots represent the classic inverted pendulum problem, in which a large mass is placed at the end of a pole. The pole is free to rotate around the base, and the base is free to move in the plane perpendicular to the vertical. The goal is to keep the pole vertical by moving the base in response to changes in the angle.

![Inverted Pendulum](./images/Mobile-wheeled-inverted-pendulum-MWIP-system-model.png "inverted pendulum")

[1] https://www.researchgate.net/profile/Jian_Huang14/publication/285576864/figure/fig6/AS:328305132752907@1455285451614/Mobile-wheeled-inverted-pendulum-MWIP-system-model.png 


An inverted pendulum is inherently unstable. The slighest disturbance from the equilibrium position produces a force away from the equilibrium point that destabilizes the system. Balancing the robot requires a low-latency control system to instantly correct any errors in tilt.


## How do we balance the robot?

We keep the robot balanced by continually moving the wheels under the vehicle as it falls. The robot's wheels are driven in the direction of its fall to counteract the fall. That keeps the robot's center of gravity above its pivot point. We will implement a PID controller that uses tilt feedback to control the motor and keep the robot balanced.


## PID Control System

The control system will compare the robot's tilt angle to the desired target angle and calculate the error difference. The PID controller then calculates the motor control signal needed to correct the error.

General Algorithm<br>
- Measure angle of inclination
- Determine direction of fall
- Send command to motors to drive in direction of fall


**Proportional Control**

Proportional control adjusts for the current system error. The control signal adjustment term is proportional to the error, scaled by the constant parameter Kp. In our application, the proportional term scales the angle error and sends that scaled value to the motors, to keep the wheels rolling into the fall direction. The further the robot falls off target, the faster the motors move. If the P-component is used on its own, the robot might stabilise for a while, but the system will tend to overshoot, oscillate and ultimately fall over. 

**Integral Control**

In a dynamic system, disturbances over time, cause the proportional control to produce an offset value. The integral controller is used to remove the offset and bring the error back to zero. The control system adjustment term is based on the accumulated error over time. 

**Derivative Control**

The derivative control looks at the rate of change in the error, trying to predict the future error. The amount of derivative correction is based on how fast the error is changing. The derivative term is critical for dampening any oscillations and vibration. 


![PID Control System](./images/pid.jpg "Self-balancing PID control system")

<br>
<br>

## PID Software Implementation


**main function control loop algorithm**

```
variables:
 target - desired robot tilt angle, reference point
 nextTime - next time to run control loop 
 angle - current robot angle

while true
    if nextTime < clock time
        nextTime = clock time + control loop interval time
        angle = getAngle()
        motorOutput = PID(target, angle)
        moveMotors(motorOutput)
```


**PID Control function algorithm**

```
PID( targetAngle, currentAngle)
    // declare tuning parameter constants
    KP = ?,  KD = ?, KI = ?

    // static function variables are initialized once at start
    // retain values from one function call to the next
    static lastTime = 0   
    static iterm = 0

    // initialize to the expected starting angular position
    static prevAngle = 90?     

    currentTime = clock time           // read timer value
    dT = currentTime - lastTime
    lastTime = currentTime

    error = targetAngle - currentAngle

    // calculate integral term, the cumulative error
    iterm = iterm + error * dT

    // calculate derivative term, the rate error
    dterm = (current - prevAngle) / dT

    // update previous to current
    prevAngle = currentAngle

    // calculate pid value
    pid = (error * KP) + (iterm * KI) + (dTerm * KD)

    // limit pid to max value
    if pid not within range, constrain

    return pid
```

## Tuning - How do we determine the K constants?


### Step 1 - Calibrate the IMU Sensor

Put the robot in its vertical upright, balanced position to determine the calibration offset values. Hold it there while the sensor calibrates.

Record the calibration values. If your system returns relatively the same values every time it calibrates the IMU sensor, you may want to alter the robot's software to set these values at startup, rather than run the calibration routine each time.

After calibration, what is the robot's tilt angle? This may be the roll (rotation about y) angle or the pitch (rotation about x) angle, depending on how the IMU is mounted on the robot. 


### Step 2 - Determine tilt angle range

Rotate the robot approximately 90 degrees forward and then 90 degrees backward. Do this more than once to establish a consistent range. If your sensor is well-calibrated, you may see tilt angles values from -89 to 89. 

With the acrylic guard on the robot, the angular range on one side will be constrained. Record the tilt range. 

What is the angle when the robot is held in a static upright position after rotation tests? Is there a lot of drift? If so, you may need to change the alpha value on the complementary filter.

It is suggested that the accelerometer tilt angle is low pass filtered and then passed through the complementary filter with the gyroscope angle. If this does not work for your hardware, then use whatever filtering method best works.


### Step 3 - Balancing States

Now that you have determined the balanced angle, this will become the set point. The image below shows a robot in the balanced state, deviation to the left state, and deviation to the right state. 


![balancing](./images/balance.jpeg "balancing")

https://miro.medium.com/max/2000/0*N-jU3hUGtPsgB_tn 

<br>
<br>

**Balance State**

In the image above, the tilt angle (ROLL) value is about 3 degrees. The robot's angle to the ground is about 87 degrees. The set point is 3 degrees, with an error of 0 degrees. The motors are not turning the wheels.


**Deviation to Right State**

When the tilt angle (ROLL) is greater than 3 degrees, the robot is falling to the right. The motors must drive in that direction to rebalance the robot. This may be clockwise or counter clockwise motor rotation, depending on your hardware configuration.


**Deviation to Left State**

When the tilt angle (ROLL) is less than 3 degrees, the robot is falling to the left. The motors must drive in that direction to rebalance the robot. This may be clockwise or counter clockwise motor rotation, depending on your hardware configuration.


### Step 4 - Tuning the Parameters

There are many algorithms for tuning PID controllers. One of those methods is presented below. The tuning process is a time consuming trial and error process. A review of each term's effect is presented before the tuning method steps. 


**Proportional Term Kp**

The proportional gain term changes the controller output proportional to the current error value. Larger Kp values typically mean a faster response: the larger the error, the larger the correction signal. Excessively large proportional gaing will lead to instability and oscillation. If the robot is wobbly or very off balance, a larger Kp term may be needed.


**Integral Term Ki**

The integral term is proportional to the amount of time the error is present. The integral term accelerates the movement of the control process towards the set value and eliminates residual steady-state error that occurs with a proportional only controller. The contribution from the integral term is dependent on both the magnitude of the error and the duration of the error.

The robot's motors and wheels are not perfect. Friction, power losses, and other factors may cause the output to be a little off from the commanded output. The integral term Ki adds up this error and affects the control system output as the error grows larger.

Example: The MPU should read 90 to be balanced, but reads 90.1. The extra 0.1 degree is a very small error and may not generate a command to turn the motors to correct that. If the sample time for reading the sensor is 100 time per second, the extra 0.1 degree of error is accumulated in the integral sum. Over the course of 1 second that is 0.1 degree error * 100 = 10 degree error.

Ki determines the time for the robot to correct itself. A large Ki will help the robot to steay itself quickly.


**Derivative Term Kd** 

The derivative controller output is proportional to the rate of change of the error. The derivative term slows the rate of the controller output value. It reduces the magnitude of the overshoot and improves stability. Larger Kd values decrease overshoot but slows down transient response to noise which can lead to instability.


The PI controller may be all that is needed to keep the robot balanced. Not all systems require the Kd term. Let's add it to the system. Start with a very small value. Increase Kd slowly until the system begins to oscillate. At that point, go back to the previous Kd value that did not cause oscillation.



**Tuning Method 1**

- Set Kp, Ki, and Kd to zero
- Adjust Kp until the system remains in balance, but rapidly and consistently oscillates around equilibrium.
- Adjust Kd until the system reaches steady state
- If there is a steady state error, tune Ki. The Ki term may not be necessary. If it makes the system worse, do not use it.


**Ziegler-Nichols Method**

Start with a low value of Kp, increasing it until the system reaches the ultimate gain Ku, which is the largest gain at which the control loop output has stable and consistent oscillations. Gains larger than Ku produce diverging oscillations. If following the Ziegler-Nichols method, use the Ku oscillation period Tu to set the P, I, and D gains. 

Example: Start by setting Kp to a low value, 0.05, and observe the robot behavior. Lightly hold the robot upright as you start it. There may not be enough power to move the motors when Kp is too low. Kp * error is used to control the PWM duty cycle that runs the motors. A low duty cycle corresponds to a low voltage.

Increase Kp to larger values until you find Ku. Observe the robot behavior for each Kp value. Is it consistently oscillating and stable? If so, you have found Ku. You will need the period of oscillation. Connect a computer to the robot with a long USB cable to record the controller output. Plot the output to verify the oscillation period Tu is consistent.


![Ziegler-Nichols method](./images/zieglerNichols.png "Ziegler-Nichols method") [1](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method)


Now, that you have Ku, Tu, you can derive and test Ziegler-Nichols values for Ki, and Kd. The Ki and Kd factors depend upon which system you are implementing: P, PI, PD, or PID. Often, a PD system is sufficient for the self-balancing robot. 

Ziegeler-Nichols does not always work. 


**Other Tuning Methods**

There are several YouTube videos and blogs describing how to tune a PID control system for a self-balancing robot. You are encouraged to find a better, easier method than the ones described above.


**Robot turns left or right**

Your robot may occassionally turn left or right due to mismatched motors, the surface it's on, or other disturbances. Do not worry about this. We will learn to control that in a future lesson.