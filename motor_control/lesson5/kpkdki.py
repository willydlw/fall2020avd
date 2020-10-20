# Illustrate PID control
# algorithm for a velocity controller.
# 
# Acceleration is the control signal

import matplotlib.pyplot as plt 
import numpy as np 

# limits
accelLimit = 100.0   # m/s^2

# goal state
desiredVelocity = 1.0     # m/s

# PID control constants
kp = 5         # proportional gain constant
kd = 0.025     # derivative control constant
ki = 0.1875       # integral control constant

# initial values
acceleration = 0.3       # m/s^2
presentVelocity = 0.0     # m/s
prevError = 0.0
errorSum = 0.0

dt = 0.3       # sample time interval, sec
count = 0       # loop count

# list for plotting values
velocity = []
velocity.append(presentVelocity)

# simulate control loop
while count < 30:
    count += 1
    presentVelocity = presentVelocity + acceleration * dt
    velocity.append(presentVelocity)
    error = desiredVelocity - presentVelocity
    errorSum = errorSum + error * dt 
    derrdt = (error - prevError) / dt 
    print("\ncount: " + str(count))
    print("present velocity: " + str(presentVelocity))
    print("error: " + str(error))
    print("acceleration: " + str(acceleration))
    acceleration = kp * error + ki * errorSum + kd * derrdt 
    if abs(acceleration) > accelLimit:
        if acceleration > 0:
            acceleration = accelLimit 
        else:
            acceleration = -accelLimit

    print("updated acceleration: " + str(acceleration))
    prevError = error 

    

# plot 
t = np.arange(0.0, (count+1)*dt, dt)
msg = "Kp = " + str(kp) + ", Ki = " + str(ki) + ", Kd = " + str(kd)
plt.plot(t,velocity)
plt.grid(True)
plt.title(msg)
plt.xlabel("time [sec]")
plt.ylabel("velocity")
plt.savefig("kpkdki.png")
plt.show() 