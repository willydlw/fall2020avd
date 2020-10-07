# Illustrate Proportional control
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
kp = 7         # proportional gain term

# initial values
acceleration = 0.3       # m/s^2
presentVelocity = 0.0     # m/s


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
    print("\ncount: " + str(count))
    print("present velocity: " + str(presentVelocity))
    print("error: " + str(error))
    print("acceleration: " + str(acceleration))
    if error > 0:   # present velocity is too slow, less than desiredVelocity
        acceleration = min([ kp * error, accelLimit])
    elif error < 0: # present velocity too fast, decrease it
        acceleration = max([kp*error, -accelLimit])
    else: # error == 0, at desired speed
        acceleration = 0    
    print("updated acceleration: " + str(acceleration))

    

# plot 
t = np.arange(0.0, (count+1)*dt, dt)
msg = "Kp = " + str(kp)
plt.plot(t,velocity)
plt.grid(True)
plt.title(msg)
plt.xlabel("time [sec]")
plt.ylabel("velocity")
plt.savefig("kp.png")
plt.show() 