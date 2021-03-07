import numpy as np
import matplotlib.pyplot as plt

encoderRes = np.power(2,14)
latency = 32  # microSeconds
deltaTime = 142  # microSeconds

rpmRange = np.linspace(0,1450,50)
omegaMax = ((1450 / 60) * encoderRes ) * np.power(10.0,-6) #in ticks / us
omega = ((rpmRange / 60) * encoderRes ) * np.power(10.0,-6) #in ticks / us

deltaThetaMax = (omegaMax * deltaTime) #in ticks, the max we'll see at 1450rpm
deltaTheta = np.round(omega * deltaTime) # encoderTicks

advance = np.round(omega * latency)  # ticks, this is required advance angle at the pwm update

displayOmega =((deltaTheta/deltaTime) *  np.power(10,6) / encoderRes)* 60 #in rpm (this is what we will display)
print(displayOmega)

fig1, ax = plt.subplots(1,2,figsize=(8,4))
ax[0].step(deltaTheta,advance)
ax[0].set_xlabel("deltaTheta (counts)")
ax[0].set_ylabel("advance angle (counts)")

ax[1].plot(rpmRange, rpmRange - displayOmega)
ax[1].set_xlabel("motor velocity (rpm)")
ax[1].set_ylabel("display error (rpm)")
plt.tight_layout()
plt.show()


12 steps / 190 us