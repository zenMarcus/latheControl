import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


df = pd.read_csv('data/intSignals01.csv')

#df['sinTheta'] = np.sin(-df.theta)
#df['cosTheta'] = np.cos(-df.theta)
print(df.head())

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)

ax1.plot(df.index, df.encAngle)
ax1.plot(df.index, df.magAngle)


ax1.set(xlabel='sample', ylabel='value', title='phases')
ax1.grid()
ax1.legend()


ax2.plot(df.index, df.focAngle)

#ax2.plot(df.index, df.Y)
#ax2.plot(df.sinTheta)
#ax2.plot(df.cosTheta)

ax2.set(xlabel='sample', ylabel='value', title='Clarke')
ax2.grid()
ax2.legend()

ax3.plot(df.index, df.pwm)
ax3.plot(df.index, df.bestSine)

ax3.set(xlabel='sample', ylabel='value', title='Park')
ax3.grid()
ax3.legend()
plt.tight_layout(pad=0.4, w_pad=0.5, h_pad=0.1)
plt.show()
