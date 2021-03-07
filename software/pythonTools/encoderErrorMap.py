import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from scipy import signal

encoderResolution = pow(2, 14)
magIndex = 1024 * 14
sampleNumber = int(encoderResolution / 16)
index = np.linspace(0, encoderResolution, magIndex, endpoint=False)
nominalEncoder = np.linspace(0, encoderResolution, encoderResolution, endpoint=False)

print(sampleNumber)
df = pd.read_csv('data/encoderMap6148.csv')
print(df.describe())
# calculate linearity error
encoderError = (df.encoder - index)

# resample encoder from magindex to encoder resolution
resampledEncoder = signal.resample(df.encoder, encoderResolution)

# create a LUT by down sampling - now done after filtering
# resampledLut = signal.resample(encoderError, sampleNumber)

# creat a 2D array to write to .csv
outArray = np.array([nominalEncoder, resampledEncoder])
#np.savetxt('data/encoderErrorSignal6148.csv', np.transpose(outArray), delimiter=',', fmt=['%d', '%f'])

# plot everything
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

ax1.plot(index, df.encoder)

ax1.set(xlabel='sample', ylabel='encoder value', title='raw encoder vs poles')
ax1.grid()
ax1.legend()

ax2.plot(index, encoderError, label='error')

ax2.set(xlabel='lut index', ylabel='encoder error (counts)', title='error')
#ax2.step(np.linspace(0, encoderResolution, sampleNumber, endpoint=False), resampledLut, label='lut')

ax2.grid()
ax2.legend()
#
# ax3.plot(df.index, df.bestSine)
#
# ax3.set(xlabel='sample', ylabel='value', title='Park')
# ax3.grid()
# ax3.legend()
plt.tight_layout(pad=0.4, w_pad=0.5, h_pad=0.1)
plt.show()
