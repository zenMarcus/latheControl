import matplotlib.pyplot as plt
import numpy as np
from scipy import signal
from scipy.fftpack import fft, fftfreq

# load matlab dataset
# matlabData = loadmat('data/filename.mat', mat_dtype=True)
# t, mySignal = np.array(matlabData['ans'])

# load csv dataset
t, mySignal = np.loadtxt('data/encoderMap02.csv', skiprows=1, usecols=(0, 1), delimiter=',', unpack=True)
error = t * (pow(2, 14) / (14 * 1024)) - mySignal
mySignal = error
t = t * 0.00025
# define the domain according to the dataset length
t0 = t[0]  # s
t1 = t[len(t) - 1]  # s
sampleCount = len(t) - 1

# calculate sampling frequency
samplingFrequency = sampleCount / (t1 - t0)
samplingPeriod = 1 / samplingFrequency
sampleCount = int((t1 - t0) * samplingFrequency)
print("sampling frequency = ", samplingFrequency, "Hz")
print("sampling period = ", samplingPeriod, "s")
print("number of samples = ", sampleCount)

# define a frequency for the notch filter
notchFrequency = 1  # Hz


# generate a test time domain
# t = np.linspace(t0, t1, num=sampleCount, endpoint=True)

# here is a f(x) function / signal
def f(x):
    y = np.sin(x * (4 * np.pi))  # 2Hz signal
    y = y + (np.sin(x * ((320 * 2) * np.pi))) / 2  # + 80Hz noise
    return y


# generate the test signal data within the time domain for tests
# mySignal = f(t)


# FFT of the input signal
win = signal.windows.hamming(sampleCount + 1)
# win = signal.windows.kaiser(sampleCount+1, 28)
# win = signal.windows.tukey(sampleCount+1, 0.6, sym=True)


# extract spectrum with and without window
spectrum = fft(mySignal)
spectrumWin = fft(mySignal * win)

# generate frequencies array
spectrumFreq = fftfreq(spectrum.size, samplingPeriod)

print("including the imaginary half")
print("spectrum array len =", len(spectrum))
print("spectrumFreq array len =", len(spectrumFreq))

# now select and design a filter
# b, a = signal.iirnotch(w0=notchFrequency, Q=40, fs=samplingFrequency)
# b, a = signal.iirpeak(w0=22, Q=2, fs=samplingFrequency, )
# b, a = signal.cheby2(N=4, rs=4, Wn=0.1, btype='lowpass', analog=False)
b, a = signal.butter(N=5, Wn=4, fs=samplingFrequency, btype='lowpass', analog=False)  # critical

# bandpass butter
# N, Wn = signal.buttord([10, 80], [1, 90], 4, 5, True)
# b, a = signal.butter(N, Wn, fs=samplingFrequency, btype='band', analog=False)

# b, a = signal.iircomb(w0=50, Q=5, ftype='notch', fs=samplingFrequency)


# create it's charcteristic
freq, h = signal.freqz(b, a, fs=samplingFrequency)

# and apply the filter
filteredSignal = signal.filtfilt(b, a, x=mySignal)
resampledSignal = signal.resample(filteredSignal, 1024)
lut = resampledSignal.astype(int)

np.savetxt('data/encoderErrorLut.csv', lut, fmt='%d')
# fft of filteres signal
# spectrumWin = fft(filteredSignal)


# plot the filter characteristic ------------------
# fig, ax = plt.subplots(3, 1, tight_layout=True)

fig1 = plt.figure(figsize=[16,8],constrained_layout=False)
gs = fig1.add_gridspec(2, 2)
ax1 = fig1.add_subplot(gs[1, :-1])
ax2 = fig1.add_subplot(gs[0, :-1])
ax3 = fig1.add_subplot(gs[0:, 1])

ax1.plot(freq, 20 * np.log10(np.abs(h)), color='blue')
ax1.set_title("Filter Frequency Response")
ax1.set_ylabel("Amplitude (dB)", color='blue')
ax1.set_xlim([0, samplingFrequency / 2])
ax1.set_ylim([-50, 10])
ax1.set(xlabel='frequency (Hz)')
ax1.grid()

# plotting only the positive real part of fft being a real sequence
ax2 = plt.subplot(221, sharex=ax1)
ax2.set_title("FFT")
# straight fft
ax2.semilogy(spectrumFreq[0:sampleCount // 2], np.abs(spectrum[0:sampleCount // 2]), color='blue', lw=0.1)

# window fft
ax2.set_ylabel("Amplitude", color='blue')
ax2.semilogy(spectrumFreq[0:sampleCount // 2], np.abs(spectrumWin[0:sampleCount // 2]), color='red', linewidth=0.3)
ax2.grid()
ax2.set(xlabel='frequency (Hz)')

# then the signals
ax3.plot(t, mySignal, color="green", linewidth=0.3, label='Signal')
ax3.plot(t, filteredSignal, color="blue", linewidth=0.6, label='filtered signal')
ax3.step(np.linspace(0, sampleCount * 0.00025, 1024), lut, color="orange", linewidth=1, label='resampled')

ax3.set(xlabel='time (s)', ylabel='Signal')
ax3.legend(loc=1)
ax3.grid()

# fig.savefig("test.png")
plt.tight_layout()
plt.show()