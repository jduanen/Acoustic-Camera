import numpy as np
import matplotlib.pyplot as plt

fs = 1000
N = 1024
t = np.arange(N) / fs

x = np.sin(2*np.pi*50*t) + 0.5*np.cos(2*np.pi*120*t)

X = np.fft.fft(x)
f = np.fft.fftfreq(N, d=1/fs)

half = N // 2

plt.figure(figsize=(10, 8))

plt.subplot(4, 1, 1)
plt.plot(t, x)
plt.title("Time Domain Signal")
plt.xlabel("Time (s)")

plt.subplot(4, 1, 2)
plt.plot(f[:half], X.real[:half])
plt.title("FFT Real Part")
plt.xlabel("Frequency (Hz)")

plt.subplot(4, 1, 3)
plt.plot(f[:half], X.imag[:half])
plt.title("FFT Imaginary Part")
plt.xlabel("Frequency (Hz)")

plt.subplot(4, 1, 4)
plt.plot(f[:half], np.abs(X[:half]))
plt.title("FFT Magnitude")
plt.xlabel("Frequency (Hz)")

plt.tight_layout()
plt.show()

#

X = np.fft.rfft(x)
f = np.fft.rfftfreq(N, d=1/fs)

plt.plot(f, np.abs(X))
plt.xlabel("Frequency (Hz)")
plt.ylabel("Magnitude")
plt.title("Single-Sided FFT Magnitude")
plt.grid(True)
plt.show()

#

plt.plot(f, np.angle(X))
plt.xlabel("Frequency (Hz)")
plt.ylabel("Phase (rad)")
plt.title("FFT Phase")
plt.grid(True)
plt.show()
