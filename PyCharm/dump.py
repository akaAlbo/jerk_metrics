'''
@author: flg-ma
@version: 1.1
@note: dump for all written, but unused code
'''

import matplotlib.pyplot as plt
import numpy as np
from scipy import signal


def butter_bandpass(lowcut, highcut, fs, order=5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = signal.butter(order, [low, high], btype='low')
    return b, a


def butter_bandpass_filter(data, lowcut, highcut, fs, order=5):
    """
    @param data: data to filter
    @param lowcut: lowpass cutoff frequency
    @param highcut: highpass cutoff frequency
    @param fs: sample rate
    @param order: order of butterworth filter
    """
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    y = signal.lfilter(b, a, data)
    return y


b1, a1 = signal.butter(5, .01)
y1 = signal.filtfilt(b1, a1, A[:, AD.VEL_X])
plt.figure(n + 4, figsize=(16.0, 10.0))
plt.title('Filtfilt', fontsize=20)
plt.plot(A[:, AD.TIME], A[:, AD.VEL_X], 'b', alpha=0.75, label='noisy signal')
plt.plot(A[:, AD.TIME], y1, 'k', label='filtfilt')
plt.xlabel('Time [s]', fontsize=20)
plt.ylabel('v [m/s]', fontsize=20)
plt.legend(loc='best', fontsize=20)
plt.grid(True)
plt.savefig('filtfilt.pdf', bbox_inches='tight')
plt.draw()

fs = 50
lowcut = 1
highcut = 10

plt.figure(n + 5, figsize=(16.0, 10.0))
y = butter_bandpass_filter(A[:, AD.VEL_X], lowcut, highcut, fs, order=6)
plt.title('Butterworth Bandpass', fontsize=20)
plt.plot(A[:, AD.TIME], y, 'r', label='Filtered signal (%g Hz)' % 50)
plt.plot(A[:, AD.TIME], A[:, AD.VEL_X], 'b', alpha=0.75, label='noisy signal')
plt.xlabel('Time [s]', fontsize=20)
plt.ylabel('v [m/s]', fontsize=20)
plt.grid(True)
plt.axis('tight')
plt.legend(loc='best', fontsize=15)
plt.draw()
plt.savefig('butterworth_bandpass.pdf', bbox_inches='tight')
# plt.show()
