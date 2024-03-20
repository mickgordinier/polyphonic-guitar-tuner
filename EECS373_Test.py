import numpy as np
import matplotlib.pyplot as plt
from numpy.fft import fft, ifft
from scipy.signal import find_peaks
import sounddevice as sd


BUFFER_LENGTH  = 4096
SAMPLING_RATE = 44100#4081.6
fundamental_freq = 82

lowest_expected_period = SAMPLING_RATE // 440 #440 Hz
highest_expected_period = SAMPLING_RATE // 70 #60 Hz


def record_audio(duration, sample_rate):
    # Record audio
    recording = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1, dtype='float32')
    sd.wait()

    # Convert recording to a 1D numpy array
    recording = np.squeeze(recording)

    return recording

def generate_signal():
    signal = np.zeros(BUFFER_LENGTH)
    for i in range(BUFFER_LENGTH):
        r = i / SAMPLING_RATE * 2 * np.pi * fundamental_freq
        # Adding harmonics up to the 5th harmonic
        s = np.sin(r) + np.sin(r*2) * 0.5 + np.sin(r*3) * 0.25 + np.sin(r*4) * 0.125 + np.sin(r*5) * 0.0625
        signal[i] = s
    return signal

def plot_signal(signal):
    plt.plot(signal)
    plt.xlabel('Sample')
    plt.ylabel('Amplitude')
    plt.title('Generated Signal')
    #plt.xlim(0, 500)
    plt.show()


def rxx(l, N, x):
    sum = 0
    for n in range(N - l - 1):
        sum += (x[n] * x[n + l])

    return sum / N


def autocorrelationWithShiftingLag(samples):
    autocorrelation = np.zeros_like(samples)
    for lag in range(np.size(samples)):
        autocorrelation[lag] = rxx(lag, np.size(samples), samples)
    return autocorrelation


def maxAbsoluteScaling(data):
    xMax = np.abs(np.max(data))
    return data / xMax
    
def autocorrelation_via_fft(x):
    # Compute the FFT and then (its magnitude)^2
    Fx = np.fft.fft(x)
    Pxx = np.abs(Fx)**2
    
    # Inverse FFT to get autocorrelation
    autocorrelation = np.fft.ifft(Pxx)
    autocorrelation = np.real(autocorrelation) 
    
    return autocorrelation[:len(x)//2]



if __name__ == "__main__":
    generated_signal = record_audio(1, SAMPLING_RATE)#generate_signal()
    generated_signal = np.hanning(len(generated_signal)) * generated_signal
    plot_signal(generated_signal)
    autocorrelated = autocorrelation_via_fft(generated_signal)#maxAbsoluteScaling(autocorrelationWithShiftingLag(generated_signal))
    plot_signal(autocorrelated)


    peaks, _ = find_peaks(autocorrelated, height=0)

    # Assuming the fundamental frequency peak will not be at the very beginning,
    # you start looking for peaks after some lag:
    lag_threshold = lowest_expected_period // 2
    valid_peaks = peaks[peaks > lag_threshold]  # `lag_threshold` depends on sampling rate & expected min frequency

    # Filter out peaks that are beyond the range we expect for guitar string frequencies
    valid_peaks = peaks[(peaks > lag_threshold) & (peaks < highest_expected_period)]

    if len(valid_peaks) > 0:
        # Choose the peak with the greatest autocorrelation value
        dominant_peak = valid_peaks[np.argmax(autocorrelated[valid_peaks])]
        period = dominant_peak
        freq = SAMPLING_RATE / period
    else:
        freq = None  # Handle the case where no valid peak was found

    print("Detected frequency: ", freq)


    # peaks, _ = find_peaks(autocorrelated, height=0)
    # dist = np.diff(peaks)
    # period = np.median(dist) / SAMPLING_RATE
    # freq = 1 / period

    # print(freq)
    ###

    # x = autocorrelated
    # X = fft(x)

    # X[500:] = 0



    # N = len(X)
    # n = np.arange(N)
    # T = N/SAMPLING_RATE
    # freq = n/T 

    # plt.figure(figsize = (12, 6))
    # plt.subplot(121)

    # plt.stem(freq, np.abs(X), 'b', \
    #         markerfmt=" ", basefmt="-b")
    # plt.xlabel('Freq (Hz)')
    # plt.ylabel('FFT Amplitude |X(freq)|')
    # plt.show()


