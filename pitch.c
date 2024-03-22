#include <fftw3.h>
#include "math.h"
#include <stdint.h>

#define BUFFER_LENGTH 4096
#define SAMPLING_RATE 4081.6


//arm_rfft_fast_instance_f32 fftHandler;


void apply_hanning_window(double * signal, uint32_t length){
	for (int i = 0; i < length; ++i){
		double han_value = 0.5f * (1.0f - cosf(2 * 3.14158265359 * i / (length - 1)));
		signal[i] *= han_value;
	}
}

void find_peaks(double* data, uint32_t length, int32_t* peaks, uint32_t* num_peaks) {
    double threshold = 0; // Adjust if needed
    *num_peaks = 0;

    for (uint32_t i = 1; i < length - 1; ++i) {
        if (data[i] > threshold && data[i] > data[i-1] && data[i] > data[i+1]) {
            peaks[*num_peaks] = i;
            (*num_peaks)++;
        }
    }
}

void autocorrelate(double* x, int N, double* autocorrelation) {
    // Allocate input and output arrays for the FFT
    fftw_complex *in, *out;
    in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
    out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);

    // Copy the real signal into the complex input array
    for (int i = 0; i < N; i++) {
        in[i][0] = x[i]; // Real part
        in[i][1] = 0.0;  // Imaginary part
    }

    // Create a plan to calculate the FFT of the signal
    fftw_plan plan_forward = fftw_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
    
    // Execute the plan to calculate the FFT
    fftw_execute(plan_forward);
    
    // Calculate the power spectrum (magnitude squared of the FFT)
    for (int i = 0; i < N; ++i) {
        double real = out[i][0];
        double imag = out[i][1];
        out[i][0] = real * real + imag * imag; // Magnitude squared
        out[i][1] = 0.0; // Zero out the imaginary part since it should not affect the result
    }

    // Create a plan to calculate the inverse FFT of the power spectrum
    fftw_plan plan_backward = fftw_plan_dft_1d(N, out, in, FFTW_BACKWARD, FFTW_ESTIMATE);
    
    // Execute the plan to calculate the inverse FFT
    fftw_execute(plan_backward);
    
    // Copy the real part of the inverse FFT result into the autocorrelation array
    for (int i = 0; i < N; ++i) {
        autocorrelation[i] = in[i][0] / N; // Scale result by N for proper normalization
    }

    // Free the memory allocated for the FFTW arrays and plans
    fftw_destroy_plan(plan_forward);
    fftw_destroy_plan(plan_backward);
    fftw_free(in);
    fftw_free(out);
}
int main(void)
{
  /* USER CODE BEGIN 1 */

	//convFlag = 0;

	// float32_t string_freqs[6] = {329.63, 246.94, 196.0, 146.83, 110.0, 82.41};
	// float32_t measured_freqs[6];



  double signal[BUFFER_LENGTH];
  double FFT_OUT[BUFFER_LENGTH];
  double autocorrelation[BUFFER_LENGTH];

  uint32_t LOWEST_PERIOD = SAMPLING_RATE / 440; //440hz max
  uint32_t HIGHEST_PERIOD = SAMPLING_RATE / 60; //60 Hz min (change to 50)



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


	  //Test signal with harmonics
	  for (int i = 0; i < BUFFER_LENGTH; ++i){
		  double r = (double)i / (double)SAMPLING_RATE;
		  r *= 3.14158265359 * 2;
		  r *= 110; //Hz

		  double s = sin(r); //+ sin(r*4) * 0.5 + sin(r*3) * 0.25;
		  signal[i] = s;
	  }



	apply_hanning_window(signal, BUFFER_LENGTH);
	autocorrelate(signal, BUFFER_LENGTH, autocorrelation);

    // Print the result (up to N/2)
    // printf("Autocorrelation:\n");
    // for (int i = 0; i < 100; i++) {
    //     printf("%f\n", autocorrelation[i]);
    // }
	 

	  uint32_t peaks[BUFFER_LENGTH/2];
	  uint32_t num_peaks = 0;

	  find_peaks(autocorrelation, BUFFER_LENGTH/2, peaks, &num_peaks);


	  //I DONT THINK THE CODE BELOW IS CORRECTLY IMPLEMENTING THE PYTHON CODE AND GETTING VALID PEAKS
	 //GOD DAMMIT I FUKING HATE C MAN WHY CANT I JUST LOGICALLY INDEX OR USE VECTORS AT THE VERY LEAST

	  double freq = 0.0f;
	     if (num_peaks > 0) {
	         // Get the highest valid peak
	         uint32_t valid_peak_index = 0;
	         double max_value = 0;
	         for (uint32_t i = 0; i < num_peaks; ++i) {
	             int32_t peak = peaks[i];
	             //peak greater than lowest period and smaller than largest possible period
	             if (peak > LOWEST_PERIOD && peak < HIGHEST_PERIOD) {
	                 if (autocorrelation[peak] > max_value) {
	                     valid_peak_index = peak;
	                     max_value = autocorrelation[peak];
	                 }
	             }
	         }
	         if (max_value > 0) {
	             freq = (double)SAMPLING_RATE / valid_peak_index;
	         }
	     }

	     double fundamental_freq = freq;
		 printf("%f\n", fundamental_freq);
  /* USER CODE END 3 */
}