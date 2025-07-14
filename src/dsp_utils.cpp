#include "dsp_utils.h"
#include "ntsc_common.h"
#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <fftw3.h>
#include <mutex> // For std::once_flag
#include <thread>

std::once_flag fftw_init_flag;

// Initialize FFTW with multi-threading for performance
void init_fftw() {
  fftw_init_threads();
  fftw_plan_with_nthreads(std::thread::hardware_concurrency());
}

// Applies a low-pass filter to signal in-place using FFTW
void apply_low_pass_filter(SignalSamples& signal, double cutoff_freq) {
  std::call_once(fftw_init_flag, init_fftw);

  size_t n = signal.size();
  if (n == 0)
    return;

  // allocate buffers (FFTW requires aligned memory)
  double* in = static_cast<double*>(fftw_malloc(sizeof(double) * n));
  std::copy(signal.begin(), signal.end(), in);

  fftw_complex* freq = static_cast<fftw_complex*>(
      fftw_malloc(sizeof(fftw_complex) * (n / 2 + 1)));

  // Plan and execute forward real-to-complex FFT
  fftw_plan forward_plan =
      fftw_plan_dft_r2c_1d(static_cast<int>(n), in, freq, FFTW_MEASURE);
  fftw_execute(forward_plan);

  // Calculate cutoff bin (freqs are 0 to fs/2 over n/2+1 bins)
  double fs = SAMPLING_RATE;
  size_t cutoff_bin = static_cast<size_t>(std::round(cutoff_freq * n / fs));

  // Zero high-frequency bins (symmetric for real signals)
  for (size_t i = cutoff_bin + 1; i < n / 2 + 1; ++i) {
    freq[i][0] = 0.0; // Real part
    freq[i][1] = 0.0; // Imag part
  }

  // Plan and execute inverse complex-to-real IFFT
  double* filtered = static_cast<double*>(fftw_malloc(sizeof(double) * n));
  fftw_plan inverse_plan =
      fftw_plan_dft_c2r_1d(static_cast<int>(n), freq, filtered, FFTW_MEASURE);
  fftw_execute(inverse_plan);

  // Scale and copy back to signal
  for (size_t i = 0; i < n; ++i) {
    signal[i] = filtered[i] / static_cast<double>(n);
  }

  // Cleanup
  fftw_destroy_plan(forward_plan);
  fftw_destroy_plan(inverse_plan);
  fftw_free(in);
  fftw_free(freq);
  fftw_free(filtered);
}

// Applies a band-pass filter to signal in-place using FFTW
void apply_band_pass_filter(SignalSamples& signal, double center_freq,
                            double bandwidth) {
  std::call_once(fftw_init_flag, init_fftw);

  size_t n = signal.size();
  if (n == 0)
    return;

  double* in = static_cast<double*>(fftw_malloc(sizeof(double) * n));
  std::copy(signal.begin(), signal.end(), in);
}

// Detects the phase offset of the color burst
double detect_burst_phase(const SignalSamples& burst_samples) {}

// Upsamples a low-rate signal to high-rate using linear interpolation
void upsample_linear(const std::vector<double>& input, SignalSamples& output,
                     size_t output_size) {}

// Downsamples a high-rate signal to low-rate by averaging
void downsample_average(const SignalSamples& input, std::vector<double>& output,
                        size_t output_size) {}
