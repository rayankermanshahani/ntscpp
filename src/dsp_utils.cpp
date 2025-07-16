#include "../include/dsp_utils.h"
#include <cmath>
#include <fftw3.h>
#include <mutex> // for std::once_flag
#include <thread>

std::once_flag fftw_init_flag;

// Initialize FFTW with multi-threading for performance
void init_fftw() {
  fftw_init_threads();
  fftw_plan_with_nthreads(std::thread::hardware_concurrency());
}

// Applies a low-pass filter to signal in-place using FFTW
// A low-pass filter removes high-frequency components above 'cutoff_freq' (Hz).
// In NTSC decoding, this isolates luminance (Y) which has bandwidth up to ~4.2
// MHz. Method: FFT to frequency domain, set bins above cutoff to zero, IFFT
// back. Normalization by 1/n on inverse to preserve amplitude. Efficient for
// long signals; FFTW optimizes for your hardware.
void apply_low_pass_filter(SignalSamples& signal, double cutoff_freq) {
  std::call_once(fftw_init_flag, init_fftw);

  size_t n = signal.size();
  if (n == 0)
    return;

  // Allocate buffers (FFTW requires aligned memory)
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

  // Zero out high-frequency bins (symmetric for real signals)
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

// Applies a band-pass filter to signal in-place using FFTW.
// Band-pass keeps frequencies in [center - bandwidth/2, center + bandwidth/2].
// In NTSC decoding, this isolates chrominance (C) around subcarrier (3.58 MHz,
// bandwidth ~1.3 MHz for I, 0.5 for Q, but approx combined). Same FFT method as
// low-pass, but zero outside the band.
void apply_band_pass_filter(SignalSamples& signal, double center_freq,
                            double bandwidth) {
  std::call_once(fftw_init_flag, init_fftw);

  size_t n = signal.size();
  if (n == 0)
    return;

  // Allocate buffers (FFTW requires aligned memory)
  double* in = static_cast<double*>(fftw_malloc(sizeof(double) * n));
  std::copy(signal.begin(), signal.end(), in);

  fftw_complex* freq = static_cast<fftw_complex*>(
      fftw_malloc(sizeof(fftw_complex) * (n / 2 + 1)));

  // Plan and execute forward real-to-complex FFT
  fftw_plan forward_plan =
      fftw_plan_dft_r2c_1d(static_cast<int>(n), in, freq, FFTW_MEASURE);
  fftw_execute(forward_plan);

  double fs = SAMPLING_RATE;
  size_t low_bin =
      static_cast<size_t>(std::round((center_freq - bandwidth / 2.0) * n / fs));
  size_t high_bin =
      static_cast<size_t>(std::round((center_freq + bandwidth / 2.0) * n / fs));

  // Zero out bins that lie outside the band
  for (size_t i = 0; i < low_bin; ++i) {
    freq[i][0] = 0.0;
    freq[i][1] = 0.0;
  }
  for (size_t i = high_bin + 1; i < n / 2 + 1; ++i) {
    freq[i][0] = 0.0;
    freq[i][1] = 0.0;
  }

  // Plan and execute inverse complex-to-real IFFT
  double* filtered = static_cast<double*>(fftw_malloc(sizeof(double) * n));
  fftw_plan inversed_plan =
      fftw_plan_dft_c2r_1d(static_cast<int>(n), freq, filtered, FFTW_MEASURE);
  fftw_execute(inversed_plan);

  // Scale and copy back to signal
  for (size_t i = 0; i < n; ++i) {
    signal[i] = filtered[i] / static_cast<double>(n);
  }

  // Cleanup
  fftw_destroy_plan(forward_plan);
  fftw_destroy_plan(inversed_plan);
  fftw_free(in);
  fftw_free(freq);
  fftw_free(filtered);
}

// Detects the phase offset of the color burst.
// The burst is a reference sine wave at subcarrier freq with known phase (180°
// offset). We demodulate by projecting onto cos (I) and sin (Q) components,
// then phase = atan2(Q, I). This locks the decoder's subcarrier phase per line.
double detect_burst_phase(const SignalSamples& burst_samples) {
  if (burst_samples.empty())
    return 0.0;

  double sum_i = 0.0; // cos projection
  double sum_q = 0.0; // sin projection
  double t = 0.0;
  double dt = 1.0 / SAMPLING_RATE;
  double omega = 2.0 * M_PI * SUBCARRIER_FREQ;

  for (double s : burst_samples) {
    sum_i += s * std::cos(omega * t);
    sum_q += s * std::sin(omega * t);
    t += dt;
  }

  double phase = std::atan2(sum_q, sum_i);

  // Adjust for NTSC burst being 180 degrees out-of-phase with reference
  phase += M_PI;
  return phase;
}

// Upsamples a low-rate signal to high-rate using linear interpolation
// Linear interp estimates values between samples by weighted average.
// Used in encoding to match pixel rate to sampling rate. Simple, but can
// introduce mild high-freq artifacts (fix later with better interp/filter).
void upsample_linear(const std::vector<double>& input, SignalSamples& output,
                     size_t output_size) {
  if (input.empty() || output_size == 0) {
    output.clear();
    return;
  }

  output.resize(output_size);
  double step = static_cast<double>(input.size() - 1) /
                static_cast<double>(output_size - 1);

  for (size_t i = 0; i < output_size; ++i) {
    double pos = static_cast<double>(i) * step;
    size_t idx = static_cast<size_t>(pos);
    double frac = pos - idx;

    if (idx + 1 < input.size()) {
      output[i] = input[idx] * (1.0 - frac) + input[idx + 1] * frac;
    } else {
      output[i] = input.back();
    }
  }
}

// Downsamples a high-rate signal to low-rate by averaging
// Averaging reduces rate while low-passing implicitly (anti-alias).
// Used in decoding to extract per-pixel Y/I/Q from oversampled signal. Step =
// input / output.
void downsample_average(const SignalSamples& input, std::vector<double>& output,
                        size_t output_size) {
  if (input.empty() || output_size == 0) {
    output.clear();
    return;
  }

  output.resize(output_size);
  double step =
      static_cast<double>(input.size()) / static_cast<double>(output_size);

  for (size_t i = 0; i < output_size; ++i) {
    size_t start = static_cast<size_t>(i * step);
    size_t end = static_cast<size_t>((i + 1) * step);
    if (end > input.size())
      end = input.size();

    double sum = 0.0;
    size_t count = end - start;
    for (size_t j = start; j < end; ++j) {
      sum += input[j];
    }
    output[i] = sum / static_cast<double>(count);
  }
}
