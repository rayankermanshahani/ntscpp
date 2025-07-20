/**
 * @file dsp_utils.cpp
 * @brief Provide DSP utilities for NTSC-M codec.
 *
 *
 * Implements FTT-based low-pass and band-pass filtering, burst-phase detection,
 * and linear sample rate conversion. References SMPTE 170 M and FCC 1953 NTSC
 * specification.
 */
#include "../include/dsp_utils.h"
#include <cmath>
#include <fftw3.h>
#include <mutex>
#include <thread>

std::once_flag fftw_init_flag;

/**
 * @brief Initialize FFTW for multithreaded operation.
 */
void init_fftw() {
  fftw_init_threads();
  fftw_plan_with_nthreads(std::thread::hardware_concurrency());
}

void apply_low_pass_filter(SignalSamples& signal, double cutoff_freq) {
  std::call_once(fftw_init_flag, init_fftw);

  size_t n = signal.size();
  if (n == 0)
    return;

  // Allocate aligned input buffers
  double* in = static_cast<double*>(fftw_malloc(sizeof(double) * n));
  std::copy(signal.begin(), signal.end(), in);

  // Allocate frequency-domain buffers
  fftw_complex* freq = static_cast<fftw_complex*>(
      fftw_malloc(sizeof(fftw_complex) * (n / 2 + 1)));

  // Plan and execute forward real-to-complex FFT
  fftw_plan forward_plan =
      fftw_plan_dft_r2c_1d(static_cast<int>(n), in, freq, FFTW_MEASURE);
  fftw_execute(forward_plan);

  // Comput ecutoff bin index
  double fs = SAMPLING_RATE;
  size_t cutoff_bin = static_cast<size_t>(std::round(cutoff_freq * n / fs));

  // Zero out high-frequency bins
  for (size_t i = cutoff_bin + 1; i < n / 2 + 1; ++i) {
    freq[i][0] = 0.0; // Real part
    freq[i][1] = 0.0; // Imag part
  }

  // Plan and execute inverse complex-to-real IFFT
  double* filtered = static_cast<double*>(fftw_malloc(sizeof(double) * n));
  fftw_plan inverse_plan =
      fftw_plan_dft_c2r_1d(static_cast<int>(n), freq, filtered, FFTW_MEASURE);
  fftw_execute(inverse_plan);

  // Normalize and copy back to signal
  for (size_t i = 0; i < n; ++i) {
    signal[i] = filtered[i] / static_cast<double>(n);
  }

  // Cleanup memory
  fftw_destroy_plan(forward_plan);
  fftw_destroy_plan(inverse_plan);
  fftw_free(in);
  fftw_free(freq);
  fftw_free(filtered);
}

void apply_band_pass_filter(SignalSamples& signal, double center_freq,
                            double bandwidth) {
  std::call_once(fftw_init_flag, init_fftw);

  size_t n = signal.size();
  if (n == 0)
    return;

  // Allocate aligned input buffer
  double* in = static_cast<double*>(fftw_malloc(sizeof(double) * n));
  std::copy(signal.begin(), signal.end(), in);

  // Allocate frequency-domain buffer
  fftw_complex* freq = static_cast<fftw_complex*>(
      fftw_malloc(sizeof(fftw_complex) * (n / 2 + 1)));

  // Plan and execute forward real-to-complex FFT
  fftw_plan forward_plan =
      fftw_plan_dft_r2c_1d(static_cast<int>(n), in, freq, FFTW_MEASURE);
  fftw_execute(forward_plan);

  // Compute band limits in bins
  double fs = SAMPLING_RATE;
  size_t low_bin =
      static_cast<size_t>(std::round((center_freq - bandwidth / 2.0) * n / fs));
  size_t high_bin =
      static_cast<size_t>(std::round((center_freq + bandwidth / 2.0) * n / fs));

  // Zero out bins below band
  for (size_t i = 0; i < low_bin; ++i) {
    freq[i][0] = 0.0;
    freq[i][1] = 0.0;
  }

  // Zero out bins above band
  for (size_t i = high_bin + 1; i < n / 2 + 1; ++i) {
    freq[i][0] = 0.0;
    freq[i][1] = 0.0;
  }

  // Plan and execute inverse complex-to-real IFFT
  double* filtered = static_cast<double*>(fftw_malloc(sizeof(double) * n));
  fftw_plan inversed_plan =
      fftw_plan_dft_c2r_1d(static_cast<int>(n), freq, filtered, FFTW_MEASURE);
  fftw_execute(inversed_plan);

  // Normalize and copy back to signal
  for (size_t i = 0; i < n; ++i) {
    signal[i] = filtered[i] / static_cast<double>(n);
  }

  // Cleanup memory
  fftw_destroy_plan(forward_plan);
  fftw_destroy_plan(inversed_plan);
  fftw_free(in);
  fftw_free(freq);
  fftw_free(filtered);
}

double detect_burst_phase(const SignalSamples& burst_samples) {
  if (burst_samples.empty())
    return 0.0;

  double sum_i = 0.0; // Cos projection
  double sum_q = 0.0; // Sin projection
  double t = 0.0;
  double dt = 1.0 / SAMPLING_RATE;
  double omega = 2.0 * M_PI * SUBCARRIER_FREQ;

  for (double s : burst_samples) {
    sum_i += s * std::cos(omega * t);
    sum_q += s * std::sin(omega * t);
    t += dt;
  }

  double phase = std::atan2(sum_q, sum_i);
  return phase;
}

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
