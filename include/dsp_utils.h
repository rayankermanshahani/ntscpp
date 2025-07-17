/**
 * @file dsp_utils.h
 *
 * Declarations for DSP utility functions used in NTSC signal processing.
 * Includes filters, phase detection, and resampling functions.
 */

#ifndef DSP_UTILS_H
#define DSP_UTILS_H

#include "ntsc_common.h"

/**
 * @brief Apply a low-pass filter to signal in-place via FFT.
 *
 * Remove high-frequency components above cutoff_freq Hz by forward FFT, zeroing
 * bins, and inverse FFT.
 *
 * @param signal Samples to filter in-place.
 * @param cutoff_freq  Cutoff frequency in Hz.
 */
void apply_low_pass_filter(SignalSamples& signal, double cutoff_freq);

/**
 * @brief Apply band-pass filter to signal in-place via FFT.
 *
 * Keep frequencies in [center_freq ± bandwidth/2] Hz and remove others.
 *
 * @param signal Samples to filter in-place.
 * @param center_freq Center frequency in Hz.
 * @param bandwidth Bandwidth in Hz.
 */
void apply_band_pass_filter(SignalSamples& signal, double center_freq,
                            double bandwidth);
/**
 * @brief Detect phase offset of NTSC color burst.
 *
 * Project burst_samples onto cos and sin at subcarrier frequency then compute
 * phase with correction for 180 degrees offset.
 *
 * @param burst_samples Samples of color burst segment.
 * @return Phase offset in radians.
 */
double detect_burst_phase(const SignalSamples& burst_samples);

/**
 * @brief Upsample a signal by linear interpolation.
 *
 * Estimate intermediate samples via weighted average between adjacent input
 * samples.
 *
 * @brief input Input samples at low rate.
 * @brief output Output container for high-rate samples.
 * @brief output_size Desired number of output samples.
 */
void upsample_linear(const std::vector<double>& input, SignalSamples& output,
                     size_t output_size);

/**
 * @brief Downsample signal by averaging.
 *
 * Partition input into output_size segments and average each segment to produce
 * one output sample.
 *
 * @brief input Input samples at high rate.
 * @brief output Output container for low-rate averages.
 * @brief output_size Desired number of output samples.
 */
void downsample_average(const SignalSamples& input, std::vector<double>& output,
                        size_t output_size);

#endif // DSP_UTILS_H
