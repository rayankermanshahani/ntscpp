#ifndef DSP_UTILS_H
#define DSP_UTILS_H

#include "ntsc_common.h"

void apply_low_pass_filter(SignalSamples& signal, double cutoff_freq);
void apply_band_pass_filter(SignalSamples& signal, double center_freq,
                            double bandwidth);
double detect_burst_phase(const SignalSamples& burst_samples);
void upsample_linear(const std::vector<double>& input, SignalSamples& output,
                     size_t output_size);
void downsample_average(const SignalSamples& input, std::vector<double>& output,
                        size_t output_size);

#endif // DSP_UTILS_H
