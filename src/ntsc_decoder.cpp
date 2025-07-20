/**
 * @file ntsc_decoder.cpp
 * @brief Decode base-band NTSC-M composite signal sampled at 4 x Fsc into video
 * frames.
 *
 * Implements sync detection, VBI skipping, burst phase extraction, Y/C
 * separation, quadrature demodulation, downsampling, and YIQ->RGB conversoin
 * per SMPTE 170 M and FCC 1953.
 */

#include "../include/dsp_utils.h"
#include "../include/ntsc_common.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

constexpr double SYNC_THRESHOLD =
    (SYNC_LEVEL + BLANKING_LEVEL) /
    2.0; // Define threshold between blanking and sync levels

constexpr int SYNC_DURATION_TOLERANCE = static_cast<int>(
    H_SYNC_SAMPLES * 0.1); // Allow ±10% tolerance for H-sync duration

/**
 * @brief Detect H-sync leading edges in a composite signal.
 *
 * Scans the input samples for falling edges below SYNC_THRESHOLD andmeasures
 * pulse width against H_SYNC_SAMPLES to identify valid sync pulses.
 *
 * @param signal Composite base-band NTSC samples.
 * @return Start indices of valid H-sync pulses.
 */
std::vector<size_t> detect_h_syncs(const SignalSamples& signal) {
  std::vector<size_t> sync_starts;
  bool in_sync = false;
  size_t potential_start = 0;

  for (size_t i = 0; i < signal.size(); ++i) {
    if (!in_sync && signal[i] < SYNC_THRESHOLD) {
      in_sync = true; // Enter sync pulse
      potential_start = i;
    } else if (in_sync && signal[i] >= SYNC_THRESHOLD) {
      in_sync = false; // Exit sync pulse
      size_t duration = i - potential_start;
      if (std::abs(static_cast<int>(duration) - H_SYNC_SAMPLES) <=
          SYNC_DURATION_TOLERANCE) {
        sync_starts.push_back(potential_start); // Record sync start
      }
    }
  }

  return sync_starts;
}

/**
 * @brief Decode a single field from NTSC-M composite signal.
 *
 * Uses sync detection to skip VBI lines, extracts burst phase, separates luma
 * and chroma, performs quadrature demodulation, and converts YIQ samples to RGB
 * pixels.
 *
 * @param input_signal Field sampels including VBI.
 * @param is_odd_field True for odd field, false for even field.
 * @return VideoFrame containing decoded pixels and field parity.
 */
VideoFrame decode_field(const SignalSamples& input_signal, bool is_odd_field) {
  VideoFrame output_frame;
  output_frame.is_field_odd = is_odd_field;

  auto sync_starts = detect_h_syncs(input_signal);
  size_t first_active_idx = static_cast<size_t>(REMAINING_VBI_LINES);

  if (first_active_idx >= sync_starts.size()) {
    return output_frame; // No active lines detected
  }

  NTSCTimings timings;
  size_t row = 0;

  for (size_t l = first_active_idx;
       l < first_active_idx + static_cast<int>(output_frame.height) &&
       l < sync_starts.size();
       ++l) {
    size_t sync_start = sync_starts[l];
    size_t line_start = sync_start > static_cast<size_t>(FRONT_PORCH_SAMPLES)
                            ? sync_start - FRONT_PORCH_SAMPLES
                            : 0;
    size_t extract_len = std::min(static_cast<size_t>(SAMPLES_PER_LINE),
                                  input_signal.size() - line_start);
    SignalSamples line(input_signal.begin() + line_start,
                       input_signal.begin() + line_start + extract_len);

    if (static_cast<int>(extract_len) < SAMPLES_PER_LINE) {
      line.resize(SAMPLES_PER_LINE, BLANKING_LEVEL); // Pad short lines
    }

    SignalSamples burst_samples(line.begin() + timings.color_burst_start,
                                line.begin() + timings.color_burst_start +
                                    COLOR_BURST_SAMPLES);
    double burst_phase = detect_burst_phase(burst_samples);
    double demod_phase = burst_phase + M_PI / 2.0;

    SignalSamples active_samples(line.begin() + timings.active_video_start,
                                 line.begin() + timings.active_video_start +
                                     timings.active_video_samples);

    SignalSamples y = active_samples;
    apply_low_pass_filter(y, 4.2e6); // Filter Y to 4.2 MHz bandwidth

    SignalSamples c = active_samples;
    apply_band_pass_filter(c, SUBCARRIER_FREQ,
                           1.3e6); // Filter C to 1.3 MHz bandwidth

    SignalSamples i_mod(active_samples.size());
    SignalSamples q_mod(active_samples.size());
    double t = 0.0;
    double dt = 1.0 / SAMPLING_RATE;
    double omega = 2.0 * M_PI * SUBCARRIER_FREQ;

    for (size_t s = 0; s < active_samples.size(); ++s) {
      double ref_cos = std::cos(omega * t + demod_phase);
      double ref_sin = std::sin(omega * t + demod_phase);
      i_mod[s] = 2.0 * c[s] * ref_cos; // Demodulate I component
      q_mod[s] = 2.0 * c[s] * ref_sin; // Demodulate Q component
      t += dt;
    }

    apply_low_pass_filter(i_mod, 1.3e6); // Filter I to 1.3 MHz
    apply_low_pass_filter(q_mod, 0.5e6); // Filter Q to 0.5 MHz

    std::vector<double> y_pix(output_frame.width);
    std::vector<double> i_pix(output_frame.width);
    std::vector<double> q_pix(output_frame.width);
    downsample_average(y, y_pix, output_frame.width);
    downsample_average(i_mod, i_pix, output_frame.width);
    downsample_average(q_mod, q_pix, output_frame.width);

    double y_scale = WHITE_LEVEL - BLACK_LEVEL;
    for (int p = 0; p < output_frame.width; ++p) {
      double norm_y = (y_pix[p] - BLACK_LEVEL) / y_scale; // Normalize Y
      std::array<double, 3> yiq = {norm_y, i_pix[p], q_pix[p]};
      std::array<double, 3> rgb = yiq_to_rgb(yiq[0], yiq[1], yiq[2]);
      output_frame.pixels[row * output_frame.width + p] = rgb;
    }
    ++row;
  }

  // TODO: remove debugging log later
  if (row < static_cast<size_t>(output_frame.height)) {
    std::cerr << "Warning: Decoded only " << row << " rows" << std::endl;
  }

  return output_frame;
}
