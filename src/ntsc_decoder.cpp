/*
 * ntsc_decoder.cpp
 *
 * Decodes a base-band NTSC-M composite signal (at 4xFsc sampling) back to a
 * Video Frame. Sync detection finds line starts, VBI is skipped (fixed count),
 * then per active line: extract burst phase, separate Y/C via filters,
 * quadrature demod I/Q (phase-locked), downsample to pixels, YIQ -> RGB.
 *
 * Matches encoder's timing/levels for loopback testing.
 */

#include "dsp_utils.h"
#include "ntsc_common.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <vector>

// Sync detection threshold (midway between BLANKING_LEVEL and SYNC_LEVEL
constexpr double SYNC_THRESHOLD = (SYNC_LEVEL + BLANKING_LEVEL) / 2.0; // ~0.2

// Tolerance for sync duration (samples: +- 10%)
constexpr int SYNC_DURATION_TOLERANCE = static_cast<int>(H_SYNC_SAMPLES * 0.1);

// Detect H-sync leading edges (returns vector of sample indices where sync
// starts)
std::vector<size_t> detect_h_syncs(const SignalSamples& signal) {
  std::vector<size_t> sync_starts;
  bool in_sync = false;
  size_t potential_start = 0;

  for (size_t i = 0; i < signal.size(); ++i) {
    if (!in_sync && signal[i] < SYNC_THRESHOLD) {
      in_sync = true;
      potential_start = i;
    } else if (in_sync && signal[i] >= SYNC_THRESHOLD) {
      in_sync = false;
      size_t duration = i - potential_start;
      if (std::abs(static_cast<int>(duration) - H_SYNC_SAMPLES) <=
          SYNC_DURATION_TOLERANCE) {
        sync_starts.push_back(potential_start);
      }
    }
  }

  return sync_starts;
}

// Decode a single field (assumes input_signal starts with VBI)
VideoFrame decode_field(const SignalSamples& input_signal, bool is_odd_field) {
  VideoFrame output_frame;
  output_frame.is_field_odd = is_odd_field;

  // Detect all H-sync starts (one per line, including VBI)
  auto sync_starts = detect_h_syncs(input_signal);
  if (sync_starts.size() < static_cast<size_t>(LINES_PER_FIELD)) {
    // Error: insuffucient lines detected (handle in main)
    return output_frame;
  }

  // Assume first VBI_LINES_PER_FIELD (~21) are VBI;
  // Next 240 are active video lines
  size_t first_active_idx = VBI_LINES_PER_FIELD;

  if (first_active_idx >= sync_starts.size()) {
    return output_frame; // No active lines
  }

  NTSCTimings timings; // Use NTSC-M standard offsets

  size_t row = 0;
  for (size_t l = first_active_idx;
       l < first_active_idx + static_cast<int>(output_frame.height) &&
       l < sync_starts.size();
       ++l) {
    size_t sync_start = sync_starts[l];

    // Line start is front porch (before sync)
    size_t line_start = sync_start > static_cast<size_t>(FRONT_PORCH_SAMPLES)
                            ? sync_start - FRONT_PORCH_SAMPLES
                            : 0;

    // Extract line (clamp if near end)
    size_t extract_len = std::min(static_cast<size_t>(FRONT_PORCH_SAMPLES),
                                  input_signal.size() - line_start);
    SignalSamples line(input_signal.begin() + line_start,
                       input_signal.begin() + line_start + extract_len);

    if (static_cast<int>(extract_len) < SAMPLES_PER_LINE) {
      line.resize(SAMPLES_PER_LINE, BLANKING_LEVEL); // Pad if short
    }

    // Extract color burst
    SignalSamples burst_samples(line.begin() + timings.active_video_start,
                                line.begin() + timings.active_video_start +
                                    timings.active_video_samples);
    double burst_phase = detect_burst_phase(burst_samples);

    // Extract active video
    SignalSamples active_samples(line.begin() + timings.active_video_start,
                                 line.begin() + timings.active_video_start +
                                     timings.active_video_samples);

    // Separate luma (Y) and chroma (C) signals
    SignalSamples y = active_samples;
    apply_low_pass_filter(y, 4.2e6); // Y bandwidth ~4.2 MHz

    SignalSamples c = active_samples;
    apply_band_pass_filter(c, SUBCARRIER_FREQ, 1.3e6); // C combined ~1.3MHz

    // Demodulate chroma to I/Q (quadrature demod, factor 2 for amplitude
    // recovery)
    SignalSamples i_mod(active_samples.size());
    SignalSamples q_mod(active_samples.size());
    double t = 0.0;
    double dt = 1.0 / SAMPLING_RATE;
    double omega = 2.0 * M_PI * SUBCARRIER_FREQ;
    for (size_t s = 0; s < active_samples.size(); ++s) {
      double ref_cos = std::cos(omega * t + burst_phase);
      double ref_sin = std::sin(omega * t + burst_phase);
      i_mod[s] = 2.0 * c[s] * ref_cos;
      q_mod[s] = 2.0 * c[s] * ref_sin;
      t += dt;
    }

    // Low-pass I/Q
    apply_low_pass_filter(i_mod, 1.3e6); // I ~1.3 MHz
    apply_low_pass_filter(q_mod, 0.5e6); // Q ~0.5 MHz

    // Downsample to pixels (normalize Y by subtracting setup and scaling)
    std::vector<double> y_pix(output_frame.width);
    std::vector<double> i_pix(output_frame.width);
    std::vector<double> q_pix(output_frame.width);
    downsample_average(y, y_pix, output_frame.width);
    downsample_average(i_mod, i_pix, output_frame.width);
    downsample_average(q_mod, q_pix, output_frame.width);

    double y_scale = WHITE_LEVEL - BLACK_LEVEL;
    for (int p = 0; p < output_frame.width; ++p) {
      double norm_y = (y_pix[p] - BLACK_LEVEL) / y_scale; // Y to [0.0, 1.0]
      std::array<double, 3> yiq = {norm_y, i_pix[p], q_pix[p]};
      std::array<double, 3> rgb = yiq_to_rgb(yiq[0], yiq[1], yiq[2]);
      output_frame.pixels[row * output_frame.width + p] = rgb;
    }
    ++row;
  }

  return output_frame;
}
