/*
 * ntsc_encoder.cpp
 *
 * Generates a base-band NTSC-M composite signal at 4×Fsc (≈14.318 MHz) from a
 * VideoFrame.  The encoder is timing-accurate down to individual samples so it
 * can be fed directly into a DAC / SDR or looped back into the software
 * decoder.  The implementation follows the 1953 NTSC signal specification
 * (FCC approval December 17 1953) and SMPTE-170M tolerances.
 */

#include "dsp_utils.h"
#include "ntsc_common.h"
#include <array>
#include <cctype>
#include <cmath>
#include <sys/wait.h>
#include <vector>

// Additional constants for VBI
constexpr double EQ_PULSE_US = 2.3; // Equalizing pulses
constexpr double EQ_INTERVAL_US = (LINE_DURATION_US / 2.0) - EQ_PULSE_US;
constexpr double BROAD_PULSE_US = (LINE_DURATION_US / 2.0) - H_SYNC_US;
constexpr double SERRATION_US = H_SYNC_US;

// Number of discrete samples for each signal portion
constexpr int EQ_PULSE_SAMPLES =
    static_cast<int>(EQ_PULSE_US * SAMPLING_RATE / 1e6 + 0.5);
constexpr int EQ_INTERVAL_SAMPLES =
    static_cast<int>(EQ_INTERVAL_US * SAMPLING_RATE / 1e6 + 0.5);
constexpr int BROAD_PULSE_SAMPLES =
    static_cast<int>(BROAD_PULSE_US * SAMPLING_RATE / 1e6 + 0.5);
constexpr int SERRATION_SAMPLES =
    static_cast<int>(SERRATION_US * SAMPLING_RATE / 1e6 + 0.5);

// Number of remaining VBI lines after the 9-line sync block (to approximate
// 20-21 lines total VBI)
constexpr int REMAINING_VBI_LINES = 12;

// Generate half-line blank signal
void generate_half_blank(SignalSamples& samples, double& subcarrier_phase) {
  double dphase = 2.0 * M_PI * SUBCARRIER_FREQ / SAMPLING_RATE;
  int half_samples = SAMPLES_PER_LINE / 2; // assume even for simplicity
  for (int i = 0; i < half_samples; ++i) {
    samples.push_back(BLANKING_LEVEL);
    subcarrier_phase += dphase;
  }
}

// Generate the vertical sync block (9 lines worth of pulses, no burst)
void generate_v_sync_block(SignalSamples& samples, double& subcarrier_phase) {
  double dphase = 2.0 * M_PI * SUBCARRIER_FREQ / SAMPLING_RATE;

  // Pre-equalizing: 6 intervals (high) + 6 equalizing pulses
  for (int p = 0; p < 6; ++p) {
    for (int i = 0; i < EQ_INTERVAL_SAMPLES; ++i) {
      samples.push_back(BLANKING_LEVEL); // Interval (high)
      subcarrier_phase += dphase;
    }
    for (int i = 0; i < EQ_PULSE_SAMPLES; ++i) {
      samples.push_back(SYNC_LEVEL); // Eq pulse (low)
      subcarrier_phase += dphase;
    }
  }

  // Vertical sync: 6 serrations (high) + 6 broad pulses (low)
  for (int p = 0; p < 6; ++p) {
    for (int i = 0; i < SERRATION_SAMPLES; ++i) {
      samples.push_back(BLANKING_LEVEL); // Serration (high)
      subcarrier_phase += dphase;
    }
    for (int i = 0; i < BROAD_PULSE_SAMPLES; ++i) {
      samples.push_back(SYNC_LEVEL); // Broad (low)
      subcarrier_phase += dphase;
    }
  }

  // Post-equalizing: 6 intervals (high) + 6 equalizing pulses
  for (int p = 0; p < 6; ++p) {
    for (int i = 0; i < EQ_INTERVAL_SAMPLES; ++i) {
      samples.push_back(BLANKING_LEVEL); // Interval (high)
      subcarrier_phase += dphase;
    }
    for (int i = 0; i < EQ_PULSE_SAMPLES; ++i) {
      samples.push_back(SYNC_LEVEL); // Eq pulse (low)
      subcarrier_phase += dphase;
    }
  }
}

// Generalized line generation function
void generate_line(const std::vector<std::array<double, 3>>& yiq_line,
                   SignalSamples& line_samples, double& subcarrier_phase,
                   bool is_blank = false, bool with_burst = true) {
  line_samples.resize(SAMPLES_PER_LINE);
  double dphase = 2.0 * M_PI * SUBCARRIER_FREQ / SAMPLING_RATE;
  double current_phase = subcarrier_phase;

  int idx = 0;

  // Front porch
  for (int i = 0; i < FRONT_PORCH_SAMPLES; ++i) {
    line_samples[idx++] = BLANKING_LEVEL;
    current_phase += dphase;
  }

  // H-sync
  for (int i = 0; i < H_SYNC_SAMPLES; ++i) {
    line_samples[idx++] = SYNC_LEVEL;
    current_phase += dphase;
  }

  // Breezeway
  for (int i = 0; i < BREEZEWAY_SAMPLES; ++i) {
    line_samples[idx++] = BLANKING_LEVEL;
    current_phase += dphase;
  }

  // Color burst if it is present
  if (with_burst) {
    for (int i = 0; i < COLOR_BURST_SAMPLES; ++i) {
      line_samples[idx++] =
          BLANKING_LEVEL + BURST_AMPLITUDE * std::sin(current_phase + M_PI);
      current_phase += dphase;
    }
  } else {
    for (int i = 0; i < COLOR_BURST_SAMPLES; ++i) {
      line_samples[idx++] = BLANKING_LEVEL;
      current_phase += dphase;
    }
  }

  // Back porch
  for (int i = 0; i < BACK_PORCH_SAMPLES; ++i) {
    line_samples[idx++] = BLANKING_LEVEL;
    current_phase += dphase;
  }

  // Active video
  double base_level = is_blank ? BLANKING_LEVEL : BLACK_LEVEL;
  double y_scale = WHITE_LEVEL - BLACK_LEVEL;
  if (is_blank) {
    for (int i = 0; i < ACTIVE_VIDEO_SAMPLES; ++i) {
      line_samples[idx++] = base_level;
      current_phase += dphase;
    }
  } else {
    std::vector<double> y_low(yiq_line.size());
    std::vector<double> i_low(yiq_line.size());
    std::vector<double> q_low(yiq_line.size());
    for (size_t p = 0; p < yiq_line.size(); ++p) {
      y_low[p] = yiq_line[p][0];
      i_low[p] = yiq_line[p][1];
      q_low[p] = yiq_line[p][2];
    }

    SignalSamples y_high(ACTIVE_VIDEO_SAMPLES);
    SignalSamples i_high(ACTIVE_VIDEO_SAMPLES);
    SignalSamples q_high(ACTIVE_VIDEO_SAMPLES);
    upsample_linear(y_low, y_high, ACTIVE_VIDEO_SAMPLES);
    upsample_linear(i_low, i_high, ACTIVE_VIDEO_SAMPLES);
    upsample_linear(q_low, q_high, ACTIVE_VIDEO_SAMPLES);

    for (int i = 0; i < ACTIVE_VIDEO_SAMPLES; ++i) {
      double luma = y_high[i];       // luminance signal
      double in_phase = i_high[i];   // in-phase component of chrominance
      double quadrature = q_high[i]; // quadrature component of chrominance
      double chroma =
          in_phase * std::cos(current_phase) +
          quadrature * std::sin(current_phase); // chrominance signal
      line_samples[idx++] = base_level + luma * y_scale + chroma;
      current_phase += dphase;
    }
  }

  subcarrier_phase = current_phase;
}

// Encode a single field with full VBI
SignalSamples encode_field(const VideoFrame& input_frame) {
  VideoFrame yiq_frame = input_frame;
  yiq_frame.to_yiq();

  SignalSamples field_signal;
  double subcarrier_phase = 0.0; // assume start at 0; chain if multiple fields

  bool is_odd_field = input_frame.is_field_odd;

  // For even field, add half-line blank to shift sync block
  if (!is_odd_field) {
    generate_half_blank(field_signal, subcarrier_phase);
  }

  // Generate V sync block
  SignalSamples vsync_samples;
  generate_v_sync_block(vsync_samples, subcarrier_phase);
  field_signal.insert(field_signal.begin(), vsync_samples.begin(),
                      vsync_samples.end());

  // Remaining VBI blank lines with burst
  for (int l = 0; l < REMAINING_VBI_LINES; ++l) {
    SignalSamples line;
    generate_line({}, line, subcarrier_phase, true, true); // blank, with burst
    field_signal.insert(field_signal.end(), line.begin(), line.end());
  }

  // Active video lines
  size_t pixel_row_idx = 0;
  for (int l = 0; l < input_frame.height; ++l) {
    std::vector<std::array<double, 3>> line_pixels(input_frame.width);
    for (int p = 0; p < input_frame.width; ++p) {
      line_pixels[p] = yiq_frame.pixels[pixel_row_idx * input_frame.width + p];
    }

    SignalSamples line;
    generate_line(line_pixels, line, subcarrier_phase, false, true);
    field_signal.insert(field_signal.end(), line.begin(), line.end());

    pixel_row_idx++;
  }

  // Add extra blank active lines if needed to approach 262 lines
  int extra_active_blank = 242 - input_frame.height; // eg. 242 - 240 = 2
  for (int l = 0; l < extra_active_blank; ++l) {
    SignalSamples line;
    generate_line({}, line, subcarrier_phase, true, true); // blank active line
    field_signal.insert(field_signal.end(), line.begin(), line.end());
  }

  //  For odd field, add half-line blank at end
  if (is_odd_field) {
    generate_half_blank(field_signal, subcarrier_phase);
  }

  return field_signal;
}
