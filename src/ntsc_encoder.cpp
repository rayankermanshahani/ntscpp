/**
 * @file ntsc_encoder.cpp
 * @brief Encode video frames into NTSC-M composite signal at 4 x Fsc.
 *
 * Generates a timing‑accurate composite signal suitable for DAC/SDR or
 * loopback testing per FCC 1953 and SMPTE 170 M specification.
 */

#include "../include/dsp_utils.h"
#include "../include/ntsc_common.h"
#include <array>
#include <cmath>
#include <vector>

/**
 * @brief Generate half-line blanking interval samples.
 *
 * Emits BLANKING_LEVEL for half a line and advances subcarrier phase.
 *
 * @param samples Container to append blank samples.
 * @param subcarrier_phase Subcarrier phase in radians; updated per sample.
 */
void generate_half_blank(SignalSamples& samples, double& subcarrier_phase) {
  double dphase = 2.0 * M_PI * SUBCARRIER_FREQ / SAMPLING_RATE;
  int half_samples =
      SAMPLES_PER_LINE / 2; // Assume even line length for simplicity
  for (int i = 0; i < half_samples; ++i) {
    samples.push_back(BLANKING_LEVEL);
    subcarrier_phase += dphase;
  }
}

/**
 * @brief Generate vertical sync block for NTSC-M.
 *
 * Emits pre-equalizing, broad sync, and post-equalizing pulses.
 *
 * @param samples Container to append sync block samples.
 * @param subcarrier_phase Subcarrier phase in radians; updated per sample.
 */
void generate_v_sync_block(SignalSamples& samples, double& subcarrier_phase) {
  double dphase = 2.0 * M_PI * SUBCARRIER_FREQ / SAMPLING_RATE;

  auto signal_push = [&](double level, int n_samples) {
    for (int i = 0; i < n_samples; ++i) {
      samples.push_back(level);
      subcarrier_phase += dphase;
    }
  };

  // Pre-equalizing pulses (6 half-lines)
  for (int k = 0; k < 6; ++k) {
    signal_push(SYNC_LEVEL, EQ_PULSE_SAMPLES);
    signal_push(BLANKING_LEVEL, EQ_INTERVAL_SAMPLES);
  }

  // Broad sync pulses (6 half-lines)
  for (int k = 0; k < 6; ++k) {
    signal_push(SYNC_LEVEL, BROAD_PULSE_SAMPLES);
    signal_push(BLANKING_LEVEL, SERRATION_SAMPLES);
  }

  // Post-equalizing pulses (6 half-lines)
  for (int k = 0; k < 6; ++k) {
    signal_push(SYNC_LEVEL, EQ_PULSE_SAMPLES);
    signal_push(BLANKING_LEVEL, EQ_INTERVAL_SAMPLES);
  }
}

/**
 * @brief Generate one horizontal line of composite signal.
 *
 * Assembles front porch, H-sync, optional color burst, back porch, and active
 * video or blank line.
 *
 * @param yiq_line Pixel YIQ values or empty for a blank line.
 * @param line_samples Output container resized to SAMPLES_PER_LINE.
 * @param subcarrier_phase Subcarrier phase in radians; updated per sample.
 * @param is_blank True to emit blank active video.
 * @param with_burst True to include color burst segment.
 */
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

  // Color burst or idle
  if (with_burst) {
    for (int i = 0; i < COLOR_BURST_SAMPLES; ++i) {
      line_samples[idx++] =
          BLANKING_LEVEL + BURST_AMPLITUDE * std::cos(current_phase + M_PI);
      // line_samples[idx++] = BLANKING_LEVEL + BURST_AMPLITUDE *
      // std::sin(current_phase + M_PI);
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

  // Active video or blank line
  double base_level = is_blank ? BLANKING_LEVEL : BLACK_LEVEL;
  double scale_factor = WHITE_LEVEL - BLACK_LEVEL;
  if (is_blank) {
    for (int i = 0; i < ACTIVE_VIDEO_SAMPLES; ++i) {
      line_samples[idx++] = base_level;
      current_phase += dphase;
    }
  } else {
    // Upsample Y, I, Q channels to sample rate
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
      double luma = y_high[i];
      double in_phase = i_high[i];
      double quadrature = q_high[i];
      double chroma = in_phase * std::cos(current_phase) +
                      quadrature * std::sin(current_phase);
      line_samples[idx++] = scale_factor * luma + chroma + base_level;
      current_phase += dphase;
    }
  }

  subcarrier_phase = current_phase;
}

/**
 * @brief Encode a VideoFrame into NTSC‑M composite field signal.
 *
 * Converts frame pixels to YIQ, emits VBI (vertical sync + blank lines),
 * encodes active lines, and appends half‑line as needed.
 *
 * @param input_frame VideoFrame with pixel data and field parity.
 * @return Composite samples for one field.
 */
SignalSamples encode_field(const VideoFrame& input_frame) {
  VideoFrame yiq_frame = input_frame;
  yiq_frame.to_yiq();

  SignalSamples field_signal;
  double subcarrier_phase = 0.0;
  bool is_odd_field = input_frame.is_field_odd;

  // Shift sync block for even field
  if (!is_odd_field) {
    generate_half_blank(field_signal, subcarrier_phase);
  }

  // Vertical sync block
  SignalSamples vsync_samples;
  generate_v_sync_block(vsync_samples, subcarrier_phase);
  field_signal.insert(field_signal.begin(), vsync_samples.begin(),
                      vsync_samples.end());

  // Remaining VBI lines with burst
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

  // Pad extra blank lines to reach active + blank total
  int extra_active_blank = ACTIVE_PLUS_BLANK_LINES - input_frame.height;
  for (int l = 0; l < extra_active_blank; ++l) {
    SignalSamples line;
    generate_line({}, line, subcarrier_phase, true, true);
    field_signal.insert(field_signal.end(), line.begin(), line.end());
  }

  // Append half-line blank for odd field
  if (is_odd_field) {
    generate_half_blank(field_signal, subcarrier_phase);
  }

  return field_signal;
}
