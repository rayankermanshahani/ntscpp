/**
 * @file ntsc_common.h
 * @brief Shared NTSC-M constants, types, and utility functions.
 *
 * Defines timing parameters, sample counts, signal levels, and color conversion
 * matrices per FCC 1953 and SMPTE 170 M NTSC-M specification.
 */

#ifndef NTSC_COMMON_H
#define NTSC_COMMON_H

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

/// Helper for compile-time rounding of double floating-piont to nearest integer
constexpr int const_round(double x) {
  return x >= 0 ? static_cast<int>(x + 0.5) : static_cast<int>(x - 0.5);
}

/* Frequencies */
constexpr double SUBCARRIER_FREQ = 3.15e8 / 88.0;           ///< ~3.579 MHz
constexpr double LINE_RATE = 2.0 * SUBCARRIER_FREQ / 455.0; ///< ~15.734 kHz
constexpr double FIELD_RATE = LINE_RATE * (2.0 / 525.0);    ///< ~59.94 Hz
constexpr double FRAME_RATE = FIELD_RATE / 2.0;             ///< ~29.97 Hz

/* Durations */
constexpr double LINE_DURATION_US = 1e6 / LINE_RATE;    ///< ~63.556 us
constexpr double SAMPLING_RATE = 4.0 * SUBCARRIER_FREQ; ///< ~14.318 MHz
constexpr int SAMPLES_PER_LINE =
    const_round(SAMPLING_RATE / LINE_RATE); ///< 910

constexpr double FRONT_PORCH_US = 1.5; ///< ~1.5 μs
constexpr double H_SYNC_US = 4.7;      ///< ~4.7 μs
constexpr double BREEZEWAY_US = 0.6;   ///< ~2.5 μs
constexpr double COLOR_BURST_US = 2.5; ///< ~2.5 μs
constexpr double BACK_PORCH_US = 1.6;  ///< ~1.6 μs
constexpr double ACTIVE_VIDEO_US =
    LINE_DURATION_US - (FRONT_PORCH_US + H_SYNC_US + BREEZEWAY_US +
                        COLOR_BURST_US + BACK_PORCH_US); ///< ~52.655 μs

constexpr int FRONT_PORCH_SAMPLES =
    const_round(FRONT_PORCH_US * SAMPLING_RATE / 1e6);
constexpr int H_SYNC_SAMPLES = const_round(H_SYNC_US * SAMPLING_RATE / 1e6);
constexpr int BREEZEWAY_SAMPLES =
    const_round(BREEZEWAY_US * SAMPLING_RATE / 1e6);
constexpr int COLOR_BURST_SAMPLES =
    const_round(COLOR_BURST_US * SAMPLING_RATE / 1e6);
constexpr int BACK_PORCH_SAMPLES =
    const_round(BACK_PORCH_US * SAMPLING_RATE / 1e6);
constexpr int ACTIVE_VIDEO_SAMPLES =
    const_round(ACTIVE_VIDEO_US * SAMPLING_RATE / 1e6);

/* VBI half-line timings */
constexpr double HALF_LINE_US = LINE_DURATION_US / 2.0;        ///< ~31.778 μs
constexpr double EQ_PULSE_US = 2.3;                            ///< ~2.3 μs
constexpr double EQ_INTERVAL_US = HALF_LINE_US - EQ_PULSE_US;  ///< ~29.478 μs
constexpr double SERRATION_US = H_SYNC_US;                     ///< ~4.7 μs
constexpr double BROAD_PULSE_US = HALF_LINE_US - SERRATION_US; ///< ~27.078 μs

constexpr int EQ_PULSE_SAMPLES = const_round(EQ_PULSE_US * SAMPLING_RATE / 1e6);
constexpr int EQ_INTERVAL_SAMPLES =
    const_round(EQ_INTERVAL_US * SAMPLING_RATE / 1e6);
constexpr int SERRATION_SAMPLES =
    const_round(SERRATION_US * SAMPLING_RATE / 1e6);
constexpr int BROAD_PULSE_SAMPLES =
    const_round(BROAD_PULSE_US * SAMPLING_RATE / 1e6);

/* Video dimensions */
constexpr int VISIBLE_WIDTH = 720;
constexpr int VISIBLE_HEIGHT = 480;
constexpr int FULL_LINES_PER_FIELD = 262;
constexpr int LINES_PER_FIELD = FULL_LINES_PER_FIELD;
constexpr int LINES_PER_FRAME = 525;
constexpr int VBI_FULL_LINES = 21;
constexpr int VBI_LINES_PER_FIELD = VBI_FULL_LINES;
constexpr int V_SYNC_EQUIV_LINES = 9; ///< 3 pre + 3 broad + 3 post full-line
constexpr int REMAINING_VBI_LINES = VBI_FULL_LINES - V_SYNC_EQUIV_LINES; ///< 12
constexpr int ACTIVE_PLUS_BLANK_LINES =
    FULL_LINES_PER_FIELD - VBI_FULL_LINES; ///< 241

/* Signal levels */
constexpr double IRE_TO_NORM =
    1.0 / 100.0; ///< scale from IRE (offset for sync)
constexpr double SYNC_LEVEL = -40.0 * IRE_TO_NORM;
constexpr double BLANKING_LEVEL = 0.0;
constexpr double BLACK_LEVEL = 7.5 * IRE_TO_NORM;
constexpr double WHITE_LEVEL = 100.0 * IRE_TO_NORM;
constexpr double BURST_AMPLITUDE = 20.0 * IRE_TO_NORM;

/// RGB to YIQ color matrix per NTSC-M
constexpr std::array<std::array<double, 3>, 3> RGB_TO_YIQ_MATRIX = {
    {{0.299, 0.587, 0.114},       ///< Y
     {0.5959, -0.2746, -0.3213},  ///< I
     {0.2115, -0.5227, 0.3112}}}; ///< Q

/// YIQ to RGB color matrix per NTSC-M
constexpr std::array<std::array<double, 3>, 3> YIQ_TO_RGB_MATRIX = {
    {{1.0, 0.956, 0.619},    ///< R
     {1.0, -0.272, -0.647},  ///< G
     {1.0, -1.106, 1.703}}}; ///< B

/** Sequence of normalize composite samples */
using SignalSamples = std::vector<double>;

/**
 * @struct NTSCTimings
 * @brief Computed sample offsets for each line segment.
 */
struct NTSCTimings {
  int front_porch_start = 0;
  int h_sync_start = FRONT_PORCH_SAMPLES;
  int breezeway_start = h_sync_start + H_SYNC_SAMPLES;
  int color_burst_start = breezeway_start + BREEZEWAY_SAMPLES;
  int back_porch_start = color_burst_start + COLOR_BURST_SAMPLES;
  int active_video_start = back_porch_start + BACK_PORCH_SAMPLES;
  int active_video_samples = ACTIVE_VIDEO_SAMPLES;
};

/**
 * @struct VideoFrame
 * @brief Container for one NTSC-M field of pixel data.
 */
struct VideoFrame {
  int width = VISIBLE_WIDTH;                 ///< pixels per line
  int height = VISIBLE_HEIGHT / 2;           ///< lines per field
  bool is_field_odd = true;                  ///< field parity
  std::vector<std::array<double, 3>> pixels; ///< row-major RGB or YIQ

  /** Initialize pixel buffer to width*height. */
  VideoFrame() { pixels.resize(height * width); }

  /** Convert stored pixels from RGB to YIQ in-place */
  void to_yiq() {
    for (auto& pixel : pixels) {
      double r = pixel[0], g = pixel[1], b = pixel[2];
      pixel[0] = RGB_TO_YIQ_MATRIX[0][0] * r + RGB_TO_YIQ_MATRIX[0][1] * g +
                 RGB_TO_YIQ_MATRIX[0][2] * b;
      pixel[1] = RGB_TO_YIQ_MATRIX[1][0] * r + RGB_TO_YIQ_MATRIX[1][1] * g +
                 RGB_TO_YIQ_MATRIX[1][2] * b;
      pixel[2] = RGB_TO_YIQ_MATRIX[2][0] * r + RGB_TO_YIQ_MATRIX[2][1] * g +
                 RGB_TO_YIQ_MATRIX[2][2] * b;
    }
  }

  /** Convert stored pixels from YIQ to RGB in-place */
  void to_rgb() {
    for (auto& pixel : pixels) {
      double y = pixel[0], i = pixel[1], q = pixel[2];
      pixel[0] = YIQ_TO_RGB_MATRIX[0][0] * y + YIQ_TO_RGB_MATRIX[0][1] * i +
                 YIQ_TO_RGB_MATRIX[0][2] * q;
      pixel[1] = YIQ_TO_RGB_MATRIX[1][0] * y + YIQ_TO_RGB_MATRIX[1][1] * i +
                 YIQ_TO_RGB_MATRIX[1][2] * q;
      pixel[2] = YIQ_TO_RGB_MATRIX[2][0] * y + YIQ_TO_RGB_MATRIX[2][1] * i +
                 YIQ_TO_RGB_MATRIX[2][2] * q;
    }
  }
};

/**
 * @brief Compute subcarrier sample at time t with optional phase offset.
 * @param t Time in seconds.
 * @param phase_offset Phase offset in radians.
 * @return sin(2πFsc·t + phase_offset).
 */
inline double subcarrier(double t, double phase_offset = 0.0) {
  return std::sin(2.0 * M_PI * SUBCARRIER_FREQ * t + phase_offset);
}

/**
 * @brief Compute NTSC color burst at time t.
 * @param t Time in seconds.
 * @return BURST_AMPLITUDE·sin(2πFsc·t + π).
 */
inline double color_burst(double t) {
  return BURST_AMPLITUDE * subcarrier(t, M_PI);
}

/**
 * @brief Convert one RGB sample to YIQ.
 * @param r Red component.
 * @param g Green component.
 * @param b Blue component.
 * @return {Y, I, Q} values.
 */
inline std::array<double, 3> rgb_to_yiq(double r, double g, double b) {
  return {RGB_TO_YIQ_MATRIX[0][0] * r + RGB_TO_YIQ_MATRIX[0][1] * g +
              RGB_TO_YIQ_MATRIX[0][2] * b,
          RGB_TO_YIQ_MATRIX[1][0] * r + RGB_TO_YIQ_MATRIX[1][1] * g +
              RGB_TO_YIQ_MATRIX[1][2] * b,
          RGB_TO_YIQ_MATRIX[2][0] * r + RGB_TO_YIQ_MATRIX[2][1] * g +
              RGB_TO_YIQ_MATRIX[2][2] * b};
}

/**
 * @brief Convert one YIQ sample to RGB.
 * @param y Luma component.
 * @param i In‑phase chroma.
 * @param q Quadrature chroma.
 * @return {R, G, B} values.
 */
inline std::array<double, 3> yiq_to_rgb(double y, double i, double q) {
  return {YIQ_TO_RGB_MATRIX[0][0] * y + YIQ_TO_RGB_MATRIX[0][1] * i +
              YIQ_TO_RGB_MATRIX[0][2] * q,
          YIQ_TO_RGB_MATRIX[1][0] * y + YIQ_TO_RGB_MATRIX[1][1] * i +
              YIQ_TO_RGB_MATRIX[1][2] * q,
          YIQ_TO_RGB_MATRIX[2][0] * y + YIQ_TO_RGB_MATRIX[2][1] * i +
              YIQ_TO_RGB_MATRIX[2][2] * q};
}

#endif // NTSC_COMMON_H
