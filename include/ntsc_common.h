/**
 * @file ntsc_common.h
 * @brief Shared NTSC-M constants, types, and utility functions.
 *
 * Defines timing parameters, sample counts, signal levels, and color conversion
 * matrices as per the SMPTE 170M-2004 NTSC-M specification.
 */

#ifndef NTSC_COMMON_H
#define NTSC_COMMON_H

#include <array>
#include <cmath>
#include <vector>

/// Helper for compile-time rounding of double floating-piont to nearest integer
constexpr int const_round(double x) {
  return x >= 0 ? static_cast<int>(x + 0.5) : static_cast<int>(x - 0.5);
}

/* Core frequencies */
constexpr double FSC = 3.15e8 / 88.0;           ///< ~3.579 MHz
constexpr double LINE_RATE = 2.0 * FSC / 455.0; ///< ~15.734 kHz
constexpr double SR = 4.0 * FSC;                ///< Sampling rate: ~14.318 MHz

/* Line timing */
constexpr double LINE_DURATION_US = 1e6 / LINE_RATE;          ///< ~63.556 us
constexpr int SAMPLES_PER_LINE = const_round(SR / LINE_RATE); ///< 910

/* Horizontal blanking intervals */
constexpr double FRONT_PORCH_US = 1.5;     ///< ~1.5 μs
constexpr double H_SYNC_US = 4.7;          ///< ~4.7 μs
constexpr double BREEZEWAY_US = 0.6;       ///< ~2.5 μs
constexpr double COLOR_BURST_US = 2.5;     ///< ~2.5 μs
constexpr double BACK_PORCH_US = 1.6;      ///< ~1.6 μs
constexpr double ACTIVE_VIDEO_US = 52.655; ///< ~52.655 μs

constexpr int FRONT_PORCH_SAMPLES = const_round(FRONT_PORCH_US * SR / 1e6);
constexpr int H_SYNC_SAMPLES = const_round(H_SYNC_US * SR / 1e6);
constexpr int BREEZEWAY_SAMPLES = const_round(BREEZEWAY_US * SR / 1e6);
constexpr int COLOR_BURST_SAMPLES = const_round(COLOR_BURST_US * SR / 1e6);
constexpr int BACK_PORCH_SAMPLES = const_round(BACK_PORCH_US * SR / 1e6);
constexpr int ACTIVE_VIDEO_SAMPLES = const_round(ACTIVE_VIDEO_US * SR / 1e6);

/* Vertical sync timing */
constexpr double HALF_LINE_US = LINE_DURATION_US / 2.0;        ///< ~31.778 μs
constexpr double EQ_PULSE_US = 2.3;                            ///< ~2.3 μs
constexpr double EQ_INTERVAL_US = HALF_LINE_US - EQ_PULSE_US;  ///< ~29.478 μs
constexpr double SERRATION_US = H_SYNC_US;                     ///< ~4.7 μs
constexpr double BROAD_PULSE_US = HALF_LINE_US - SERRATION_US; ///< ~27.078 μs

constexpr int EQ_PULSE_SAMPLES = const_round(EQ_PULSE_US * SR / 1e6);
constexpr int EQ_INTERVAL_SAMPLES = const_round(EQ_INTERVAL_US * SR / 1e6);
constexpr int SERRATION_SAMPLES = const_round(SERRATION_US * SR / 1e6);
constexpr int BROAD_PULSE_SAMPLES = const_round(BROAD_PULSE_US * SR / 1e6);

/* Video frame structure */
constexpr int VISIBLE_WIDTH = 720;   ///< 720 pixels
constexpr int VISIBLE_HEIGHT = 480;  ///< 480 pixels
constexpr int LINES_PER_FIELD = 262; ///< 525 lines per frame / 2 (rounded down)
constexpr int VBI_LINES = 21;        ///< 21 lines
constexpr int REMAINING_VBI_LINES = 12; ///< 21 total VBI lines - 9 sync lines
constexpr int ACTIVE_PLUS_BLANK_LINES =
    LINES_PER_FIELD - VBI_LINES; ///< 241 lines

/* Signal levels (normalized) */
constexpr double SYNC_LEVEL = -0.40;     ///< -40 IRE
constexpr double BLANKING_LEVEL = 0.0;   ///< 0 IRE
constexpr double BLACK_LEVEL = 0.075;    ///< 7.5 IRE
constexpr double WHITE_LEVEL = 1.0;      ///< 100 IRE
constexpr double BURST_AMPLITUDE = 0.20; ///< 20 IRE

/* Color matrices per SMTPE 170M-2004 */
constexpr std::array<std::array<double, 3>, 3> RGB_TO_YIQ_MATRIX = {
    {{0.299, 0.587, 0.114},       // Y
     {0.5959, -0.2746, -0.3213},  // I
     {0.2115, -0.5227, 0.3112}}}; // Q

constexpr std::array<std::array<double, 3>, 3> YIQ_TO_RGB_MATRIX = {
    {{1.0, 0.956, 0.619},    // R
     {1.0, -0.272, -0.647},  // G
     {1.0, -1.106, 1.703}}}; // B

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
  int width = VISIBLE_WIDTH;                 // Pixels per line
  int height = VISIBLE_HEIGHT / 2;           // Lines per field
  bool is_field_odd = true;                  // Field parity
  std::vector<std::array<double, 3>> pixels; // Row-major RGB or YIQ

  VideoFrame() { pixels.resize(height * width); }

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
