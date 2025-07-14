#ifndef NTSC_COMMON_H
#define NTSC_COMMON_H

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

// NTSC constants (based on NTSC-M specs)
constexpr double SUBCARRIER_FREQ = 3.579545e6;  // Hz (exact: 315/88 MHz)
constexpr double FIELD_RATE = 59.94005994;      // Fields/sec
constexpr double FRAME_RATE = FIELD_RATE / 2.0; // ~29.97 fps
constexpr double LINE_RATE = 15734.265734; // Hz (approx 525 * FIELD_RATE / 2)
constexpr double LINE_DURATION_US = 1e6 / LINE_RATE;    // ~63.556 us
constexpr double SAMPLING_RATE = 4.0 * SUBCARRIER_FREQ; // ~14.31818 MHz
constexpr int SAMPLES_PER_LINE =
    static_cast<int>(SAMPLING_RATE / LINE_RATE + 0.5); // ~910 samples/line

// Precomputed timing offsets in samples
constexpr double FRONT_PORCH_US = 1.5;
constexpr double H_SYNC_US = 4.7;
constexpr double BREEZEWAY_US = 0.6;
constexpr double COLOR_BURST_US = 2.5;
constexpr double BACK_PORCH_US = 1.6;
constexpr double ACTIVE_VIDEO_US =
    LINE_DURATION_US - (FRONT_PORCH_US + H_SYNC_US + BREEZEWAY_US +
                        COLOR_BURST_US + BACK_PORCH_US); // ~52.655 us

constexpr int FRONT_PORCH_SAMPLES =
    static_cast<int>(FRONT_PORCH_US * SAMPLING_RATE / 1e6 + 0.5);
constexpr int H_SYNC_SAMPLES =
    static_cast<int>(H_SYNC_US * SAMPLING_RATE / 1e6 + 0.5);
constexpr int BREEZEWAY_SAMPLES =
    static_cast<int>(BREEZEWAY_US * SAMPLING_RATE / 1e6 + 0.5);
constexpr int COLOR_BURST_SAMPLES =
    static_cast<int>(COLOR_BURST_US * SAMPLING_RATE / 1e6 + 0.5);
constexpr int BACK_PORCH_SAMPLES =
    static_cast<int>(BACK_PORCH_US * SAMPLING_RATE / 1e6 + 0.5);
constexpr int ACTIVE_VIDEO_SAMPLES =
    SAMPLES_PER_LINE -
    (FRONT_PORCH_SAMPLES + H_SYNC_SAMPLES + BREEZEWAY_SAMPLES +
     COLOR_BURST_SAMPLES + BACK_PORCH_SAMPLES);

// Video dimensions
constexpr int VISIBLE_WIDTH = 720;   // Pixels per line
constexpr int VISIBLE_HEIGHT = 480;  // Visible lines per frame
constexpr int LINES_PER_FIELD = 263; // Approx; Field 1: 262.5, but integerize
                                     // for sim (adjust for half-line)
constexpr int LINES_PER_FRAME = 525;
constexpr int VBI_LINES_PER_FIELD =
    LINES_PER_FIELD - (VISIBLE_HEIGHT / 2); // ~21

// Signal levels (normalized for full range: -40 to 100 IRE = 140 IRE units)
constexpr double IRE_TO_NORM = 1.0 / 100.0; // Scale from IRE (offset for sync)
constexpr double SYNC_LEVEL = -40.0 * IRE_TO_NORM;
constexpr double BLANKING_LEVEL = 0.0;
constexpr double BLACK_LEVEL = 7.5 * IRE_TO_NORM;
constexpr double WHITE_LEVEL = 100.0 * IRE_TO_NORM;
constexpr double BURST_AMPLITUDE = 20.0 * IRE_TO_NORM; // 20 IRE p-p

// Color matrices
constexpr std::array<std::array<double, 3>, 3> RGB_TO_YIQ_MATRIX = {
    {{0.299, 0.587, 0.114},       // Y
     {0.5959, -0.2746, -0.3213},  // I
     {0.2115, -0.5227, 0.3112}}}; // Q

constexpr std::array<std::array<double, 3>, 3> YIQ_TO_RGB_MATRIX = {
    {{1.0, 0.956, 0.619},    // R
     {1.0, -0.272, -0.647},  // G
     {1.0, -1.106, 1.703}}}; // B

// Data structures
using SignalSamples = std::vector<double>; // Sequence of normalized composite
                                           // samples in range [-0.4, 1.0]

struct NTSCTimings {
  int front_porch_start = 0;
  int h_sync_start = FRONT_PORCH_SAMPLES;
  int breezeway_start = h_sync_start + H_SYNC_SAMPLES;
  int color_burst_start = breezeway_start + BREEZEWAY_SAMPLES;
  int back_porch_start = color_burst_start + COLOR_BURST_SAMPLES;
  int active_video_start = back_porch_start + BACK_PORCH_SAMPLES;
  int active_video_samples = ACTIVE_VIDEO_SAMPLES;
};

struct VideoFrame {
  int width = VISIBLE_WIDTH;
  int height = VISIBLE_HEIGHT / 2; // Per field (240 lines for interlaced)

  bool is_field_odd = true; // True for Field 1 (odd lines)
  std::vector<std::array<double, 3>>
      pixels; // Row-major, height * width elements total, each [R/Y, G/I, B/Q]
  VideoFrame() { pixels.resize(height * width); }

  // Convert pixels from RGB to YIQ in place
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

  // Convert pixels from YIQ to RGB in place
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

// Core functions
//
// Generate subcarrier value at time t (seconds), with phase offset (radians)
inline double subcarrier(double t, double phase_offset = 0.0) {
  return std::sin(2.0 * M_PI * SUBCARRIER_FREQ * t + phase_offset);
}

// Color burst is at 180 degrees (pi radians) phase
inline double color_burst(double t) {
  return BURST_AMPLITUDE * subcarrier(t, M_PI);
}

// Matrix multiply for single pixel (if needed outside of the VideoFrame struct)
inline std::array<double, 3> rgb_to_yiq(double r, double g, double b) {
  return {RGB_TO_YIQ_MATRIX[0][0] * r + RGB_TO_YIQ_MATRIX[0][1] * g +
              RGB_TO_YIQ_MATRIX[0][2] * b,
          RGB_TO_YIQ_MATRIX[1][0] * r + RGB_TO_YIQ_MATRIX[1][1] * g +
              RGB_TO_YIQ_MATRIX[1][2] * b,
          RGB_TO_YIQ_MATRIX[2][0] * r + RGB_TO_YIQ_MATRIX[2][1] * g +
              RGB_TO_YIQ_MATRIX[2][2] * b};
}

inline std::array<double, 3> yiq_to_rgb(double y, double i, double q) {
  return {YIQ_TO_RGB_MATRIX[0][0] * y + YIQ_TO_RGB_MATRIX[0][1] * i +
              YIQ_TO_RGB_MATRIX[0][2] * q,
          YIQ_TO_RGB_MATRIX[1][0] * y + YIQ_TO_RGB_MATRIX[1][1] * i +
              YIQ_TO_RGB_MATRIX[1][2] * q,
          YIQ_TO_RGB_MATRIX[2][0] * y + YIQ_TO_RGB_MATRIX[2][1] * i +
              YIQ_TO_RGB_MATRIX[2][2] * q};
}

#endif // NTSC_COMMON_H
