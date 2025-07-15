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
#include <cmath>
#include <cstddef>
#include <vector>

// Sync detection threshold (midway between BLANKING_LEVEL and SYNC_LEVEL
constexpr double SYNC_THRESHOLD = (SYNC_LEVEL + BLANKING_LEVEL) / 2.0; // ~0.2

// Tolerance for sync duration (samples: +- 10%)
constexpr int SYNC_DURATION_TOLERANCE = static_cast<int>(H_SYNC_SAMPLES * 0.1);

// Detect H-sync leading edges (returns vector of sample indices where sync
// starts)
std::vector<size_t> detect_h_syncs(const SignalSamples& signal);

// Decode a single field (assumes input_signal starts with VBI)
VideoFrame decode_field(const SignalSamples& input_signal, bool is_odd_field);
