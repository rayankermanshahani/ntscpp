#ifndef NTSC_CODEC_H
#define NTSC_CODEC_H

#include <string>

/**
 * @brief Encode MP4 video to raw NTSC composite signal (.ntsc).
 *
 * Reads frames from input MP4, splits into composite fields, encodes each
 * field, and writes concatenated raw double samples to output.
 *
 * @param input_path Path to input MP4 file.
 * @param output_path Path to output .ntsc file.
 */
void encode(const std::string& input_path, const std::string& output_path);

/**
 * @brief Decode raw NTSC composite signal (.ntsc) to MP4 video.
 *
 * Reads raw doubles, splits into fields based on expected size, decodes each
 * field, interleaves into frames, and encodes output to MP4.
 *
 * @param input_path Path to input .ntsc file.
 * @param output_path Path to output MP4 file.
 */
void decode(const std::string& input_path, const std::string& output_path);

#endif // NTSC_CODEC_H
