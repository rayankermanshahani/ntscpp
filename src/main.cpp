/**
 * @file main.cpp
 * @brief CLI entrypoint for NTSC-M codec.
 *
 * Parses command-line arguments and dispatches encoding/decoding operations
 * using FFMPEG for MP4 I/O and the NTSC encoder/decoder operations.
 *
 * Usage:
 *    ./ntscpp encode <input.mp4> <output.ntsc>
 *    ./ntscpp decode <input.ntsc> <output.mp4>
 *
 *
 * Assumptions for first iteration:
 * - Input MP4 for encoding: 480p30 progressive video (no audio, exact
 * dimensions)
 * - Splits progressive frames into interlaced fields (odd/even lines)
 * - Output .ntsc: raw binary doubles of composite signal (full video)
 * - For decoding: Assumes .ntsc contains integer number of fields
 * - Outpput MP4 for decoding: H.264 extended 480p30 (YUV420P)
 * - No error handling for mismatched fimensions/framerates
 * - No audio support
 */

// #include <array>
#include <algorithm>
#include <array>
#include <cerrno>
// #include <cmath>
// #include <concepts>
#include <cstdint>
#include <fstream>
#include <ios>
#include <iostream>
#include <ostream>
#include <string>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#include "../include/ntsc_common.h"

// Forward declarations for encoder/decoder (defined in ntsc_encoder.cp:writesp
// and ntsc_decoder.cpp)
SignalSamples encode_field(const VideoFrame& input_frame);
VideoFrame decode_field(const SignalSamples& input_signal, bool is_odd_field);

/**
 * @brief Encode MP4 video to raw NTSC composite signal (.ntsc).
 *
 * Reads frames from input MP4, splits into composite fields, encodes each
 * field, and writes concatenated raw double samples to output.
 *
 * @param input_path Path to input MP4 file.
 * @param output_path Path to output .ntsc file.
 */
void encode(const std::string& input_path, const std::string& output_path) {
  AVFormatContext* fmt_ctx = nullptr;
  if (avformat_open_input(&fmt_ctx, input_path.c_str(), nullptr, nullptr) < 0) {
    std::cerr << "Error: Could not open input file " << input_path << std::endl;
    return;
  }

  if (avformat_find_stream_info(fmt_ctx, nullptr) < 0) {
    std::cerr << "Error: Could not find stream info" << std::endl;
    avformat_close_input(&fmt_ctx);
    return;
  }

  int video_stream_idx = -1;
  for (unsigned int i = 0; i < fmt_ctx->nb_streams; ++i) {
    if (fmt_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
      video_stream_idx = i;
      break;
    }
  }
  if (video_stream_idx == -1) {
    std::cerr << "Error: No video stream found" << std::endl;
    avformat_close_input(&fmt_ctx);
    return;
  }

  AVCodecParameters* codec_parameters =
      fmt_ctx->streams[video_stream_idx]->codecpar;
  const AVCodec* decoder = avcodec_find_decoder(codec_parameters->codec_id);
  if (!decoder) {
    std::cerr << "Error: Unsupported codec" << std::endl;
    avformat_close_input(&fmt_ctx);
    return;
  }

  AVCodecContext* dec_ctx = avcodec_alloc_context3(decoder);
  if (!dec_ctx) {
    std::cerr << "Error: Could not allocate decoder context" << std::endl;
    avformat_close_input(&fmt_ctx);
    return;
  }

  if (avcodec_parameters_to_context(dec_ctx, codec_parameters) < 0) {
    std::cerr << "Error: Could not copy codec parameters" << std::endl;
    avcodec_free_context(&dec_ctx);
    avformat_close_input(&fmt_ctx);
    return;
  }

  if (avcodec_open2(dec_ctx, decoder, nullptr) < 0) {
    std::cerr << "Error: Could not open decoder" << std::endl;
    avcodec_free_context(&dec_ctx);
    avformat_close_input(&fmt_ctx);
    return;
  }

  SwsContext* sws_ctx =
      sws_getContext(dec_ctx->width, dec_ctx->height, dec_ctx->pix_fmt,
                     dec_ctx->width, dec_ctx->height, AV_PIX_FMT_RGB24,
                     SWS_BICUBLIN, nullptr, nullptr, nullptr);
  if (!sws_ctx) {
    std::cerr << "Error: Could not initialize swscale context" << std::endl;
    avcodec_free_context(&dec_ctx);
    avformat_close_input(&fmt_ctx);
    return;
  }

  AVFrame* frame = av_frame_alloc();
  AVFrame* rgb_frame = av_frame_alloc();
  if (!frame || !rgb_frame) {
    std::cerr << "Error: Could not allocate frames" << std::endl;
    sws_freeContext(sws_ctx);
    avcodec_free_context(&dec_ctx);
    avformat_close_input(&fmt_ctx);
    return;
  }

  uint8_t* rgb_buffer =
      static_cast<uint8_t*>(av_malloc(av_image_get_buffer_size(
          AV_PIX_FMT_RGB24, dec_ctx->width, dec_ctx->height, 1)));
  av_image_fill_arrays(rgb_frame->data, rgb_frame->linesize, rgb_buffer,
                       AV_PIX_FMT_RGB24, dec_ctx->width, dec_ctx->height, 1);

  SignalSamples full_signal;
  AVPacket* pkt = av_packet_alloc();
  if (!pkt) {
    std::cerr << "Error: Could not allocate packet" << std::endl;
    sws_freeContext(sws_ctx);
    avcodec_free_context(&dec_ctx);
    avformat_close_input(&fmt_ctx);
    return;
  }

  while (av_read_frame(fmt_ctx, pkt) >= 0) {
    if (pkt->stream_index != video_stream_idx) {
      av_packet_unref(pkt);
      continue;
    }

    if (avcodec_send_packet(dec_ctx, pkt) < 0) {
      std::cerr << "Error: Failed to send packet to decoder" << std::endl;
      break;
    }

    while (true) {
      int ret = avcodec_receive_frame(dec_ctx, frame);
      if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
        break;
      } else if (ret < 0) {
        std::cerr << "Error: Failed to receive frame from decoder" << std::endl;
        break;
      }

      sws_scale(sws_ctx, frame->data, frame->linesize, 0, frame->height,
                rgb_frame->data, rgb_frame->linesize);

      // Split into two fields (odd and even lines)
      for (bool is_odd : {true, false}) {
        VideoFrame vframe;
        vframe.is_field_odd = is_odd;
        vframe.height = dec_ctx->height / 2;
        vframe.width = dec_ctx->width;
        vframe.pixels.resize(vframe.width * vframe.height);

        int field_line = 0;
        for (int y = is_odd ? 0 : 1; y < dec_ctx->height; y += 2) {
          for (int x = 0; x < dec_ctx->width; ++x) {
            int offset = y * rgb_frame->linesize[0] + x * 3;
            double r = rgb_frame->data[0][offset] / 255.0;
            double g = rgb_frame->data[0][offset + 1] / 255.0;
            double b = rgb_frame->data[0][offset + 2] / 255.0;
            vframe.pixels[field_line * vframe.width + x] = {r, g, b};
          }
          ++field_line;
        }

        SignalSamples field_signal = encode_field(vframe);
        full_signal.insert(full_signal.end(), field_signal.begin(),
                           field_signal.end());
      }
    }
    av_packet_unref(pkt);
  }

  // Cleanup FFmpeg
  av_packet_free(&pkt);
  av_freep(&rgb_buffer);
  av_frame_free(&frame);
  av_frame_free(&rgb_frame);
  sws_freeContext(sws_ctx);
  avcodec_free_context(&dec_ctx);
  avformat_close_input(&fmt_ctx);

  // Write raw doubles to .ntsc output file
  std::ofstream outf(output_path, std::ios::binary);
  if (!outf) {
    std::cerr << "Error: Could not open output file " << output_path
              << std::endl;
    return;
  }
  outf.write(reinterpret_cast<const char*>(full_signal.data()),
             full_signal.size() * sizeof(double));
  outf.close();
  std::cout << "Encoding complete: " << output_path << std::endl;
}

/**
 * @brief Decode raw NTSC composite signal (.ntsc) to MP4 video.
 *
 * Reads raw doubles, splits into fields based on expected size, decodes each
 * field, interleaves into frames, and encodes output to MP4.
 *
 * @param input_path Path to input .ntsc file.
 * @param output_path Path to output MP4 file.
 */
void decode(const std::string& input_path, const std::string& output_path) {
  // Read raw .ntsc file
  std::ifstream inf(input_path, std::ios::binary);
  if (!inf) {
    std::cerr << "Error: Could not open input file" << std::endl;
    return;
  }
  inf.seekg(0, std::ios::end);
  size_t file_size = inf.tellg();
  inf.seekg(0, std::ios::beg);

  size_t num_samples = file_size / sizeof(double);
  SignalSamples full_signal(num_samples);
  inf.read(reinterpret_cast<char*>(full_signal.data()), file_size);
  inf.close();

  // Compute expected field size (accounts for 262 full lines + half line)
  size_t field_size =
      static_cast<size_t>(FULL_LINES_PER_FIELD) * SAMPLES_PER_LINE +
      (SAMPLES_PER_LINE / 2);
  size_t num_fields = num_samples / field_size;
  if (num_fields < 2 || num_samples % field_size != 0) {
    std::cerr << "Error: Input size not multiple of field size" << std::endl;
    return;
  }

  // Setup output MP4
  AVFormatContext* out_fmt_ctx = nullptr;
  if (avformat_alloc_output_context2(&out_fmt_ctx, nullptr, nullptr,
                                     output_path.c_str()) < 0) {
    std::cerr << "Error: Could not allocate output context" << std::endl;
    return;
  }

  const AVCodec* encoder = avcodec_find_encoder(AV_CODEC_ID_H264);
  if (!encoder) {
    std::cerr << "Error: H.264 encoder not found" << std::endl;
    avformat_free_context(out_fmt_ctx);
    return;
  }

  AVStream* out_stream = avformat_new_stream(out_fmt_ctx, nullptr);
  if (!out_stream) {
    std::cerr << "Error: Could not create output stream" << std::endl;
    avformat_free_context(out_fmt_ctx);
    return;
  }

  AVCodecContext* enc_ctx = avcodec_alloc_context3(encoder);
  if (!enc_ctx) {
    std::cerr << "Error: Could not allocate encoder context" << std::endl;
    avformat_free_context(out_fmt_ctx);
    return;
  }

  AVRational ntsc_framerate = {30000, 1001}; // ~29.97 fps

  enc_ctx->width = VISIBLE_WIDTH;
  enc_ctx->height = VISIBLE_HEIGHT;
  enc_ctx->pix_fmt = AV_PIX_FMT_YUV420P;
  enc_ctx->time_base = {ntsc_framerate.den, ntsc_framerate.num}; // inverse fr
  enc_ctx->framerate = ntsc_framerate;
  enc_ctx->gop_size = 12;
  enc_ctx->max_b_frames = 1;

  if (out_fmt_ctx->oformat->flags & AVFMT_GLOBALHEADER) {
    enc_ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
  }

  AVDictionary* opts = nullptr;
  av_dict_set(&opts, "preset", "slow", 0);
  av_dict_set(&opts, "crf", "23", 0);

  if (avcodec_open2(enc_ctx, encoder, &opts) < 0) {
    std::cerr << "Error: Could not open encoder" << std::endl;
    av_dict_free(&opts);
    avcodec_free_context(&enc_ctx);
    avformat_free_context(out_fmt_ctx);
    return;
  }
  av_dict_free(&opts);

  if (avcodec_parameters_from_context(out_stream->codecpar, enc_ctx) < 0) {
    std::cerr << "Error: Could not set stream parameters" << std::endl;
    av_dict_free(&opts);
    avcodec_free_context(&enc_ctx);
    avformat_free_context(out_fmt_ctx);
    return;
  }

  if (!(out_fmt_ctx->oformat->flags & AVFMT_NOFILE)) {
    if (avio_open(&out_fmt_ctx->pb, output_path.c_str(), AVIO_FLAG_WRITE) < 0) {
      std::cerr << "Error: Could not open output file " << output_path
                << std::endl;
      avcodec_free_context(&enc_ctx);
      avformat_free_context(out_fmt_ctx);
      return;
    }
  }

  if (avformat_write_header(out_fmt_ctx, nullptr) < 0) {
    std::cerr << "Error: Could not write header" << std::endl;
    avcodec_free_context(&enc_ctx);
    avformat_free_context(out_fmt_ctx);
    return;
  }

  // Setup swscale for RGB to YUV420P
  SwsContext* out_sws_ctx =
      sws_getContext(VISIBLE_WIDTH, VISIBLE_HEIGHT, AV_PIX_FMT_RGB24,
                     VISIBLE_WIDTH, VISIBLE_HEIGHT, AV_PIX_FMT_YUV420P,
                     SWS_BICUBLIN, nullptr, nullptr, nullptr);

  if (!out_sws_ctx) {
    std::cerr << "Error: Could not initialize output swscale" << std::endl;
    avcodec_free_context(&enc_ctx);
    avformat_free_context(out_fmt_ctx);
    return;
  }

  AVFrame* out_frame = av_frame_alloc();
  if (!out_frame) {
    std::cerr << "Error: Could not allocate output frame" << std::endl;
    sws_freeContext(out_sws_ctx);
    avcodec_free_context(&enc_ctx);
    avformat_free_context(out_fmt_ctx);
    return;
  }
  out_frame->format = AV_PIX_FMT_YUV420P;
  out_frame->width = VISIBLE_WIDTH;
  out_frame->height = VISIBLE_HEIGHT;
  if (av_frame_get_buffer(out_frame, 0) < 0) {
    std::cerr << "Error: Could not allocate output frame buffer" << std::endl;
    av_frame_free(&out_frame);
    sws_freeContext(out_sws_ctx);
    avcodec_free_context(&enc_ctx);
    avformat_free_context(out_fmt_ctx);
    return;
  }
  // Temporate RGB frame for interleaving
  AVFrame* rgb_temp = av_frame_alloc();
  uint8_t* rgb_buffer =
      static_cast<uint8_t*>(av_malloc(av_image_get_buffer_size(
          AV_PIX_FMT_RGB24, VISIBLE_WIDTH, VISIBLE_HEIGHT, 1)));
  av_image_fill_arrays(rgb_temp->data, rgb_temp->linesize, rgb_buffer,
                       AV_PIX_FMT_RGB24, VISIBLE_WIDTH, VISIBLE_HEIGHT, 1);

  int frame_count = 0;
  for (size_t field_idx = 0; field_idx < num_fields; field_idx += 2) {
    // Decode odd field first
    size_t odd_start = field_idx * field_size;
    SignalSamples odd_signal(full_signal.begin() + odd_start,
                             full_signal.begin() + odd_start + field_size);
    VideoFrame odd_frame = decode_field(odd_signal, true);

    // Decode even field second
    size_t even_start = (field_idx + 1) * field_size;
    SignalSamples even_signal(full_signal.begin() + even_start,
                              full_signal.begin() + even_start + field_size);
    VideoFrame even_frame = decode_field(even_signal, false);

    if (odd_frame.pixels.empty() || even_frame.pixels.empty()) {
      std::cerr << "Warning: Skipping invalid field pair" << std::endl;
      continue;
    }

    // Interleave fields into full RGB frame
    for (int y = 0; y < VISIBLE_HEIGHT; ++y) {
      const VideoFrame& src_frame = (y % 2 == 0) ? odd_frame : even_frame;
      int src_y = y / 2;
      for (int x = 0; x < VISIBLE_WIDTH; ++x) {
        std::array<double, 3> pix = src_frame.pixels[src_y * VISIBLE_WIDTH + x];
        int offset = y * rgb_temp->linesize[0] + x * 3;
        rgb_temp->data[0][offset] =
            static_cast<uint8_t>(std::clamp(pix[0] * 255.0, 0.0, 255.0));
        rgb_temp->data[0][offset + 1] =
            static_cast<uint8_t>(std::clamp(pix[1] * 255.0, 0.0, 255.0));
        rgb_temp->data[0][offset + 2] =
            static_cast<uint8_t>(std::clamp(pix[2] * 255.0, 0.0, 255.0));
      }
    }

    // Convert RGB to YUV420P
    sws_scale(out_sws_ctx, rgb_temp->data, rgb_temp->linesize, 0,
              VISIBLE_HEIGHT, out_frame->data, out_frame->linesize);

    out_frame->pts = frame_count; // Set presentation timestamp

    if (avcodec_send_frame(enc_ctx, out_frame) < 0) {
      std::cerr << "Error: Failed to send frame to encoder" << std::endl;
      break;
    }

    AVPacket* enc_pkt = av_packet_alloc();
    if (!enc_pkt) {
      std::cerr << "Error: Could not allocate packet" << std::endl;
      sws_freeContext(out_sws_ctx);
      avcodec_free_context(&enc_ctx);
      avformat_close_input(&out_fmt_ctx);
      return;
    }

    while (true) {
      int ret = avcodec_receive_packet(enc_ctx, enc_pkt);
      if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
        break;
      } else if (ret < 0) {
        std::cerr << "Error: Failed to receive packet from encoder"
                  << std::endl;
        break;
      }

      enc_pkt->stream_index = 0;
      av_packet_rescale_ts(enc_pkt, enc_ctx->time_base, out_stream->time_base);
      if (av_interleaved_write_frame(out_fmt_ctx, enc_pkt) < 0) {
        std::cerr << "Error: Failed to write packet" << std::endl;
      }
      av_packet_unref(enc_pkt);
    }
    ++frame_count;
  }

  // Flush encoder
  avcodec_send_frame(enc_ctx, nullptr);
  AVPacket* enc_pkt = av_packet_alloc();
  if (!enc_pkt) {
    std::cerr << "Error: Could not allocate packet" << std::endl;
    sws_freeContext(out_sws_ctx);
    avcodec_free_context(&enc_ctx);
    avformat_close_input(&out_fmt_ctx);
    return;
  }
  while (avcodec_receive_packet(enc_ctx, enc_pkt) == 0) {
    enc_pkt->stream_index = 0;
    av_packet_rescale_ts(enc_pkt, enc_ctx->time_base, out_stream->time_base);
    av_interleaved_write_frame(out_fmt_ctx, enc_pkt);
    av_packet_unref(enc_pkt);
  }

  av_write_trailer(out_fmt_ctx);

  // Cleanup FFmpeg
  av_packet_free(&enc_pkt);
  av_freep(&rgb_buffer);
  av_frame_free(&out_frame);
  av_frame_free(&rgb_temp);
  sws_freeContext(out_sws_ctx);
  if (!(out_fmt_ctx->oformat->flags & AVFMT_NOFILE)) {
    avio_closep(&out_fmt_ctx->pb);
  }
  avcodec_free_context(&enc_ctx);
  avformat_free_context(out_fmt_ctx);

  std::cout << "Decoding complete: " << output_path << std::endl;
}

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "Usage: " << argv[0] << " <encode|decode> <input> <output>"
              << std::endl;
    return 1;
  }

  std::string mode = argv[1];
  std::string input = argv[2];
  std::string output = argv[3];

  if (mode == "encode") {
    encode(input, output);
  } else if (mode == "decode") {
    decode(input, output);
  } else {
    std::cerr << "Error: Invalid mode. Use 'encode' or 'decode'." << std::endl;
    return 1;
  }

  return 0;
}
