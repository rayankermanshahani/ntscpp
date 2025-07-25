/**
 * @file main.cpp
 * @brief CLI entrypoint for NTSC-M codec.
 *
 * Parses command-line arguments and dispatches encoding/decoding operations.
 *
 * Usage:
 *    ./ntscpp encode <input.mp4> <output.ntsc>
 *    ./ntscpp decode <input.ntsc> <output.mp4>
 */
#include <iostream>
#include <string>

#include "../include/ntsc_codec.h"

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
