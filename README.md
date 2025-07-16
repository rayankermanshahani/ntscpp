## ntscpp

An NTSC-M codec implementation in C++ for encoding/decoding baseband composite signals.

### Overview
This project provides a command-line tool to encode RGB video frames to NTSC composite signals and decode them back, simulating analog TV standards.
It uses FFmpeg for MP4 I/O and FFTW for DSP.

### Build and Run
Run `./runme.sh` to build and execute the binary.

### Usage
Encoding mode: `./build/ntscpp encode input.mp4 output.ntsc`
Decoding mode: `./build/ntscpp decode input.ntsc output.mp4`

### To-Do
- Optimize DSP operations via batching
- Add audio support (FM modulation for audio signal)
- Add support for encoding / decoding live streams of data


