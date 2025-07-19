## ntscpp

An NTSC-M codec implementation in C++ for encoding/decoding baseband composite signals.

### Overview
This project provides a command-line tool to encode RGB video frames to NTSC composite signals and decode them back, simulating analog TV standards.

### Requirements
- [CMake](https://cmake.org/) for the build system.
- [FFmpeg](https://ffmpeg.org/) for audio and video format conversions.
- [FFTW](https://www.fftw.org/) for performant Discrete Fourier Transform operations (with [multithreading](https://www.fftw.org/doc/Installation-and-Supported-Hardware_002fSoftware.html) support).

### Build
Run `chmod +x runme.sh && ./runme.sh` to build the binary.

### Usage
- Encoding MP4 to NTSC: `./build/ntscpp encode input.mp4 output.ntsc`
- Decoding NTSC to MP4: `./build/ntscpp decode input.ntsc output.mp4`

### Future upgrades
- Optimize DSP operations via batching
- Add audio support (FM modulation for audio signal)
- Add support for encoding / decoding live streams of data


