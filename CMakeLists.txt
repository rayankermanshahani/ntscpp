cmake_minimum_required(VERSION 3.16)
project(ntscpp)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find ffmpeg libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(AVFORMAT libavformat REQUIRED)
pkg_check_modules(AVCODEC libavcodec REQUIRED)
pkg_check_modules(AVUTIL libavutil REQUIRED)
pkg_check_modules(SWSCALE libswscale REQUIRED)

# find base fftw library
pkg_check_modules(FFTW fftw3 REQUIRED)

# explicitly find fftw threads library 
# (homebrew on macos doesn't provide a separate pkg-config for it)
find_library(FFTW_THREADS_LIBRARY NAMES fftw3_threads 
  HINTS /opt/homebrew/lib /usr/local/lib
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${AVFORMAT_INCLUDE_DIRS}
  ${AVCODEC_INCLUDE_DIRS}
  ${AVUTIL_INCLUDE_DIRS}
  ${SWSCALE_INCLUDE_DIRS}
  ${FFTW_INCLUDE_DIRS}
)

add_executable(ntscpp 
  src/main.cpp
  src/ntsc_encoder.cpp
  src/ntsc_decoder.cpp
  src/dsp_utils.cpp
)

target_link_libraries(ntscpp
  ${AVFORMAT_LDFLAGS}
  ${AVCODEC_LDFLAGS}
  ${AVUTIL_LDFLAGS}
  ${SWSCALE_LDFLAGS}
  ${FFTW_LIBRARIES}
  ${FFTW_THREADS_LIBRARY}
  pthread # required for fftw threading
)

# performance optimization flags
target_compile_options(ntscpp PRIVATE -O3 -march=native)
