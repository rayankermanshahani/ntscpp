#!/usr/bin/env bash
set -euo pipefail
mkdir -p build
echo "Configuring..."
cmake -S . -B build
echo "Building..."
cmake --build build --config Release
# echo "Running executable..."
# ./build/ntscpp
