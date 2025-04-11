#!/bin/bash
set -e

# Create build directory
mkdir -p build
cd build

# Configure and build
cmake -G Ninja ..
cmake --build .

# Run tests
ctest --output-on-failure