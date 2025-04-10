#!/bin/bash
set -e

# Create build directory
mkdir -p build
cd build

# Configure and build
cmake ..
cmake --build .

# Run tests
ctest --output-on-failure