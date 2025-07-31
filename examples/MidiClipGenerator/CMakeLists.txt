cmake_minimum_required(VERSION 3.12)
project(MidiClipGenerator)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add executable
add_executable(midi_clip_generator src/main.cpp)

# Include header-only library
target_include_directories(midi_clip_generator PRIVATE ../../)
