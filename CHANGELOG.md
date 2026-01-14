# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - 2026-01-14

Initial development release of the DSP Toolbox - a header-only C++23 library providing high-performance, validated DSP building blocks for real-time audio applications.

### Added

- **Core Infrastructure**
  - Strong types for audio parameters: `Frequency`, `Decibels`, `LinearGain`, `SampleRate`, `Milliseconds`, `Seconds`, `Samples`, `MidiNoteNumber`
  - `BufferView` and `StaticAudioBuffer` for zero-allocation audio buffer handling
  - `HeapAudioBuffer` for dynamic buffer allocation
  - `ProcessSpec` and `ConstexprSpec` for processor configuration
  - `SmoothedValue` for parameter smoothing
  - `ModulationSystem` for modular parameter modulation
  - User-defined literals for intuitive parameter creation (`_Hz`, `_dB`, `_ms`, etc.)

- **Math Utilities**
  - Constexpr mathematical functions optimized for DSP
  - Mathematical constants for audio processing

- **Processors**
  - CRTP-based processor architecture with `IProcessor` interface and `ProcessorBase`
  - **Filters**: Biquad filter, Multiband EQ, Stereo Multiband EQ
  - **Generators**: VCO (voltage-controlled oscillator), White Noise
  - **Modulators**: AD Envelope, Clock, Clock Divider, Step Sequencer
  - **Utilities**: Gain, Stereo Gain, VCA, Attenuverter, Mixer, Panner, Stereo Expander
  - **Graph**: Constexpr graph processing system

- **SIMD Support**
  - Block-based SIMD processing utilities
  - Cross-platform SIMD abstraction layer

- **Benchmarking Framework**
  - Autotune capabilities for performance optimization
  - Execution plan and executor for benchmark orchestration

- **JUCE Integration**
  - `playfultones_dsptoolbox` module for JUCE framework integration
  - Buffer view adapters for JUCE audio buffers
  - Parameter helpers for JUCE plugin development
  - Processor adapter for seamless integration

- **Developer Tooling**
  - Clang-format, clang-tidy, and clangd configurations
