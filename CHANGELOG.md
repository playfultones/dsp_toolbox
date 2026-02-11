# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.1] - 2026-02-11

### Fixed

- **Math Functions**: Replace bare `==`/`!=` float comparisons with `exactlyEquals()` in `sqrt`, `exp`, and `log` for correctness under `-Wfloat-equal`
- **Oversampler2x**: Fix zero-size static buffer when using `RuntimeSpec` (blockSize == 0) by clamping upsampled buffer size to minimum of 1

## [0.2.0] - 2026-02-11

### Added

- **Processors**
  - `OnePole`: First-order IIR filter (6 dB/octave) with lowpass and highpass modes, bilinear transform coefficient design
  - `Waveshaper`: Generic static nonlinear processor with concept-constrained transfer functions and zero-overhead inlining
  - `Oversampler2x`: 2x oversampling wrapper with 12-tap half-band FIR anti-aliasing filter (~70 dB rejection), polyphase implementation
  - `LookupTable`: Uniformly-spaced lookup table with cubic Hermite (Catmull-Rom) interpolation, linear interpolation, and nearest-neighbor modes

- **Filter Types**
  - `LowShelfSlope` and `HighShelfSlope` biquad types for slope-parameterized shelf filters (available in Biquad, MultibandEQ, and StereoMultibandEQ)

### Changed

- **Math Functions**: Added `exactlyEquals()` for explicit floating-point equality comparison, improved NaN and infinity handling in `sqrt`, `exp`, and `log`, added float-specific overflow/underflow thresholds for `exp`
- **Filters**: Replaced raw `==`/`!=` float comparisons with `exactlyEquals()` across Biquad, MultibandEQ, StereoMultibandEQ, and StereoGain for correctness under `-Wfloat-equal`
- **SmoothedValue**: Use `exactlyEquals` for floating-point target comparison
- **IProcessor**: Use `exactlyEquals` for sample rate matching in `ProcessorWrapper`
- **JUCE Integration**: Enable `GenericAudioProcessorEditor` in processor adapter

## [0.1.1] - 2026-01-14

### Changed

- **ConstexprGraph**: Added pre-indexed connection handling for improved processing performance. Connection lookups are now O(1) per node instead of O(C) where C is total connections.

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
