# DSP Toolbox

Header-only C++23 library providing high-performance DSP building blocks for real-time audio applications.

**Status: In Development** - APIs may change without notice.

## Features

The `include/dsp_toolbox/` directory provides:

- **Core**: Buffer views, strong types (Decibels, Frequency, SampleRate), smoothed values, process specs
- **Processors**: Gain, biquad filters, multiband EQ, VCO, white noise, AD envelope, step sequencer
- **Math**: Constexpr math functions and constants
- **SIMD**: Portable acceleration via [xsimd](https://github.com/xtensor-stack/xsimd) (ARM NEON, SSE, AVX)

## JUCE Integration

The `playfultones_dsptoolbox/` module provides JUCE adapters for zero-copy buffer conversion and processor wrapping.

## Usage

Add the include directory to your project and include what you need:

```cmake
target_include_directories(your_target PRIVATE path/to/dsp_toolbox/include)
```

```cpp
#include <dsp_toolbox/core/decibels.hpp>
#include <dsp_toolbox/processors/utilities/gain.hpp>
```

For SIMD features, also add [xsimd](https://github.com/xtensor-stack/xsimd) to your project.

## Requirements

- C++23 compiler

## License

MIT License
