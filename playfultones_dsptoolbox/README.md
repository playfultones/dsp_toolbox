# DSP Toolbox JUCE Adapter

JUCE user module providing seamless integration between the DSP Toolbox and JUCE audio processors.

## Features

- **Zero-copy buffer wrapping** - Direct conversion from `juce::AudioBuffer` to `BufferView`
- **RuntimeSpec fallback** - Supports any sample rate/block size combination
- **Generic adapter template** - Works with any DSP Toolbox processor
- **Minimal boilerplate** - Simple API mapping to JUCE lifecycle methods

## Installation

### Using CMake

Add the module path to your JUCE project:

```cmake
juce_add_modules(
    PATHS path/to/dsp_toolbox
    playfultones_dsptoolbox
)

target_link_libraries(YourPlugin
    PRIVATE
        playfultones_dsptoolbox
        juce::juce_audio_processors
)
```

## Usage

### Basic Integration

```cpp
#include <playfultones_dsptoolbox/playfultones_dsptoolbox.h>

class MyPluginProcessor : public juce::AudioProcessor {
public:
    void prepareToPlay(double sampleRate, int samplesPerBlock) override {
        filter_.prepareToPlay(sampleRate, samplesPerBlock);
    }

    void processBlock(juce::AudioBuffer<float>& buffer, juce::MidiBuffer&) override {
        filter_.processBlock(buffer);
    }

    void releaseResources() override {
        filter_.reset();
    }

private:
    PlayfulTones::DspToolbox::Juce::JuceProcessorAdapter<
        PlayfulTones::DspToolbox::Biquad> filter_;
};
```

### Parameter Access

```cpp
// Set parameters via visitor
filter_.visit([](auto& proc) {
    if constexpr (!std::is_same_v<std::decay_t<decltype(proc)>, std::monostate>) {
        proc.template setParam<kFrequency>(1000.0f);
        proc.template setParam<kQ>(0.707f);
    }
});
```

### Custom Spec Sets

For reduced binary size or specific requirements:

```cpp
using namespace PlayfulTones::DspToolbox;

// Only 48kHz support + runtime fallback
using MinimalSpecs = SpecSet<Spec48000_512, RuntimeSpec>;
Juce::JuceProcessorAdapter<Biquad, MinimalSpecs> filter_;
```

## How It Works

1. **DesktopSpecsWithFallback** pre-instantiates 18 common spec variants (44.1k, 48k, 96k x 6 block sizes)
2. When `prepareToPlay()` is called, it tries to match the exact spec
3. If no static match is found, **RuntimeSpec** handles it dynamically
4. Processing uses zero-copy buffer views for optimal performance

## Requirements

- JUCE 7.0 or later
- C++23 compatible compiler
- DSP Toolbox headers in sibling `include/` directory

## License

MIT License - See LICENSE file for details.
