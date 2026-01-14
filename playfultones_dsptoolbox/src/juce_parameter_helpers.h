/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include <juce_audio_processors/juce_audio_processors.h>

#include <string>

namespace PlayfulTones::DspToolbox::Juce
{

    /**
 * @brief Create JUCE parameter layout from a compile-time ParamDescriptor array.
 *
 * Maps DSP Toolbox ParamDescriptor metadata to JUCE AudioParameterFloat:
 * - id -> ParameterID
 * - name -> Parameter name
 * - minValue/maxValue -> NormalisableRange
 * - defaultValue -> Default value
 * - unit -> Parameter label
 *
 * @tparam Descriptors Compile-time array of ParamDescriptor
 * @return ParameterLayout for AudioProcessorValueTreeState construction
 *
 * ## Example
 * @code
 * using MyParams = ParamSet<MyParamDescriptors>;
 * auto layout = createParameterLayout<MyParams::descriptors>();
 * @endcode
 */
    template <auto Descriptors>
    [[nodiscard]] inline juce::AudioProcessorValueTreeState::ParameterLayout
        createParameterLayout()
    {
        juce::AudioProcessorValueTreeState::ParameterLayout layout;

        for (std::size_t i = 0; i < Descriptors.size(); ++i)
        {
            const auto& desc = Descriptors[i];

            auto range = juce::NormalisableRange<float> (desc.minValue, desc.maxValue);

            auto param = std::make_unique<juce::AudioParameterFloat> (
                juce::ParameterID { std::string (desc.id.c_str()), 1 },
                std::string (desc.name.c_str()),
                range,
                desc.defaultValue,
                juce::AudioParameterFloatAttributes().withLabel (std::string (desc.unit.c_str())));

            layout.add (std::move (param));
        }

        return layout;
    }

} // namespace PlayfulTones::DspToolbox::Juce
