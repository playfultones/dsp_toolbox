/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

namespace PlayfulTones::DspToolbox::Juce
{

    /**
 * @brief Zero-copy wrapper from juce::AudioBuffer to DSP Toolbox BufferView.
 *
 * Creates a BufferView that directly references the data in a JUCE AudioBuffer
 * without any copying. The AudioBuffer must outlive the BufferView.
 *
 * @tparam SampleType The sample type (float or double)
 * @param juceBuffer The JUCE AudioBuffer to wrap
 * @return BufferView referencing the same memory as juceBuffer
 *
 * Usage:
 * @code
 * void processBlock(juce::AudioBuffer<float>& buffer, juce::MidiBuffer&) override {
 *     auto view = makeBufferView(buffer);
 *     myProcessor.process(view);
 * }
 * @endcode
 */
    template <typename SampleType>
    [[nodiscard]] inline BufferView<SampleType> makeBufferView (juce::AudioBuffer<SampleType>& juceBuffer)
    {
        // JUCE returns float* const* but BufferView expects float**
        // The const is on the pointers themselves, not the data, so this cast is safe
        return BufferView<SampleType> (
            const_cast<SampleType**> (juceBuffer.getArrayOfWritePointers()),
            static_cast<std::size_t> (juceBuffer.getNumChannels()),
            static_cast<std::size_t> (juceBuffer.getNumSamples()));
    }

    /**
 * @brief Create a const BufferView from a const juce::AudioBuffer.
 *
 * @tparam SampleType The sample type (float or double)
 * @param juceBuffer The const JUCE AudioBuffer to wrap
 * @return Const BufferView referencing the same memory
 */
    template <typename SampleType>
    [[nodiscard]] inline BufferView<const SampleType> makeBufferView (const juce::AudioBuffer<SampleType>& juceBuffer)
    {
        return BufferView<const SampleType> (
            juceBuffer.getArrayOfReadPointers(),
            static_cast<std::size_t> (juceBuffer.getNumChannels()),
            static_cast<std::size_t> (juceBuffer.getNumSamples()));
    }

} // namespace PlayfulTones::DspToolbox::Juce
