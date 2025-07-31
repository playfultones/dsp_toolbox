/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once
#include "../processor.h"

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief High-performance output processor using CRTP
     * 
     * This processor simply passes audio through without modification.
     * It serves as a dedicated terminal node in the audio processing graph.
     */
    template<typename SampleType = float, 
             size_t BlockSize = 512, 
             size_t SampleRate = 44100,
             size_t NumChannels = 2>
    class OutputProcessor : public ProcessorBase<OutputProcessor<SampleType, BlockSize, SampleRate, NumChannels>,
                                                 SampleType, BlockSize, SampleRate, NumChannels>
    {
    public:
        using Base = ProcessorBase<OutputProcessor, SampleType, BlockSize, SampleRate, NumChannels>;
        using AudioBuffer = typename Base::AudioBuffer;
        
        OutputProcessor() = default;

        void prepare_impl() noexcept
        {
            // No processing needed for output node
        }

        void process_audio_impl(AudioBuffer& /* buffer */) noexcept
        {
            // This is the terminal node, it just passes audio through
            // No processing needed
        }
        
        void process_control_impl() noexcept
        {
            // No control processing needed
        }

        void reset_impl() noexcept
        {
            // Nothing to reset
        }
    };
    
    // Common type aliases for convenience
    using OutputProcessorF32 = OutputProcessor<float, 512, 44100, 2>;
    using OutputProcessorF64 = OutputProcessor<double, 512, 44100, 2>;
}
