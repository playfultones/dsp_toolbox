/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once
#include "../processors/processor.h"
#include <algorithm>
#include <atomic>
#include <cmath>

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief High-performance limiter processor using CRTP
     *
     * This class implements a hard limiter that ensures no sample exceeds
     * the specified ceiling value. Uses compile-time optimization and 
     * lockless parameter updates.
     * 
     * @tparam SampleType The sample type (float, double)
     * @tparam BlockSize Fixed block size for processing
     * @tparam SampleRate Sample rate (compile-time constant)
     * @tparam NumChannels Number of audio channels
     */
    template <typename SampleType = float,
        size_t BlockSize = 512,
        size_t SampleRate = 44100,
        size_t NumChannels = 2>
    class Limiter : public ProcessorBase<Limiter<SampleType, BlockSize, SampleRate, NumChannels>,
                        SampleType,
                        BlockSize,
                        SampleRate,
                        NumChannels>
    {
    public:
        using Base = ProcessorBase<Limiter, SampleType, BlockSize, SampleRate, NumChannels>;
        using AudioBuffer = typename Base::AudioBuffer;
        using sample_type = SampleType;

        static constexpr sample_type kNoReduction = sample_type { 1 };
        static constexpr sample_type kSmallValue = sample_type { 1e-6 };

        /**
         * @brief Construct a new Limiter processor
         * @param ceiling Maximum allowed amplitude after limiting
         */
        Limiter (sample_type ceiling = sample_type { 1 })
            : ceiling_ (ceiling)
        {
            ceiling_.store (ceiling, std::memory_order_relaxed);
        }

        void prepare_impl() noexcept
        {
            // Nothing to prepare for limiter
        }

        void reset_impl() noexcept
        {
            // Nothing to reset for limiter
        }

        void process_audio_impl (AudioBuffer& buffer) noexcept
        {
            constexpr size_t numFrames = Base::block_size;
            constexpr size_t numChannels = Base::num_channels;

            const sample_type ceiling = ceiling_.load (std::memory_order_relaxed);

            // Process each channel
            for (size_t ch = 0; ch < numChannels; ++ch)
            {
                sample_type* channelData = buffer[ch].data();

                // Process samples with branch prediction hints
                for (size_t i = 0; i < numFrames; ++i)
                {
                    const sample_type sample = channelData[i];
                    const sample_type absValue = std::abs (sample);

                    // Branch prediction: most samples don't need limiting
                    if (absValue > ceiling) [[unlikely]]
                    {
                        const sample_type gainReduction = ceiling / (absValue + kSmallValue);
                        channelData[i] = sample * gainReduction;
                    }
                }
            }
        }

        void process_control_impl() noexcept
        {
            // No control-rate processing needed for basic limiter
        }

        /**
         * @brief Set the limiter ceiling (thread-safe)
         * @param newCeiling The new ceiling value
         */
        void setCeiling (sample_type newCeiling) noexcept
        {
            ceiling_.store (std::max (kSmallValue, newCeiling), std::memory_order_relaxed);
        }

        /**
         * @brief Get the current ceiling value
         */
        sample_type getCeiling() const noexcept
        {
            return ceiling_.load (std::memory_order_relaxed);
        }

    private:
        // Thread-safe parameter storage
        std::atomic<sample_type> ceiling_ { sample_type { 1 } };
    };

    // Common type aliases
    using LimiterF32 = Limiter<float, 512, 44100, 2>;
    using LimiterF64 = Limiter<double, 512, 44100, 2>;

} // namespace PlayfulTones::DspToolBox
