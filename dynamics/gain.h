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

// Platform-specific denormal handling
#if defined(__SSE__)
    #include <xmmintrin.h>
#endif

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief High-performance gain processor with ramping capabilities using CRTP
     *
     * This class implements a high-performance gain processor that can apply a gain 
     * factor to audio samples with smooth ramping to avoid clicks and pops.
     * Uses compile-time optimization and lockless parameter updates.
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
    class Gain : public ProcessorBase<Gain<SampleType, BlockSize, SampleRate, NumChannels>,
                     SampleType,
                     BlockSize,
                     SampleRate,
                     NumChannels>
    {
    public:
        using Base = ProcessorBase<Gain, SampleType, BlockSize, SampleRate, NumChannels>;
        using AudioBuffer = typename Base::AudioBuffer;
        using sample_type = SampleType;

        /**
         * @brief Construct a new Gain processor
         * @param initialGain Initial gain value
         */
        Gain (sample_type initialGain = sample_type { 1 })
            : gain_ (initialGain), targetGain_ (initialGain)
        {
            targetGain_.store (initialGain, std::memory_order_relaxed);
        }

        void prepare_impl() noexcept
        {
            // Set up denormal handling if available
#if defined(__SSE__)
            _MM_SET_FLUSH_ZERO_MODE (_MM_FLUSH_ZERO_ON);
            _MM_SET_DENORMALS_ZERO_MODE (_MM_DENORMALS_ZERO_ON);
#endif
            reset_impl();
        }

        void reset_impl() noexcept
        {
            gain_ = targetGain_.load (std::memory_order_relaxed);
            currentRampSample_ = 0;
        }

        void process_audio_impl (AudioBuffer& buffer) noexcept
        {
            constexpr size_t numFrames = Base::block_size;
            constexpr size_t numChannels = Base::num_channels;

            const sample_type targetGain = targetGain_.load (std::memory_order_relaxed);
            const size_t rampLengthSamples = rampLengthSamples_.load (std::memory_order_relaxed);

            // Branch prediction friendly - check if ramping is needed
            const bool needsRamping = (rampLengthSamples > 0 && currentRampSample_ < rampLengthSamples && std::abs (targetGain - gain_) > sample_type { 0.0001 });

            if (needsRamping) [[unlikely]]
            {
                // Process with gain ramping (less common path)
                const sample_type gainIncrement = (targetGain - gain_) / static_cast<sample_type> (rampLengthSamples);

                for (size_t i = 0; i < numFrames; ++i)
                {
                    // Branchless ramp update
                    const bool stillRamping = currentRampSample_ < rampLengthSamples;
                    gain_ += stillRamping ? gainIncrement : sample_type { 0 };
                    currentRampSample_ += stillRamping ? 1 : 0;

                    // Apply gain to all channels with denormal prevention
                    const sample_type processedGain = gain_ + kDenormalOffset;
                    for (size_t ch = 0; ch < numChannels; ++ch)
                    {
                        buffer[ch][i] = buffer[ch][i] * processedGain + kDenormalOffset;
                    }
                }
            }
            else [[likely]]
            {
                // Process without ramping (fast path)
                gain_ = targetGain;

                // SIMD-friendly processing - apply denormal prevention
                const sample_type processedGain = gain_ + kDenormalOffset;

                // Loop unroll hint for compiler - process channels
                for (size_t ch = 0; ch < numChannels; ++ch)
                {
                    sample_type* channelData = buffer[ch].data();

                    // Process samples in blocks for better vectorization
                    // Compiler can auto-vectorize this pattern more easily
                    for (size_t i = 0; i < numFrames; ++i)
                    {
                        channelData[i] = channelData[i] * processedGain + kDenormalOffset;
                    }
                }
            }
        }

        void process_control_impl() noexcept
        {
            // Control-rate processing - update ramp length if needed
            const sample_type rampLengthSeconds = rampLengthSeconds_.load (std::memory_order_relaxed);
            const size_t newRampLengthSamples = static_cast<size_t> (rampLengthSeconds * Base::sample_rate);

            if (newRampLengthSamples != rampLengthSamples_.load (std::memory_order_relaxed))
            {
                rampLengthSamples_.store (newRampLengthSamples, std::memory_order_relaxed);
                currentRampSample_ = 0; // Reset ramp on length change
            }
        }

        /**
         * @brief Set the target gain value (thread-safe)
         * @param newGain The new gain value
         */
        void setGain (sample_type newGain) noexcept
        {
            targetGain_.store (newGain, std::memory_order_relaxed);

            // If no ramping, apply immediately
            if (rampLengthSamples_.load (std::memory_order_relaxed) == 0)
            {
                gain_ = newGain;
            }
        }

        /**
         * @brief Set the gain ramp length in seconds (thread-safe)
         * @param seconds Ramp duration in seconds
         */
        void setGainRampLength (sample_type seconds) noexcept
        {
            rampLengthSeconds_.store (seconds, std::memory_order_relaxed);
        }

        /**
         * @brief Get the current gain value
         */
        sample_type getCurrentGain() const noexcept
        {
            return gain_;
        }

        /**
         * @brief Get the target gain value
         */
        sample_type getTargetGain() const noexcept
        {
            return targetGain_.load (std::memory_order_relaxed);
        }

        /**
         * @brief Get the ramp length in seconds
         */
        sample_type getRampLengthSeconds() const noexcept
        {
            return rampLengthSeconds_.load (std::memory_order_relaxed);
        }

        /**
         * @brief Check if ramping is currently active
         */
        bool isRamping() const noexcept
        {
            const size_t rampLength = rampLengthSamples_.load (std::memory_order_relaxed);
            return rampLength > 0 && currentRampSample_ < rampLength;
        }

    private:
        // Audio thread state (not atomic - only accessed from audio thread)
        sample_type gain_ { 1 };
        size_t currentRampSample_ { 0 };

        // Thread-safe parameter storage - aligned to prevent false sharing
#ifdef _MSC_VER
    #pragma warning(push)
    #pragma warning(disable : 4324) // Suppress structure padding warning
#endif
        alignas (64) std::atomic<sample_type> targetGain_ { sample_type { 1 } };
        alignas (64) std::atomic<sample_type> rampLengthSeconds_ { sample_type { 0 } };
        alignas (64) std::atomic<size_t> rampLengthSamples_ { 0 };
#ifdef _MSC_VER
    #pragma warning(pop)
#endif

        // Denormal prevention constant
        static constexpr sample_type kDenormalOffset = sample_type { 1e-25 };
    };

    // Common type aliases
    using GainF32 = Gain<float, 512, 44100, 2>;
    using GainF64 = Gain<double, 512, 44100, 2>;
} // namespace PlayfulTones::DspToolBox
