/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once

#include "../core/ring_buffer.h"
#include "../processors/processor.h"
#include <cmath>
#include <algorithm>
#include <atomic>
#include <array>

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief High-performance multi-channel delay processor using CRTP
     * 
     * This delay processor supports delay times up to 2 seconds, with controls for 
     * feedback level and dry/wet mix. Uses compile-time optimization and separate
     * delay lines for each channel.
     * 
     * @tparam SampleType The sample type (float, double)
     * @tparam BlockSize Fixed block size for processing
     * @tparam SampleRate Sample rate (compile-time constant)
     * @tparam NumChannels Number of audio channels
     */
    template<typename SampleType = float, 
             size_t BlockSize = 512, 
             size_t SampleRate = 44100,
             size_t NumChannels = 2>
    class Delay : public ProcessorBase<Delay<SampleType, BlockSize, SampleRate, NumChannels>,
                                       SampleType, BlockSize, SampleRate, NumChannels>
    {
    public:
        using Base = ProcessorBase<Delay, SampleType, BlockSize, SampleRate, NumChannels>;
        using AudioBuffer = typename Base::AudioBuffer;
        using sample_type = SampleType;
        
        static constexpr sample_type MaxDelayTimeMs = sample_type{2000}; // Maximum delay time in milliseconds
        
        /**
         * @brief Construct a new Delay processor
         * 
         * @param initialDelayMs Initial delay time in milliseconds
         * @param initialFeedback Initial feedback amount (0.0 to 1.0)
         * @param initialMix Initial dry/wet mix (0.0 = dry, 1.0 = wet)
         */
        Delay(sample_type initialDelayMs = sample_type{500}, 
              sample_type initialFeedback = sample_type{0.5}, 
              sample_type initialMix = sample_type{0.5})
        {
            // Clamp initial values to valid ranges and store atomically
            delayMs_.store(std::clamp(initialDelayMs, sample_type{0}, MaxDelayTimeMs), std::memory_order_relaxed);
            feedback_.store(std::clamp(initialFeedback, sample_type{0}, sample_type{1}), std::memory_order_relaxed);
            mix_.store(std::clamp(initialMix, sample_type{0}, sample_type{1}), std::memory_order_relaxed);
            
            // Initialize delay buffers for each channel
            for (size_t ch = 0; ch < NumChannels; ++ch)
            {
                delayBuffers_[ch] = RingBuffer<sample_type>(1); // Will be resized in prepare()
            }
        }

        void prepare_impl() noexcept
        {
            // Calculate the maximum number of samples needed for 2 seconds of delay
            constexpr size_t maxDelaySamples = static_cast<size_t>(std::ceil(2.0 * Base::sample_rate)) + 1;

            // Resize and clear all delay buffers
            for (size_t ch = 0; ch < NumChannels; ++ch)
            {
                delayBuffers_[ch].resize(maxDelaySamples);
                delayBuffers_[ch].clear();
            }

            updateDelaySamples();
        }

        void process_audio_impl(AudioBuffer& buffer) noexcept
        {
            constexpr size_t numFrames = Base::block_size;
            constexpr size_t numChannels = Base::num_channels;

            const sample_type currentFeedback = feedback_.load(std::memory_order_relaxed);
            const sample_type currentMix = mix_.load(std::memory_order_relaxed);
            const size_t currentDelaySamples = delaySamples_.load(std::memory_order_relaxed);

            // Precompute mix coefficients for better performance
            const sample_type dryGain = sample_type{1} - currentMix;
            const sample_type wetGain = currentMix;

            for (size_t ch = 0; ch < numChannels; ++ch)
            {
                auto& delayBuffer = delayBuffers_[ch];
                sample_type* channelData = buffer[ch].data();
                
                for (size_t i = 0; i < numFrames; ++i)
                {
                    const sample_type inputSample = channelData[i];
                    sample_type delaySample = sample_type{0};
                    
                    // Read the delayed sample
                    delayBuffer.peekAt(delaySample, currentDelaySamples);

                    // Prevent denormals in delay sample
                    constexpr sample_type kDenormalEpsilon = sample_type{1e-30};
                    if (std::abs(delaySample) < kDenormalEpsilon)
                        delaySample = sample_type{0};

                    // Calculate the output sample with dry/wet mix (branchless)
                    channelData[i] = dryGain * inputSample + wetGain * delaySample;

                    // Calculate feedback sample with denormal protection
                    sample_type feedbackSample = inputSample + currentFeedback * delaySample;
                    if (std::abs(feedbackSample) < kDenormalEpsilon)
                        feedbackSample = sample_type{0};

                    // Discard the oldest sample if the buffer is full
                    if (delayBuffer.isFull()) [[unlikely]]
                    {
                        delayBuffer.discard(1);
                    }

                    // Write input + feedback to the delay buffer
                    delayBuffer.push(feedbackSample);
                }
            }
        }

        void process_control_impl() noexcept
        {
            // Update delay samples if delay time has changed
            updateDelaySamples();
        }

        void reset_impl() noexcept
        {
            for (size_t ch = 0; ch < NumChannels; ++ch)
            {
                delayBuffers_[ch].clear();
            }
        }

        /**
         * @brief Set the delay time in milliseconds (thread-safe)
         * 
         * @param delayTimeMs The new delay time in milliseconds (0 to 2000)
         */
        void setDelayTime(sample_type delayTimeMs) noexcept
        {
            const sample_type currentDelayMs = delayMs_.load(std::memory_order_relaxed);
            if (std::abs(delayTimeMs - currentDelayMs) < sample_type{0.001})
                return; // No significant change
                
            // Clamp to valid range
            delayTimeMs = std::clamp(delayTimeMs, sample_type{0}, MaxDelayTimeMs);
            delayMs_.store(delayTimeMs, std::memory_order_relaxed);
        }

        /**
         * @brief Set the feedback amount (thread-safe)
         * 
         * @param feedbackAmount The new feedback amount (0.0 to 1.0)
         */
        void setFeedback(sample_type feedbackAmount) noexcept
        {
            const sample_type currentFeedback = feedback_.load(std::memory_order_relaxed);
            if (std::abs(feedbackAmount - currentFeedback) < sample_type{0.001})
                return; // No significant change
                
            // Clamp to valid range
            feedback_.store(std::clamp(feedbackAmount, sample_type{0}, sample_type{1}), 
                           std::memory_order_relaxed);
        }

        /**
         * @brief Set the dry/wet mix (thread-safe)
         * 
         * @param mixAmount The new mix amount (0.0 = dry, 1.0 = wet)
         */
        void setMix(sample_type mixAmount) noexcept
        {
            const sample_type currentMix = mix_.load(std::memory_order_relaxed);
            if (std::abs(mixAmount - currentMix) < sample_type{0.001})
                return; // No significant change
                
            // Clamp to valid range
            mix_.store(std::clamp(mixAmount, sample_type{0}, sample_type{1}), 
                      std::memory_order_relaxed);
        }

        /**
         * @brief Get the current delay time in milliseconds
         */
        sample_type getDelayTime() const noexcept
        {
            return delayMs_.load(std::memory_order_relaxed);
        }

        /**
         * @brief Get the current feedback amount
         */
        sample_type getFeedback() const noexcept
        {
            return feedback_.load(std::memory_order_relaxed);
        }

        /**
         * @brief Get the current dry/wet mix
         */
        sample_type getMix() const noexcept
        {
            return mix_.load(std::memory_order_relaxed);
        }

    private:
        void updateDelaySamples() noexcept
        {
            const sample_type currentDelayMs = delayMs_.load(std::memory_order_relaxed);
            const size_t newDelaySamples = static_cast<size_t>(
                std::max(sample_type{0}, (currentDelayMs / sample_type{1000}) * Base::sample_rate));
            delaySamples_.store(newDelaySamples, std::memory_order_relaxed);
        }

        // Thread-safe parameter storage
        std::atomic<sample_type> delayMs_{sample_type{500}}; // Delay time in milliseconds
        std::atomic<sample_type> feedback_{sample_type{0.5}}; // Feedback amount (0.0 to 1.0)
        std::atomic<sample_type> mix_{sample_type{0.5}}; // Dry/wet mix (0.0 = dry, 1.0 = wet)
        std::atomic<size_t> delaySamples_{0}; // Delay time in samples

        // Per-channel delay buffers
        std::array<RingBuffer<sample_type>, NumChannels> delayBuffers_;
    };
    
    // Common type aliases
    using DelayF32 = Delay<float, 512, 44100, 2>;
    using DelayF64 = Delay<double, 512, 44100, 2>;
} // namespace PlayfulTones::DspToolBox
