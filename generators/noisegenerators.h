/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once
#include "../processors/processor.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <random>

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief High-performance white noise generator following CRTP pattern
     * 
     * This processor generates white noise with compile-time optimizations and real-time safety.
     * Pre-allocates random number generator state to avoid allocations in audio callbacks.
     * 
     * @tparam SampleType The sample type (float, double)
     * @tparam BlockSize Fixed block size for processing (compile-time constant)
     * @tparam SampleRate Sample rate (compile-time constant)
     * @tparam NumChannels Number of audio channels
     */
    template <typename SampleType = float,
        size_t BlockSize = 512,
        size_t SampleRate = 44100,
        size_t NumChannels = 2>
    class WhiteNoiseGenerator : public ProcessorBase<WhiteNoiseGenerator<SampleType, BlockSize, SampleRate, NumChannels>,
                                    SampleType,
                                    BlockSize,
                                    SampleRate,
                                    NumChannels>
    {
    public:
        using Base = ProcessorBase<WhiteNoiseGenerator, SampleType, BlockSize, SampleRate, NumChannels>;
        using AudioBuffer = typename Base::AudioBuffer;
        using sample_type = SampleType;

        /**
         * @brief Constructor with initial gain setting
         * @param initialGain Initial gain factor (0.0 to 1.0)
         */
        explicit WhiteNoiseGenerator (sample_type initialGain = sample_type { 0.1 }) noexcept
            : gain_ (std::clamp (initialGain, sample_type { 0 }, sample_type { 1 })), targetGain_ (initialGain), rampSamples_ (0), currentRampSample_ (0), generator_ (std::random_device {}()), distribution_ (-sample_type { 1 }, sample_type { 1 })
        {
            targetGain_.store (initialGain, std::memory_order_relaxed);
        }

        void prepare_impl() noexcept
        {
            // Pre-warm the generator to avoid initial allocation costs
            for (int i = 0; i < 100; ++i)
            {
                distribution_ (generator_);
            }
        }

        void reset_impl() noexcept
        {
            gain_ = targetGain_.load (std::memory_order_relaxed);
            currentRampSample_ = 0;
        }

        void process_audio_impl (AudioBuffer& buffer) noexcept
        {
            const auto currentGain = gain_;
            const auto target = targetGain_.load (std::memory_order_relaxed);

            // Handle gain ramping for smooth parameter changes
            if (currentRampSample_ < rampSamples_)
            {
                processWithRamping (buffer, currentGain, target);
            }
            else
            {
                gain_ = target;
                processWithConstantGain (buffer, target);
            }
        }

        void process_control_impl() noexcept
        {
            // Control-rate processing - parameter updates happen here
        }

        /**
         * @brief Set the noise gain with smooth ramping
         * @param newGain New gain value (0.0 to 1.0)
         */
        void setGain (sample_type newGain) noexcept
        {
            const auto clampedGain = std::clamp (newGain, sample_type { 0 }, sample_type { 1 });
            targetGain_.store (clampedGain, std::memory_order_relaxed);

            // Start ramping if gain changed significantly
            const auto currentTarget = targetGain_.load (std::memory_order_relaxed);
            if (std::abs (gain_ - currentTarget) > sample_type { 0.001 })
            {
                currentRampSample_ = 0;
            }
        }

        /**
         * @brief Set gain ramp length in seconds
         * @param seconds Ramp time in seconds
         */
        void setGainRampLength (sample_type seconds) noexcept
        {
            rampSamples_ = static_cast<size_t> (seconds * SampleRate);
        }

        /**
         * @brief Get current gain value
         */
        sample_type getCurrentGain() const noexcept
        {
            return gain_;
        }

        /**
         * @brief Get target gain value
         */
        sample_type getTargetGain() const noexcept
        {
            return targetGain_.load (std::memory_order_relaxed);
        }

    private:
        void processWithConstantGain (AudioBuffer& buffer, sample_type gain) noexcept
        {
            // Generate BlockSize samples at once for cache efficiency
            alignas (32) std::array<sample_type, BlockSize> noiseBuffer;

            // Vectorizable loop - compiler can optimize this
            for (size_t i = 0; i < BlockSize; ++i)
            {
                noiseBuffer[i] = distribution_ (generator_) * gain;
            }

            // Copy to all channels - also vectorizable
            for (auto& channel : buffer)
            {
                for (size_t i = 0; i < BlockSize; ++i)
                {
                    channel[i] = noiseBuffer[i];
                }
            }
        }

        void processWithRamping (AudioBuffer& buffer, sample_type startGain, sample_type endGain) noexcept
        {
            const auto rampIncrement = (endGain - startGain) / static_cast<sample_type> (rampSamples_);

            for (size_t i = 0; i < BlockSize; ++i)
            {
                const auto sample = distribution_ (generator_);

                // Calculate ramped gain
                const auto rampedGain = startGain + (rampIncrement * currentRampSample_);
                const auto processedSample = sample * rampedGain;

                // Apply to all channels
                for (auto& channel : buffer)
                {
                    channel[i] = processedSample;
                }

                if (currentRampSample_ < rampSamples_)
                {
                    ++currentRampSample_;
                }
                else
                {
                    gain_ = endGain;
                }
            }
        }

        sample_type gain_;
        std::atomic<sample_type> targetGain_;
        size_t rampSamples_ { static_cast<size_t> (0.01 * SampleRate) }; // 10ms default ramp
        size_t currentRampSample_ { 0 };

        // Pre-allocated random number generator - real-time safe
        mutable std::mt19937 generator_;
        mutable std::uniform_real_distribution<sample_type> distribution_;
    };

    // Type aliases for common configurations
    using WhiteNoise = WhiteNoiseGenerator<float, 512, 44100, 2>;
    using WhiteNoiseMono = WhiteNoiseGenerator<float, 512, 44100, 1>;
    using WhiteNoiseHD = WhiteNoiseGenerator<double, 512, 96000, 2>;

    /**
     * @brief Simple utility function to generate white noise for testing/prototyping
     * 
     * This is a lightweight function for generating white noise without the full
     * processor infrastructure. Suitable for testing and simple use cases.
     * 
     * @param buffer Array of channel pointers to fill with noise
     * @param numChannels Number of channels to process
     * @param numFrames Number of frames per channel
     * @param gain Gain factor to apply to the noise (default 1.0f)
     */
    inline void generateWhiteNoise(float** buffer, int numChannels, int numFrames, float gain = 1.0f)
    {
        static thread_local std::mt19937 generator(std::random_device{}());
        static thread_local std::uniform_real_distribution<float> distribution(-1.0f, 1.0f);
        
        for (int frame = 0; frame < numFrames; ++frame)
        {
            const float sample = distribution(generator) * gain;
            for (int ch = 0; ch < numChannels; ++ch)
            {
                buffer[ch][frame] = sample;
            }
        }
    }

} // namespace PlayfulTones::DspToolBox
