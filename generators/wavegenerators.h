/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once
#include "../core/constants.h"
#include "../processors/processor.h"
#include <algorithm>
#include <atomic>
#include <cmath>

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief High-performance sine wave generator using CRTP
     * 
     * Thread-safe sine wave generator with smooth parameter ramping.
     * All configuration is done at compile-time for maximum performance.
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
    class SineWaveGenerator : public ProcessorBase<SineWaveGenerator<SampleType, BlockSize, SampleRate, NumChannels>,
                                  SampleType,
                                  BlockSize,
                                  SampleRate,
                                  NumChannels>
    {
    public:
        using Base = ProcessorBase<SineWaveGenerator<SampleType, BlockSize, SampleRate, NumChannels>,
            SampleType,
            BlockSize,
            SampleRate,
            NumChannels>;
        using AudioBuffer = typename Base::AudioBuffer;
        using sample_type = SampleType;

        /**
         * @brief Constructor with initial frequency and gain
         * @param initialFrequency Starting frequency in Hz
         * @param initialGain Starting gain (0.0 to 1.0)
         */
        explicit SineWaveGenerator (sample_type initialFrequency = sample_type { 440 },
            sample_type initialGain = sample_type { 0.1 }) noexcept
            : frequency_ (std::clamp (initialFrequency, sample_type { 20 }, sample_type { 20000 })),
              gain_ (std::clamp (initialGain, sample_type { 0 }, sample_type { 1 })),
              targetFrequency_ (frequency_),
              targetGain_ (gain_),
              phase_ (sample_type { 0 }),
              frequencyRampSamples_ (static_cast<sample_type> (SampleRate) * sample_type { 0.01 }), // 10ms default
              gainRampSamples_ (static_cast<sample_type> (SampleRate) * sample_type { 0.01 })
        {
            targetFrequency_.store (frequency_, std::memory_order_relaxed);
            targetGain_.store (gain_, std::memory_order_relaxed);
        }

        void prepare_impl() noexcept
        {
            // Pre-calculate constants for performance
            updatePhaseDelta();
        }

        void reset_impl() noexcept
        {
            phase_ = sample_type { 0 };
            frequency_ = targetFrequency_.load (std::memory_order_relaxed);
            gain_ = targetGain_.load (std::memory_order_relaxed);
            updatePhaseDelta();
        }

        void process_audio_impl (AudioBuffer& buffer) noexcept
        {
            const auto targetFreq = targetFrequency_.load (std::memory_order_relaxed);
            const auto targetGainValue = targetGain_.load (std::memory_order_relaxed);

            const bool frequencyRamping = std::abs (frequency_ - targetFreq) > sample_type { 0.01 };
            const bool gainRamping = std::abs (gain_ - targetGainValue) > sample_type { 0.0001 };

            if (!frequencyRamping && !gainRamping)
            {
                // Optimized path for constant parameters
                processWithConstantParams (buffer, frequency_, gain_);
            }
            else
            {
                // Ramping path for smooth parameter changes
                processWithRamping (buffer, targetFreq, targetGainValue);
            }
        }

        void process_control_impl() noexcept
        {
            // Control-rate processing happens here
            // Audio-rate processing handles parameter ramping
        }

        /**
         * @brief Set the frequency with thread-safe atomic operation
         * @param newFrequency Frequency in Hz (clamped to 20-20000 Hz)
         */
        void setFrequency (sample_type newFrequency) noexcept
        {
            const auto clampedFreq = std::clamp (newFrequency, sample_type { 20 }, sample_type { 20000 });
            targetFrequency_.store (clampedFreq, std::memory_order_relaxed);
        }

        /**
         * @brief Set the gain with thread-safe atomic operation
         * @param newGain Gain value (clamped to 0.0-1.0)
         */
        void setGain (sample_type newGain) noexcept
        {
            const auto clampedGain = std::clamp (newGain, sample_type { 0 }, sample_type { 1 });
            targetGain_.store (clampedGain, std::memory_order_relaxed);
        }

        /**
         * @brief Set frequency ramping length in seconds
         * @param seconds Ramp time in seconds
         */
        void setFrequencyRampLength (sample_type seconds) noexcept
        {
            frequencyRampSamples_ = std::max (sample_type { 1 }, seconds * static_cast<sample_type> (SampleRate));
        }

        /**
         * @brief Set gain ramping length in seconds
         * @param seconds Ramp time in seconds
         */
        void setGainRampLength (sample_type seconds) noexcept
        {
            gainRampSamples_ = std::max (sample_type { 1 }, seconds * static_cast<sample_type> (SampleRate));
        }

        /**
         * @brief Get current frequency
         */
        sample_type getCurrentFrequency() const noexcept
        {
            return frequency_;
        }

        /**
         * @brief Get target frequency
         */
        sample_type getTargetFrequency() const noexcept
        {
            return targetFrequency_.load (std::memory_order_relaxed);
        }

        /**
         * @brief Get current gain
         */
        sample_type getCurrentGain() const noexcept
        {
            return gain_;
        }

        /**
         * @brief Get target gain
         */
        sample_type getTargetGain() const noexcept
        {
            return targetGain_.load (std::memory_order_relaxed);
        }

        /**
         * @brief Check if frequency is currently ramping
         */
        bool isFrequencyRamping() const noexcept
        {
            const auto target = targetFrequency_.load (std::memory_order_relaxed);
            return std::abs (frequency_ - target) > sample_type { 0.01 };
        }

        /**
         * @brief Check if gain is currently ramping
         */
        bool isGainRamping() const noexcept
        {
            const auto target = targetGain_.load (std::memory_order_relaxed);
            return std::abs (gain_ - target) > sample_type { 0.0001 };
        }

    private:
        // Current parameter values (audio thread)
        sample_type frequency_;
        sample_type gain_;
        sample_type phase_;
        sample_type phaseDelta_;

        // Target parameter values (thread-safe)
        std::atomic<sample_type> targetFrequency_;
        std::atomic<sample_type> targetGain_;

        // Ramping configuration
        sample_type frequencyRampSamples_;
        sample_type gainRampSamples_;

        static constexpr sample_type twoPi = static_cast<sample_type> (Constants::twoPi);

        void updatePhaseDelta() noexcept
        {
            phaseDelta_ = twoPi * frequency_ / static_cast<sample_type> (SampleRate);
        }

        void processWithConstantParams (AudioBuffer& buffer, sample_type freq, sample_type gainValue) noexcept
        {
            markUsed (freq);
            // Cache-friendly processing with constant parameters
            for (size_t i = 0; i < BlockSize; ++i)
            {
                const auto sample = std::sin (phase_) * gainValue;
                phase_ = std::fmod (phase_ + phaseDelta_, twoPi);

                // Apply to all channels
                for (auto& channel : buffer)
                {
                    channel[i] = sample;
                }
            }
        }

        void processWithRamping (AudioBuffer& buffer, sample_type targetFreq, sample_type targetGainValue) noexcept
        {
            // Calculate ramp increments
            const auto freqIncrement = (targetFreq - frequency_) / frequencyRampSamples_;
            const auto gainIncrement = (targetGainValue - gain_) / gainRampSamples_;

            for (size_t i = 0; i < BlockSize; ++i)
            {
                // Update parameters with ramping
                if (std::abs (frequency_ - targetFreq) > sample_type { 0.01 })
                {
                    frequency_ += freqIncrement;
                    updatePhaseDelta();
                }

                if (std::abs (gain_ - targetGainValue) > sample_type { 0.0001 })
                {
                    gain_ += gainIncrement;
                }

                // Generate sample
                const auto sample = std::sin (phase_) * gain_;
                phase_ = std::fmod (phase_ + phaseDelta_, twoPi);

                // Apply to all channels
                for (auto& channel : buffer)
                {
                    channel[i] = sample;
                }
            }
        }
    };

} // namespace PlayfulTones::DspToolBox
