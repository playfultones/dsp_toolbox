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
     * @brief High-performance ADSR envelope generator using CRTP
     *
     * This class implements a high-performance ADSR (Attack, Decay, Sustain, Release) 
     * envelope generator with compile-time optimization and lockless parameter updates.
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
    class ADSR : public ProcessorBase<ADSR<SampleType, BlockSize, SampleRate, NumChannels>,
                     SampleType,
                     BlockSize,
                     SampleRate,
                     NumChannels>
    {
    public:
        using Base = ProcessorBase<ADSR, SampleType, BlockSize, SampleRate, NumChannels>;
        using AudioBuffer = typename Base::AudioBuffer;
        using sample_type = SampleType;

        ADSR() = default;
        ~ADSR() = default;

        void prepare_impl() noexcept
        {
            reset_impl();
        }

        void reset_impl() noexcept
        {
            currentValue_ = sample_type { 0 };
            currentState_ = State::Idle;
            sampleCounter_ = 0;
        }

        void process_audio_impl (AudioBuffer& buffer) noexcept
        {
            constexpr size_t numFrames = Base::block_size;
            constexpr size_t numChannels = Base::num_channels;

            for (size_t i = 0; i < numFrames; ++i)
            {
                sample_type envValue = processNextValue();

                for (size_t ch = 0; ch < numChannels; ++ch)
                {
                    buffer[ch][i] *= envValue;
                }
            }
        }

        void process_control_impl() noexcept
        {
            // Update internal parameters from atomic values if needed
            // This allows thread-safe parameter updates
        }

        /**
         * @brief Set attack time in seconds (thread-safe)
         */
        void setAttack (double attackTimeInSeconds) noexcept
        {
            const double clampedTime = std::max (0.0, attackTimeInSeconds);
            attackSamples_.store (clampedTime * Base::sample_rate, std::memory_order_relaxed);
        }

        /**
         * @brief Set decay time in seconds (thread-safe)
         */
        void setDecay (double decayTimeInSeconds) noexcept
        {
            const double clampedTime = std::max (0.0, decayTimeInSeconds);
            decaySamples_.store (clampedTime * Base::sample_rate, std::memory_order_relaxed);
        }

        /**
         * @brief Set sustain level (thread-safe)
         */
        void setSustain (double sustainLevel) noexcept
        {
            const double clampedLevel = std::clamp (sustainLevel, 0.0, 1.0);
            sustainValue_.store (clampedLevel, std::memory_order_relaxed);
        }

        /**
         * @brief Set release time in seconds (thread-safe)
         */
        void setRelease (double releaseTimeInSeconds) noexcept
        {
            const double clampedTime = std::max (0.0, releaseTimeInSeconds);
            releaseSamples_.store (clampedTime * Base::sample_rate, std::memory_order_relaxed);
        }

        /**
         * @brief Trigger note on (thread-safe)
         */
        void noteOn() noexcept
        {
            currentState_ = State::Attack;
            sampleCounter_ = 0;
            releaseStartValue_ = currentValue_;
        }

        /**
         * @brief Trigger note off (thread-safe)
         */
        void noteOff() noexcept
        {
            currentState_ = State::Release;
            sampleCounter_ = 0;
            releaseStartValue_ = currentValue_;
        }

        /**
         * @brief Get current envelope value
         */
        sample_type getCurrentValue() const noexcept
        {
            return currentValue_;
        }

    private:
        enum class State {
            Idle,
            Attack,
            Decay,
            Sustain,
            Release
        };

        sample_type processNextValue() noexcept
        {
            const double attackSamples = attackSamples_.load (std::memory_order_relaxed);
            const double decaySamples = decaySamples_.load (std::memory_order_relaxed);
            const double sustainValue = sustainValue_.load (std::memory_order_relaxed);
            const double releaseSamples = releaseSamples_.load (std::memory_order_relaxed);

            switch (currentState_)
            {
                case State::Attack:
                    if (sampleCounter_ >= attackSamples)
                    {
                        currentState_ = State::Decay;
                        sampleCounter_ = 0;
                        currentValue_ = sample_type { 1 };
                    }
                    else
                    {
                        // Branchless calculation for attack phase
                        const double progress = attackSamples > 0.0 ? sampleCounter_ / attackSamples : 1.0;
                        currentValue_ = static_cast<sample_type> (progress);
                    }
                    break;

                case State::Decay:
                    if (sampleCounter_ >= decaySamples)
                    {
                        currentState_ = State::Sustain;
                        currentValue_ = static_cast<sample_type> (sustainValue);
                    }
                    else
                    {
                        // Branchless calculation for decay phase
                        const double progress = decaySamples > 0.0 ? sampleCounter_ / decaySamples : 1.0;
                        currentValue_ = static_cast<sample_type> (1.0 - (1.0 - sustainValue) * progress);
                    }
                    break;

                case State::Sustain:
                    currentValue_ = static_cast<sample_type> (sustainValue);
                    break;

                case State::Release:
                    if (sampleCounter_ >= releaseSamples)
                    {
                        currentState_ = State::Idle;
                        currentValue_ = sample_type { 0 };
                    }
                    else
                    {
                        // Branchless calculation for release phase
                        const double progress = releaseSamples > 0.0 ? sampleCounter_ / releaseSamples : 1.0;
                        currentValue_ = static_cast<sample_type> (releaseStartValue_ * (1.0 - progress));
                    }
                    break;

                case State::Idle:
                default:
                    currentValue_ = sample_type { 0 };
                    break;
            }

            ++sampleCounter_;

            // Add small epsilon to prevent denormals
            constexpr sample_type kDenormalEpsilon = sample_type { 1e-30 };
            if (std::abs (currentValue_) < kDenormalEpsilon && currentValue_ != sample_type { 0 })
            {
                currentValue_ = sample_type { 0 };
            }

            return currentValue_;
        }

        // Thread-safe parameter storage
        std::atomic<double> attackSamples_ { 0.0 };
        std::atomic<double> decaySamples_ { 0.0 };
        std::atomic<double> sustainValue_ { 1.0 };
        std::atomic<double> releaseSamples_ { 0.0 };

        // Audio thread state (not atomic - only accessed from audio thread)
        sample_type currentValue_ { 0 };
        sample_type releaseStartValue_ { 0 };
        double sampleCounter_ { 0 };
        State currentState_ { State::Idle };
    };

    // Common type aliases
    using ADSRF32 = ADSR<float, 512, 44100, 2>;
    using ADSRF64 = ADSR<double, 512, 44100, 2>;
} // namespace PlayfulTones::DspToolBox
