/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/core/buffer_view.hpp"
#include "dsp_toolbox/core/io_config.hpp"
#include "dsp_toolbox/core/param.hpp"
#include "dsp_toolbox/processors/core/processor_base.hpp"

#include <array>
#include <cstddef>

namespace PlayfulTones::DspToolbox::Modulators
{

    /**
     * @brief Clock IOConfig: no inputs, 1 CV output (trigger).
     *
     * Buffer layout: [CV out]
     * - Channel 0: Trigger output (1.0 pulse for one sample, 0.0 otherwise)
     */
    using ClockConfig = IOConfig<0, 0, 0, 1>;

    /**
     * @brief Parameter indices for Clock.
     */
    enum ClockParamIndex : std::size_t {
        kBpm = 0,
        kNumClockParams
    };

    /**
     * @brief Parameter descriptors for Clock.
     */
    inline constexpr std::array<ParamDescriptor, kNumClockParams> ClockParamDescriptors { { { "bpm", "BPM", 20.0f, 300.0f, 120.0f, "BPM" } } };

    /**
     * @brief Clock unified state with embedded parameters.
     *
     * Contains both serializable parameters (in params) and transient
     * timing state (sample counter, first sample flag).
     */
    struct ClockState
    {
        // Serializable parameters
        ParamSet<ClockParamDescriptors> params;

        // Transient state (reset on reset(), not serialized)
        std::size_t sampleCounter { 0 };
        bool firstSample { true };

        /**
         * @brief Prepare state for given sample rate.
         */
        constexpr void prepare (double sampleRate, std::size_t /*blockSize*/) noexcept
        {
            params.prepare (sampleRate);
            resetTransient();
        }

        /**
         * @brief Reset transient state only (preserves params).
         */
        constexpr void resetTransient() noexcept
        {
            sampleCounter = 0;
            firstSample = true;
        }
    };

    /**
     * @brief Clock generator that outputs trigger pulses at specified BPM.
     *
     * Generates a single-sample trigger pulse (1.0) at regular intervals
     * determined by the BPM setting. Between triggers, outputs 0.0.
     *
     * ## Timing
     * - Samples per beat = sampleRate / (BPM / 60)
     * - At 120 BPM, 48kHz: 48000 / 2 = 24000 samples between triggers
     *
     * ## CV Convention
     * - Trigger output: 1.0 for trigger sample, 0.0 otherwise
     * - First sample always triggers (to start patterns immediately)
     *
     * ## Parameter Access
     * Use index-based access:
     * - `setParam<kBpm>(140.0f)` - set tempo
     * - `getParam<kBpm>()` - get current tempo
     *
     * @tparam Spec Compile-time processor configuration
     */
    template <ConstexprSpec Spec = DefaultSpec>
    class Clock : public ProcessorBase<Clock<Spec>, ClockConfig, ClockState, Spec>
    {
    public:
        /**
         * @brief Set tempo in beats per minute.
         */
        constexpr void setBpm (float bpm) noexcept
        {
            this->template setParam<kBpm> (bpm);
        }

        /**
         * @brief Get current tempo in BPM.
         */
        [[nodiscard]] constexpr float getBpm() const noexcept
        {
            return this->template getParam<kBpm>();
        }

        /**
         * @brief Get samples per beat for current BPM.
         */
        [[nodiscard]] constexpr std::size_t getSamplesPerBeat() const noexcept
        {
            float const sampleRate = static_cast<float> (this->getSampleRate().value);
            float const beatsPerSecond = this->template getParam<kBpm>() / 60.0f;
            return static_cast<std::size_t> (sampleRate / beatsPerSecond);
        }

        template <typename SampleType>
        constexpr void processImpl (BufferView<SampleType>& buffer, ClockState& state, std::size_t sampleCount) noexcept
        {
            auto* triggerOut = buffer.getWritePointer (0);

            float const sampleRate = static_cast<float> (this->getSampleRate().value);
            float const beatsPerSecond = state.params.get (kBpm) / 60.0f;
            std::size_t const samplesPerBeat = static_cast<std::size_t> (sampleRate / beatsPerSecond);

            for (std::size_t i = 0; i < sampleCount; ++i)
            {
                bool trigger = false;

                if (state.firstSample)
                {
                    trigger = true;
                    state.firstSample = false;
                    state.sampleCounter = 0;
                }
                else if (state.sampleCounter >= samplesPerBeat)
                {
                    trigger = true;
                    state.sampleCounter = 0;
                }

                triggerOut[i] = trigger ? SampleType (1) : SampleType (0);
                ++state.sampleCounter;
            }
        }
    };

} // namespace PlayfulTones::DspToolbox::Modulators
