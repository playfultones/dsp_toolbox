/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/core/buffer_view.hpp"
#include "dsp_toolbox/core/io_config.hpp"
#include "dsp_toolbox/processors/core/processor_base.hpp"

#include <array>
#include <cstddef>

namespace PlayfulTones::DspToolbox::Processors
{

    /**
     * @brief Mixer state: per-channel gain values.
     *
     * Unified state for Mixer - gains are preserved on reset(),
     * only cleared on resetAll().
     *
     * @tparam NumInputs Number of input channels to mix
     */
    template <std::size_t NumInputs>
    struct MixerState
    {
        std::array<float, NumInputs> gains {};

        constexpr MixerState() noexcept
        {
            for (std::size_t i = 0; i < NumInputs; ++i)
            {
                gains[i] = 1.0f;
            }
        }

        /**
         * @brief Prepare state for given sample rate.
         */
        constexpr void prepare (double /*sampleRate*/, std::size_t /*blockSize*/) noexcept
        {
            // Gains don't need sample-rate preparation
        }

        /**
         * @brief Reset transient state only (preserves gains).
         *
         * Mixer has no transient state, so this is a no-op.
         * Gains are considered parameters, not transient state.
         */
        constexpr void resetTransient() noexcept
        {
            // No transient state to reset - gains are params
        }
    };

    /**
     * @brief Multi-channel audio mixer.
     *
     * Sums multiple audio inputs into a single output with per-channel
     * gain control. Essential for combining multiple sound sources.
     *
     * ## Buffer Layout
     * - Channels [0, NumInputs): Audio inputs (read)
     * - Channel 0: Audio output (write, overwrites first input)
     *
     * Note: Uses channel 0 for both first input and output (in-place).
     * The mixer reads all inputs, sums them, and writes to channel 0.
     *
     * ## Gain Control
     * - Default gain per channel: 1.0 (unity)
     * - Use setGain() to attenuate/boost individual channels
     * - Output = sum(input[i] * gain[i]) for i in 0..NumInputs
     *
     * ## Example Usage
     * @code
     * Mixer<2, MySpec> mixer;  // 2-input mixer
     * mixer.setGain(0, 0.8f);  // First input at 80%
     * mixer.setGain(1, 0.5f);  // Second input at 50%
     * mixer.process(buffer, numSamples);
     * // Output in channel 0 = input0*0.8 + input1*0.5
     * @endcode
     *
     * @tparam NumInputs Number of audio inputs to mix (must be >= 2)
     * @tparam Spec Compile-time processor configuration
     */
    template <std::size_t NumInputs, ConstexprSpec Spec = DefaultSpec>
    class Mixer : public ProcessorBase<Mixer<NumInputs, Spec>, IOConfig<NumInputs, 1, 0, 0>, MixerState<NumInputs>, Spec>
    {
        static_assert (NumInputs >= 2, "Mixer requires at least 2 inputs");

    public:
        /**
         * @brief Set gain for a specific input channel.
         *
         * @param channel Input channel index (0 to NumInputs-1)
         * @param gain Gain value (0.0 = silent, 1.0 = unity)
         */
        constexpr void setGain (std::size_t channel, float gain) noexcept
        {
            if (channel < NumInputs)
            {
                this->state_.gains[channel] = gain;
            }
        }

        /**
         * @brief Get gain for a specific input channel.
         *
         * @param channel Input channel index
         * @return Gain value for that channel
         */
        [[nodiscard]] constexpr float getGain (std::size_t channel) const noexcept
        {
            if (channel < NumInputs)
            {
                return this->state_.gains[channel];
            }
            return 0.0f;
        }

        /**
         * @brief Set all gains to the same value.
         *
         * @param gain Gain value for all channels
         */
        constexpr void setAllGains (float gain) noexcept
        {
            for (std::size_t i = 0; i < NumInputs; ++i)
            {
                this->state_.gains[i] = gain;
            }
        }

        template <typename SampleType>
        constexpr void processImpl (BufferView<SampleType>& buffer, MixerState<NumInputs>& state, std::size_t sampleCount) noexcept
        {
            auto* output = buffer.getWritePointer (0);

            for (std::size_t i = 0; i < sampleCount; ++i)
            {
                float sum = 0.0f;

                for (std::size_t ch = 0; ch < NumInputs; ++ch)
                {
                    auto const* input = buffer.getReadPointer (ch);
                    sum += static_cast<float> (input[i]) * state.gains[ch];
                }

                output[i] = static_cast<SampleType> (sum);
            }
        }
    };

} // namespace PlayfulTones::DspToolbox::Processors
