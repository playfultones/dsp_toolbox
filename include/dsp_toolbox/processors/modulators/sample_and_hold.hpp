/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/core/buffer_view.hpp"
#include "dsp_toolbox/core/io_config.hpp"
#include "dsp_toolbox/processors/core/processor_base.hpp"

#include <cstddef>

namespace PlayfulTones::DspToolbox::Modulators
{

    /**
     * @brief SampleAndHold IOConfig: 2 CV in (signal + trigger), 1 CV out.
     *
     * Buffer layout:
     * - Channel 0: Signal input / Held value output (shared, read-before-write)
     * - Channel 1: Trigger input (read only)
     *
     * On a rising edge of the trigger (crossing 0.5 threshold),
     * the current signal value is captured and held until the next trigger.
     */
    using SampleAndHoldConfig = IOConfig<0, 0, 2, 1>;

    /**
     * @brief SampleAndHold state.
     */
    struct SampleAndHoldState
    {
        float heldValue { 0.0f };
        float previousTrigger { 0.0f };

        /**
         * @brief Prepare state for given sample rate.
         */
        constexpr void prepare (double /*sampleRate*/, std::size_t /*blockSize*/) noexcept
        {
            resetTransient();
        }

        /**
         * @brief Reset transient state only.
         */
        constexpr void resetTransient() noexcept
        {
            heldValue = 0.0f;
            previousTrigger = 0.0f;
        }
    };

    /**
     * @brief Sample-and-hold processor with trigger-based capture.
     *
     * Samples the input signal on rising edge of trigger and holds the
     * value until the next trigger event. Useful for quantizing continuous
     * CPU signals into stepped values.
     *
     * ## CV Convention
     * - Trigger input: > 0.5 is considered high, rising edge captures
     * - Signal input: any range
     * - Output: held value
     *
     * @tparam Spec Compile-time processor configuration
     */
    template <ConstexprSpec Spec = DefaultSpec>
    class SampleAndHold : public ProcessorBase<SampleAndHold<Spec>, SampleAndHoldConfig, SampleAndHoldState, Spec>
    {
    public:
        template <typename SampleType>
        constexpr void processImpl (BufferView<SampleType>& buffer, SampleAndHoldState& state, std::size_t sampleCount) noexcept
        {
            auto const* signal = buffer.getReadPointer (0);
            auto const* trigger = buffer.getReadPointer (1);
            auto* out = buffer.getWritePointer (0);

            for (std::size_t i = 0; i < sampleCount; ++i)
            {
                float const trig = static_cast<float> (trigger[i]);
                if (trig > 0.5f && state.previousTrigger <= 0.5f)
                    state.heldValue = static_cast<float> (signal[i]);

                state.previousTrigger = trig;
                out[i] = static_cast<SampleType> (state.heldValue);
            }
        }
    };

} // namespace PlayfulTones::DspToolbox::Modulators
