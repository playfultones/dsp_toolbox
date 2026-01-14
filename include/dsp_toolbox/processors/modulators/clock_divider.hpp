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
     * @brief ClockDivider IOConfig: 1 CV in (trigger), 1 CV out (divided trigger).
     *
     * Buffer layout: [CV I/O]
     * - Channel 0: Trigger input / Divided trigger output (shared, read-before-write)
     */
    using ClockDividerConfig = IOConfig<0, 0, 1, 1>;

    /**
     * @brief Parameter indices for ClockDivider.
     */
    enum ClockDividerParamIndex : std::size_t {
        kDivisor = 0,
        kNumClockDividerParams
    };

    /**
     * @brief Parameter descriptors for ClockDivider.
     */
    inline constexpr std::array<ParamDescriptor, kNumClockDividerParams> ClockDividerParamDescriptors { { { "divisor", "Divisor", 1.0f, 32.0f, 2.0f, "" } } };

    /**
     * @brief ClockDivider unified state with embedded parameters.
     *
     * Contains both serializable parameters (in params) and transient
     * trigger counting state (counter, edge detection).
     */
    struct ClockDividerState
    {
        // Serializable parameters
        ParamSet<ClockDividerParamDescriptors> params;

        // Transient state (reset on reset(), not serialized)
        std::size_t counter { 0 };
        float prevTrigger { 0.0f };

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
            counter = 0;
            prevTrigger = 0.0f;
        }
    };

    /**
     * @brief Divides incoming trigger pulses by a configurable ratio.
     *
     * Outputs a trigger every N input triggers, where N is the divisor.
     * Useful for creating polyrhythms and timing variations.
     *
     * ## Behavior
     * - Detects rising edge on input trigger (> 0.5)
     * - Counts input triggers
     * - Outputs trigger when count reaches divisor
     * - Counter resets after output
     *
     * ## CV Convention
     * - Trigger input: > 0.5 is considered high
     * - Trigger output: 1.0 for trigger sample, 0.0 otherwise
     *
     * ## Parameter Access
     * Use index-based access:
     * - `setParam<kDivisor>(4.0f)` - divide by 4
     * - `getParam<kDivisor>()` - get current divisor
     *
     * ## Example
     * With divisor = 2:
     * - Input:  [1,0,0,1,0,0,1,0,0,1,0,0]
     * - Output: [1,0,0,0,0,0,1,0,0,0,0,0]
     *
     * @tparam Spec Compile-time processor configuration
     */
    template <ConstexprSpec Spec = DefaultSpec>
    class ClockDivider : public ProcessorBase<ClockDivider<Spec>, ClockDividerConfig, ClockDividerState, Spec>
    {
    public:
        /**
         * @brief Set division ratio.
         *
         * @param divisor Number of input triggers per output trigger (must be >= 1)
         */
        constexpr void setDivisor (std::size_t divisor) noexcept
        {
            this->template setParam<kDivisor> (static_cast<float> ((divisor >= 1) ? divisor : 1));
        }

        /**
         * @brief Get current division ratio.
         */
        [[nodiscard]] constexpr std::size_t getDivisor() const noexcept
        {
            return static_cast<std::size_t> (this->template getParam<kDivisor>());
        }

        /**
         * @brief Get current trigger counter.
         */
        [[nodiscard]] constexpr std::size_t getCounter() const noexcept
        {
            return this->state_.counter;
        }

        template <typename SampleType>
        constexpr void processImpl (BufferView<SampleType>& buffer, ClockDividerState& state, std::size_t sampleCount) noexcept
        {
            auto* cv = buffer.getWritePointer (0);
            std::size_t const divisor = static_cast<std::size_t> (state.params.get (kDivisor));

            for (std::size_t i = 0; i < sampleCount; ++i)
            {
                float const triggerIn = static_cast<float> (cv[i]);

                bool const risingEdge = (triggerIn > 0.5f) && (state.prevTrigger <= 0.5f);
                state.prevTrigger = triggerIn;

                bool triggerOut = false;

                if (risingEdge)
                {
                    ++state.counter;
                    if (state.counter >= divisor)
                    {
                        triggerOut = true;
                        state.counter = 0;
                    }
                }

                cv[i] = triggerOut ? SampleType (1) : SampleType (0);
            }
        }
    };

} // namespace PlayfulTones::DspToolbox::Modulators
