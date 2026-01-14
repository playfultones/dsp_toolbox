/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/core/buffer_view.hpp"
#include "dsp_toolbox/core/conversions.hpp"
#include "dsp_toolbox/core/crv.hpp"
#include "dsp_toolbox/core/io_config.hpp"
#include "dsp_toolbox/core/param.hpp"
#include "dsp_toolbox/processors/core/processor_base.hpp"
#include "dsp_toolbox/simd/simd.hpp"

#include <array>
#include <cstddef>
#include <type_traits>

namespace PlayfulTones::DspToolbox::Processors
{

    /**
     * @brief Gain IOConfig: 1 audio in, 1 audio out, no CV.
     *
     * Buffer layout: [Audio I/O]
     * - Channel 0: Audio input/output (in-place processing)
     */
    using GainConfig = IOConfig<1, 1, 0, 0>;

    /**
     * @brief Parameter indices for Gain.
     */
    enum GainParamIndex : std::size_t {
        kGain = 0,
        kNumGainParams
    };

    /**
     * @brief Parameter descriptors for Gain.
     */
    inline constexpr std::array<ParamDescriptor, kNumGainParams> GainParamDescriptors { { { .id = "gain_db", .name = "Gain", .minValue = -60.0f, .maxValue = 12.0f, .defaultValue = 0.0f, .unit = "dB" } } };

    /**
     * @brief Gain unified state with embedded parameters.
     *
     * Contains serializable parameters only (no transient state).
     */
    struct GainState
    {
        // Serializable parameters
        ParamSet<GainParamDescriptors> params;

        // Linear gain smoothing (faster than per-sample dB conversion)
        float linearGainTarget { 1.0f };
        float linearGainCurrent { 1.0f };
        float linearGainStep { 0.0f };
        std::size_t smoothingStepsRemaining { 0 };
        std::size_t smoothingSteps { 480 }; // Default: 10ms at 48kHz

        // Dirty flag: set when gain changes, cleared after param ticking
        bool paramsNeedTick { false };

        /**
         * @brief Prepare state for given sample rate.
         */
        constexpr void prepare (double sampleRate, std::size_t /*blockSize*/) noexcept
        {
            params.prepare (sampleRate);
            // 10ms smoothing time
            smoothingSteps = static_cast<std::size_t> (sampleRate * 0.01);
        }

        /**
         * @brief Reset transient state only.
         */
        constexpr void resetTransient() noexcept
        {
            linearGainCurrent = linearGainTarget;
            smoothingStepsRemaining = 0;
            paramsNeedTick = false;
        }
    };

    /**
     * @brief Simple gain/volume control with dB parameter.
     *
     * Multiplies audio by a linear gain derived from a dB parameter.
     * Parameter changes are smoothed for click-free automation.
     *
     * ## Parameter Range
     * - -60 dB to +12 dB (unity = 0 dB)
     * - 10ms smoothing (default)
     *
     * ## Buffer Layout
     * - Channel 0: Audio input (read), Audio output (write)
     *
     * ## Parameter Access
     * Use index-based access:
     * - `setParam<kGain>(-6.0f)` - set gain to -6 dB
     * - `getParam<kGain>()` - get current gain in dB
     *
     * Or use convenience methods:
     * - `setGainDb(-6.0f)` - set gain to -6 dB
     * - `setGain(Decibels<float>{-6.0f})` - using strong type
     *
     * ## Stereo Usage
     * For stereo processing, use StereoExpander:
     * @code
     * using StereoGain = StereoExpander<Gain::Processor, StereoEffectConfig, Spec>;
     * @endcode
     *
     * @tparam Spec Compile-time processor configuration
     */
    template <ConstexprSpec Spec = DefaultSpec>
    class Gain : public ProcessorBase<Gain<Spec>, GainConfig, GainState, Spec>
    {
    public:
        /**
         * @brief Processor template alias for StereoExpander.
         */
        template <ConstexprSpec S>
        using Processor = Gain<S>;

        /**
         * @brief Set gain in dB (smoothed transition).
         *
         * Uses parameter smoothing to prevent clicks when gain changes.
         * Converts dB to linear once and smooths in linear domain for efficiency.
         *
         * @param dB Gain value in decibels (-60 to +12)
         */
        constexpr void setGainDb (float dB) noexcept
        {
            this->state_.params.template setTarget<kGain> (dB);

            // Convert to linear ONCE, then smooth in linear domain
            float const newLinearTarget = toLinearGain (Decibels<float> { dB }).value();
            if (newLinearTarget != this->state_.linearGainTarget)
            {
                this->state_.paramsNeedTick = true;
                this->state_.linearGainTarget = newLinearTarget;
                if (this->state_.smoothingSteps > 0)
                {
                    this->state_.linearGainStep = (newLinearTarget - this->state_.linearGainCurrent) / static_cast<float> (this->state_.smoothingSteps);
                    this->state_.smoothingStepsRemaining = this->state_.smoothingSteps;
                }
                else
                {
                    this->state_.linearGainCurrent = newLinearTarget;
                }
            }
        }

        /**
         * @brief Get current gain in dB.
         *
         * @return Current gain value in decibels
         */
        [[nodiscard]] constexpr float getGainDb() const noexcept
        {
            return this->template getParam<kGain>();
        }

        /**
         * @brief Set gain using strong type.
         *
         * @param dB Gain value as Decibels strong type
         */
        constexpr void setGain (Decibels<float> dB) noexcept
        {
            setGainDb (dB.value());
        }

        /**
         * @brief Get current gain as strong type.
         *
         * @return Current gain as Decibels
         */
        [[nodiscard]] constexpr Decibels<float> getGain() const noexcept
        {
            return Decibels<float> { getGainDb() };
        }

        /**
         * @brief Process audio buffer.
         *
         * Applies gain with linear-domain smoothing for efficiency.
         * Optimizations:
         * - Value hoisting for gain=0 (silence) and gain=1 (pass-through)
         * - 2x loop unrolling for better instruction-level parallelism
         * - SIMD-accelerated gain ramping during smoothing
         */
        template <typename SampleType>
        constexpr void processImpl (BufferView<SampleType>& buffer, GainState& state, std::size_t sampleCount) noexcept
        {
            auto* audio = buffer.getWritePointer (0);

            // Only tick params if they changed (dirty flag optimization)
            if (state.paramsNeedTick)
            {
                for (std::size_t i = 0; i < sampleCount; ++i)
                    (void) state.params.template tick<kGain>();
                state.paramsNeedTick = false;
            }

            if (state.smoothingStepsRemaining == 0)
            {
                // Static gain - use hoisting to optimize special cases
                hoisted<LinearGainHoistSpec<float>> (state.linearGainCurrent, [&] (auto gainCRV) {
                    if constexpr (isConstantZero<decltype (gainCRV)>)
                    {
                        // gain=0: output silence
                        if constexpr (std::is_same_v<SampleType, float>)
                        {
                            simd::clear (audio, sampleCount);
                        }
                        else
                        {
                            for (std::size_t i = 0; i < sampleCount; ++i)
                                audio[i] = SampleType { 0 };
                        }
                    }
                    else if constexpr (isConstantOne<decltype (gainCRV)>)
                    {
                        // gain=1: pass-through, no processing needed
                    }
                    else
                    {
                        // Normal gain - optimized SIMD multiply with 2x unrolling
                        if constexpr (std::is_same_v<SampleType, float>)
                        {
                            simd::multiplyUnrolled (audio, sampleCount, gainCRV.get());
                        }
                        else
                        {
                            float const gain = gainCRV.get();
                            for (std::size_t i = 0; i < sampleCount; ++i)
                                audio[i] = static_cast<SampleType> (static_cast<float> (audio[i]) * gain);
                        }
                    }
                });
            }
            else
            {
                // Ramping gain - SIMD-accelerated smoothing
                if constexpr (std::is_same_v<SampleType, float>)
                {
                    simd::multiplyRamp (audio, sampleCount, state.linearGainCurrent, state.linearGainStep, state.smoothingStepsRemaining, state.linearGainTarget);
                }
                else
                {
                    for (std::size_t i = 0; i < sampleCount; ++i)
                    {
                        audio[i] = static_cast<SampleType> (static_cast<float> (audio[i]) * state.linearGainCurrent);
                        if (state.smoothingStepsRemaining > 0)
                        {
                            state.linearGainCurrent += state.linearGainStep;
                            --state.smoothingStepsRemaining;
                            if (state.smoothingStepsRemaining == 0)
                                state.linearGainCurrent = state.linearGainTarget;
                        }
                    }
                }
            }
        }
    };

} // namespace PlayfulTones::DspToolbox::Processors
