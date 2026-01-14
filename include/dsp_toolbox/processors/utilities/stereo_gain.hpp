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
#include "dsp_toolbox/processors/utilities/gain.hpp"
#include "dsp_toolbox/simd/simd.hpp"

#include <cstddef>
#include <type_traits>

namespace PlayfulTones::DspToolbox::Processors
{

    /**
     * @brief StereoGain IOConfig: stereo audio in/out, no CV.
     *
     * Buffer layout: [Audio I/O]
     * - Channel 0: Left input/output (in-place processing)
     * - Channel 1: Right input/output (in-place processing)
     */
    using StereoGainConfig = IOConfig<2, 2, 0, 0>;

    /**
     * @brief StereoGain unified state with embedded parameters.
     *
     * Single shared state for both L/R channels. Contains one gain parameter
     * and one smoothing mechanism applied to both channels identically.
     */
    struct StereoGainState
    {
        /// Shared state flag - true because both channels use identical gain
        static constexpr bool shared = true;

        /// Serializable parameters (single shared param for both channels)
        ParamSet<GainParamDescriptors> params;

        /// Linear gain smoothing (faster than per-sample dB conversion)
        float linearGainTarget { 1.0f };
        float linearGainCurrent { 1.0f };
        float linearGainStep { 0.0f };
        std::size_t smoothingStepsRemaining { 0 };
        std::size_t smoothingSteps { 480 }; // Default: 10ms at 48kHz

        /// Dirty flag: set when gain changes, cleared after param ticking
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
     * @brief Native stereo gain/volume control with dB parameter.
     *
     * Optimized stereo implementation that processes L/R channels together
     * with shared gain parameter. Provides better performance than wrapping
     * mono Gain in StereoExpander by:
     *
     * - Single gain parameter and smoothing state for both channels
     * - No StereoExpander abstraction overhead (no buffer splitting)
     * - Single process call instead of two
     * - Identical gain applied to both channels simultaneously
     *
     * ## Parameter Range
     * - -60 dB to +12 dB (unity = 0 dB)
     * - 10ms smoothing (default)
     *
     * ## Buffer Layout
     * - Channel 0: Left audio input/output
     * - Channel 1: Right audio input/output
     *
     * ## Parameter Access
     * Use convenience methods:
     * - `setGainDb(-6.0f)` - set gain to -6 dB (applied to both L/R)
     * - `setGain(Decibels<float>{-6.0f})` - using strong type
     *
     * @tparam Spec Compile-time processor configuration
     */
    template <ConstexprSpec Spec = DefaultSpec>
    class StereoGain : public ProcessorBase<StereoGain<Spec>, StereoGainConfig, StereoGainState, Spec>
    {
    public:
        /**
         * @brief Processor template alias for graph usage.
         */
        template <ConstexprSpec S>
        using Processor = StereoGain<S>;

        /**
         * @brief Set gain in dB (smoothed transition).
         *
         * Uses parameter smoothing to prevent clicks when gain changes.
         * Converts dB to linear once and smooths in linear domain for efficiency.
         * Applied identically to both L and R channels.
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
         * @brief Process stereo audio buffer.
         *
         * Applies gain with linear-domain smoothing for efficiency.
         * Both L and R channels receive identical gain simultaneously.
         *
         * Optimizations:
         * - Value hoisting for gain=0 (silence) and gain=1 (pass-through)
         * - SIMD-accelerated gain multiplication
         * - SIMD-accelerated gain ramping during smoothing
         */
        template <typename SampleType>
        constexpr void processImpl (BufferView<SampleType>& buffer, StereoGainState& state, std::size_t sampleCount) noexcept
        {
            auto* audioL = buffer.getWritePointer (0);
            auto* audioR = buffer.getWritePointer (1);

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
                        // gain=0: output silence on both channels
                        if constexpr (std::is_same_v<SampleType, float>)
                        {
                            simd::clear (audioL, sampleCount);
                            simd::clear (audioR, sampleCount);
                        }
                        else
                        {
                            for (std::size_t i = 0; i < sampleCount; ++i)
                            {
                                audioL[i] = SampleType { 0 };
                                audioR[i] = SampleType { 0 };
                            }
                        }
                    }
                    else if constexpr (isConstantOne<decltype (gainCRV)>)
                    {
                        // gain=1: pass-through, no processing needed
                    }
                    else
                    {
                        // Normal gain - SIMD multiply on both channels
                        if constexpr (std::is_same_v<SampleType, float>)
                        {
                            simd::multiplyUnrolled (audioL, sampleCount, gainCRV.get());
                            simd::multiplyUnrolled (audioR, sampleCount, gainCRV.get());
                        }
                        else
                        {
                            float const gain = gainCRV.get();
                            for (std::size_t i = 0; i < sampleCount; ++i)
                            {
                                audioL[i] = static_cast<SampleType> (static_cast<float> (audioL[i]) * gain);
                                audioR[i] = static_cast<SampleType> (static_cast<float> (audioR[i]) * gain);
                            }
                        }
                    }
                });
            }
            else
            {
                // Apply same gain ramp to both channels
                if constexpr (std::is_same_v<SampleType, float>)
                {
                    // Save state before first channel so we can replay the same ramp
                    float const startGain = state.linearGainCurrent;
                    std::size_t const startSteps = state.smoothingStepsRemaining;
                    float const target = state.linearGainTarget;

                    simd::multiplyRamp (audioL, sampleCount, state.linearGainCurrent, state.linearGainStep, state.smoothingStepsRemaining, state.linearGainTarget);

                    // Replay same ramp for right channel
                    state.linearGainCurrent = startGain;
                    state.smoothingStepsRemaining = startSteps;
                    simd::multiplyRamp (audioR, sampleCount, state.linearGainCurrent, state.linearGainStep, state.smoothingStepsRemaining, target);
                }
                else
                {
                    // Non-float: interleaved scalar processing
                    for (std::size_t i = 0; i < sampleCount; ++i)
                    {
                        float const gain = state.linearGainCurrent;
                        audioL[i] = static_cast<SampleType> (static_cast<float> (audioL[i]) * gain);
                        audioR[i] = static_cast<SampleType> (static_cast<float> (audioR[i]) * gain);

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
