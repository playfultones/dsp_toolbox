/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/core/buffer_view.hpp"
#include "dsp_toolbox/core/conversions.hpp"
#include "dsp_toolbox/core/frequency.hpp"
#include "dsp_toolbox/core/io_config.hpp"
#include "dsp_toolbox/core/param.hpp"
#include "dsp_toolbox/math/constants.hpp"
#include "dsp_toolbox/math/functions.hpp"
#include "dsp_toolbox/processors/core/processor_base.hpp"

#include <array>
#include <cstddef>

namespace PlayfulTones::DspToolbox::Generators
{

    /**
     * @brief VCO IOConfig: 2 audio out (sine, supersaw), 1 CV in (pitch).
     *
     * Buffer layout: [Sine out, SuperSaw out, Pitch CV in]
     * - Channel 0: Sine wave output
     * - Channel 1: SuperSaw output (7 detuned saws, JP-8000 style)
     * - Channel 2: Pitch CV input (1V/Oct)
     */
    using VCOConfig = IOConfig<0, 2, 1, 0>;

    /**
     * @brief Number of oscillators in SuperSaw.
     */
    inline constexpr std::size_t kSupersawVoices = 7;

    /**
     * @brief JP-8000 style detune ratios (normalized).
     *
     * Original JP-8000 table: { 0, 128, -128, 816, -824, 1408, -1448 }
     * Normalized so outer voices = ±1.0, scaled by detune parameter.
     */
    inline constexpr std::array<float, kSupersawVoices> kSupersawDetuneRatios = {
        0.0f, // Center voice
        0.08840f, // Voice 1: 128/1448
        -0.08840f, // Voice 2: -128/1448
        0.56354f, // Voice 3: 816/1448
        -0.56906f, // Voice 4: -824/1448
        0.97238f, // Voice 5: 1408/1448
        -1.0f // Voice 6: -1448/1448
    };

    /**
     * @brief Parameter indices for VCO.
     */
    enum VCOParamIndex : std::size_t {
        kBaseFrequency = 0,
        kSupersawDetune = 1,
        kSupersawMix = 2,
        kNumVCOParams
    };

    /**
     * @brief Parameter descriptors for VCO.
     */
    inline constexpr std::array<ParamDescriptor, kNumVCOParams> VCOParamDescriptors { { { "baseFrequency", "Base Frequency", 20.0f, 20000.0f, 261.63f, "Hz" },
        { "supersawDetune", "SuperSaw Detune", 0.0f, 1.0f, 0.5f, "" },
        { "supersawMix", "SuperSaw Mix", 0.0f, 1.0f, 0.5f, "" } } };

    /**
     * @brief VCO unified state with embedded parameters.
     *
     * Contains both serializable parameters (in params) and transient
     * oscillator state (phase accumulators, filter state).
     */
    struct VCOState
    {
        // Serializable parameters
        ParamSet<VCOParamDescriptors> params;

        // Transient state (reset on reset(), not serialized)
        float phase { 0.0f }; ///< Main phase (sine)
        std::array<float, kSupersawVoices> sawPhases {}; ///< SuperSaw phases
        float hpPrevInput { 0.0f }; ///< DC blocker previous input
        float hpState { 0.0f }; ///< DC blocker state

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
            phase = 0.0f;
            sawPhases = {};
            hpPrevInput = 0.0f;
            hpState = 0.0f;
        }
    };

    /**
     * @brief Voltage Controlled Oscillator with Sine and SuperSaw outputs.
     *
     * Multi-waveform oscillator with:
     * - **Sine**: Clean sine wave on channel 0
     * - **SuperSaw**: JP-8000 style 7-voice detuned saw on channel 1
     *
     * ## SuperSaw Implementation
     * Based on the Roland JP-8000 algorithm:
     * - 7 naive sawtooth oscillators with fixed detune ratios
     * - Center voice at full level, side voices scaled by mix parameter
     * - 2x internal oversampling to reduce aliasing
     * - Built-in DC blocker (~20Hz highpass)
     *
     * ## CV Convention
     * - Pitch CV input uses 1V/Oct: cv=0 → baseFreq, cv=1 → 2x baseFreq
     * - Output range: -1.0 to +1.0
     *
     * ## Parameter Access
     * Use index-based access:
     * - `setParam<kBaseFrequency>(440.0f)` - set base frequency
     * - `setParam<kSupersawDetune>(0.7f)` - set detune amount
     * - `setParam<kSupersawMix>(0.8f)` - set voice mix
     *
     * ## Buffer Layout
     * - Channel 0: Sine wave output
     * - Channel 1: SuperSaw output
     * - Channel 2: Pitch CV input (1V/Oct)
     *
     * @tparam Spec Compile-time processor configuration
     */
    template <ConstexprSpec Spec = DefaultSpec>
    class VCO : public ProcessorBase<VCO<Spec>, VCOConfig, VCOState, Spec>
    {
    public:
        /**
         * @brief Get current phase (0.0 to 1.0).
         */
        [[nodiscard]] constexpr float getPhase() const noexcept
        {
            return this->state_.phase;
        }

        /**
         * @brief Set base frequency in Hz.
         */
        constexpr void setBaseFrequencyHz (float hz) noexcept
        {
            this->template setParam<kBaseFrequency> (hz);
        }

        /**
         * @brief Get base frequency in Hz.
         */
        [[nodiscard]] constexpr float getBaseFrequencyHz() const noexcept
        {
            return this->template getParam<kBaseFrequency>();
        }

        /**
         * @brief Get base frequency as Frequency type.
         */
        [[nodiscard]] constexpr Frequency<float> getBaseFrequency() const noexcept
        {
            return Frequency<float> { this->template getParam<kBaseFrequency>() };
        }

        /**
         * @brief Set SuperSaw detune amount (0.0 to 1.0).
         */
        constexpr void setSupersawDetune (float detune) noexcept
        {
            this->template setParam<kSupersawDetune> (detune);
        }

        /**
         * @brief Get SuperSaw detune amount.
         */
        [[nodiscard]] constexpr float getSupersawDetune() const noexcept
        {
            return this->template getParam<kSupersawDetune>();
        }

        /**
         * @brief Set SuperSaw mix amount (0.0 to 1.0).
         */
        constexpr void setSupersawMix (float mix) noexcept
        {
            this->template setParam<kSupersawMix> (mix);
        }

        /**
         * @brief Get SuperSaw mix amount.
         */
        [[nodiscard]] constexpr float getSupersawMix() const noexcept
        {
            return this->template getParam<kSupersawMix>();
        }

        template <typename SampleType>
        constexpr void processImpl (BufferView<SampleType>& buffer, VCOState& state, std::size_t sampleCount) noexcept
        {
            auto* sineOut = buffer.getWritePointer (0);
            auto* supersawOut = buffer.getWritePointer (1);
            auto const* pitchCV = buffer.getReadPointer (2);

            float const sampleRate = static_cast<float> (this->getSampleRate().value);
            float const oversampledRate = sampleRate * 2.0f;

            // DC blocker coefficient: R = 1 - (2 * pi * fc / fs), fc ≈ 20Hz
            float const hpCoeff = 1.0f - (Math::twoPi<float> * 20.0f / sampleRate);

            // Max detune in CV (1 semitone = 1/12 octave)
            float const maxDetuneCv = 1.0f / 12.0f;

            float const baseFreqHz = state.params.get (kBaseFrequency);
            float const supersawDetune = state.params.get (kSupersawDetune);
            float const supersawMix = state.params.get (kSupersawMix);
            Frequency<float> const baseFrequency { baseFreqHz };

            for (std::size_t i = 0; i < sampleCount; ++i)
            {
                float const cv = static_cast<float> (pitchCV[i]);
                Frequency<float> const freq = cvToFrequency (cv, baseFrequency);

                float const phaseIncrement = freq.value() / sampleRate;

                sineOut[i] = static_cast<SampleType> (Math::sin (state.phase * Math::twoPi<float>));

                state.phase += phaseIncrement;
                if (state.phase >= 1.0f)
                {
                    state.phase -= 1.0f;
                }

                float supersawSum1 = 0.0f;
                float supersawSum2 = 0.0f;

                for (std::size_t v = 0; v < kSupersawVoices; ++v)
                {
                    float const detuneOffset = kSupersawDetuneRatios[v] * supersawDetune * maxDetuneCv;
                    Frequency<float> const voiceFreq = cvToFrequency (cv + detuneOffset, baseFrequency);
                    float const voiceIncrement = voiceFreq.value() / oversampledRate;

                    // First oversample
                    float saw1 = state.sawPhases[v] * 2.0f - 1.0f; // Convert 0-1 phase to -1 to +1 saw

                    state.sawPhases[v] += voiceIncrement;
                    if (state.sawPhases[v] >= 1.0f)
                    {
                        state.sawPhases[v] -= 1.0f;
                    }

                    // Second oversample
                    float saw2 = state.sawPhases[v] * 2.0f - 1.0f;

                    state.sawPhases[v] += voiceIncrement;
                    if (state.sawPhases[v] >= 1.0f)
                    {
                        state.sawPhases[v] -= 1.0f;
                    }

                    // Mix: center voice full, side voices scaled
                    float const voiceGain = (v == 0) ? 1.0f : supersawMix;
                    supersawSum1 += saw1 * voiceGain;
                    supersawSum2 += saw2 * voiceGain;
                }

                // Average oversampled values (simple box filter for decimation)
                float supersawRaw = (supersawSum1 + supersawSum2) * 0.5f;

                // Normalize by number of voices (adjusted for mix)
                float const normalization = 1.0f / (1.0f + 6.0f * supersawMix);
                supersawRaw *= normalization;

                // DC blocker: y[n] = x[n] - x[n-1] + R * y[n-1]
                float const hpOut = supersawRaw - state.hpPrevInput + hpCoeff * state.hpState;
                state.hpPrevInput = supersawRaw;
                state.hpState = hpOut;

                supersawOut[i] = static_cast<SampleType> (hpOut);
            }
        }
    };

} // namespace PlayfulTones::DspToolbox::Generators
