/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/core/buffer_view.hpp"
#include "dsp_toolbox/core/io_config.hpp"
#include "dsp_toolbox/core/param.hpp"
#include "dsp_toolbox/math/constants.hpp"
#include "dsp_toolbox/math/functions.hpp"
#include "dsp_toolbox/processors/core/processor_base.hpp"

#include <array>
#include <cmath>
#include <cstddef>

namespace PlayfulTones::DspToolbox::Modulators
{

    /**
     * @brief LFO IOConfig: 0 audio, 0 CV in, 1 CV out (the LFO signal).
     *
     * Buffer layout: [CV out]
     * - Channel 0: LFO output
     */
    using LfoConfig = IOConfig<0, 0, 0, 1>;

    /**
     * @brief Available LFO waveform shapes.
     */
    enum class LfoWaveform : std::uint8_t {
        Sine,
        Triangle,
        Saw,
        Square
    };

    /**
     * @brief Parameter indices for Lfo.
     */
    enum LfoParamIndex : std::size_t {
        kFrequency = 0,
        kNumLfoParams
    };

    /**
     * @brief Parameter descriptors for Lfo.
     */
    inline constexpr std::array<ParamDescriptor, kNumLfoParams> LfoParamDescriptors { { { "frequency", "Frequency", 0.01f, 20.0f, 1.0f, "Hz" } } };

    /**
     * @brief Lfo unified state with embedded parameters.
     *
     * Contains both serializable parameters (in params) and transient
     * phase state.
     */
    struct LfoState
    {
        ParamSet<LfoParamDescriptors> params;

        float phase { 0.0f };
        LfoWaveform waveform { LfoWaveform::Sine };

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
        }
    };

    /**
     * @brief Low-frequency oscillator with sine, triangle, saw, and square waveforms.
     *
     * Produces bipolar [-1, 1] output at sub-audio rates (0.01–20 Hz).
     * Uses a phase accumulator incremented per sample.
     *
     * ## CV Convention
     * - Output: bipolar [-1.0, 1.0] CV
     *
     * ## Parameter Access
     * Use `setFrequency(hz)` or index-based `setParam<kFrequency>(hz)`.
     *
     * @tparam Spec Compile-time processor configuration
     */
    template <ConstexprSpec Spec = DefaultSpec>
    class Lfo : public ProcessorBase<Lfo<Spec>, LfoConfig, LfoState, Spec>
    {
    public:
        /**
         * @brief Set LFO frequency in Hz.
         */
        constexpr void setFrequency (float hz) noexcept
        {
            this->template setParam<kFrequency> (hz);
        }

        /**
         * @brief Set LFO waveform shape.
         */
        constexpr void setWaveform (LfoWaveform w) noexcept
        {
            this->state_.waveform = w;
        }

        /**
         * @brief Get current phase [0, 1).
         */
        [[nodiscard]] constexpr float getPhase() const noexcept
        {
            return this->state_.phase;
        }

        /**
         * @brief Get current waveform.
         */
        [[nodiscard]] constexpr LfoWaveform getWaveform() const noexcept
        {
            return this->state_.waveform;
        }

        template <typename SampleType>
        constexpr void processImpl (BufferView<SampleType>& buffer, LfoState& state, std::size_t sampleCount) noexcept
        {
            auto* out = buffer.getWritePointer (0);
            float const freq = state.params.template get<kFrequency>();
            float const phaseInc = freq / static_cast<float> (this->getSampleRate().value);

            for (std::size_t i = 0; i < sampleCount; ++i)
            {
                float value = 0.0f;
                switch (state.waveform)
                {
                    case LfoWaveform::Sine:
                        value = Math::sin (state.phase * Math::twoPi<float>);
                        break;
                    case LfoWaveform::Triangle:
                        value = 4.0f * std::abs (state.phase - 0.5f) - 1.0f;
                        break;
                    case LfoWaveform::Saw:
                        value = 2.0f * state.phase - 1.0f;
                        break;
                    case LfoWaveform::Square:
                        value = (state.phase < 0.5f) ? 1.0f : -1.0f;
                        break;
                }

                out[i] = static_cast<SampleType> (value);
                state.phase += phaseInc;
                if (state.phase >= 1.0f)
                    state.phase -= 1.0f;
            }
        }
    };

} // namespace PlayfulTones::DspToolbox::Modulators
