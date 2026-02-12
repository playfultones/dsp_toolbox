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
#include <cstddef>

namespace PlayfulTones::DspToolbox::Processors
{

    /**
     * @brief First-order IIR filter type enumeration.
     */
    enum class OnePoleType {
        Lowpass = 0,
        Highpass = 1
    };

    /**
     * @brief OnePole IOConfig: mono audio in/out, no CV.
     *
     * Buffer layout: [Audio I/O]
     * - Channel 0: Audio input/output (in-place processing)
     */
    using OnePoleConfig = IOConfig<1, 1, 0, 0>;

    /**
     * @brief First-order IIR filter coefficients.
     *
     * Transfer function: H(z) = (b0 + b1*z^-1) / (1 + a1*z^-1)
     */
    struct OnePoleCoefficients
    {
        float b0 { 1.0f };
        float b1 { 0.0f };
        float a1 { 0.0f };
    };

    /**
     * @brief Cached parameter values for coefficient hoisting.
     *
     * Invalid initial values force coefficient computation on first use.
     */
    struct OnePoleParamCache
    {
        float frequency { -1.0f };
        OnePoleType type { static_cast<OnePoleType> (255) };
        double sampleRate { 0.0 };
    };

    /**
     * @brief Parameter indices for OnePole.
     */
    enum OnePoleParamIndex : std::size_t {
        kOnePoleFrequency = 0,
        kOnePoleType = 1,
        kNumOnePoleParams
    };

    /**
     * @brief Parameter descriptors for OnePole.
     */
    inline constexpr std::array<ParamDescriptor, kNumOnePoleParams> OnePoleParamDescriptors { { { "frequency", "Frequency", 5.0f, 20000.0f, 1000.0f, "Hz" },
        { "type", "Type", 0.0f, 1.0f, 0.0f, "" } } };

    /**
     * @brief OnePole unified state with embedded parameters.
     *
     * Contains both serializable parameters (in params) and transient
     * filter state (delay line, coefficients).
     */
    struct OnePoleState
    {
        ParamSet<OnePoleParamDescriptors> params;

        float z1 { 0.0f }; // y[n-1] for lowpass, or filter delay element
        OnePoleCoefficients coeffs {};
        OnePoleParamCache paramCache {};
        bool dirty { true };

        constexpr void prepare (double sampleRate, std::size_t /*blockSize*/) noexcept
        {
            params.prepare (sampleRate);
            resetTransient();
        }

        constexpr void resetTransient() noexcept
        {
            z1 = 0.0f;
            paramCache = {};
            dirty = true;
        }
    };

    /**
     * @brief First-order IIR filter (6 dB/octave).
     *
     * Implements lowpass and highpass filters using bilinear transform
     * coefficient design. Provides 6 dB/octave rolloff.
     *
     * ## Transfer Function
     * H(z) = (b0 + b1*z^-1) / (1 + a1*z^-1)
     *
     * ## Filter Types
     * - **Lowpass**: Passes frequencies below cutoff
     *   - s-domain: H(s) = wc / (s + wc)
     *   - Bilinear: b0 = w*n, b1 = w*n, a1 = (w - 2fs) / (w + 2fs)
     * - **Highpass**: Passes frequencies above cutoff
     *   - s-domain: H(s) = s / (s + wc)
     *   - Bilinear: b0 = 2fs*n, b1 = -2fs*n, a1 = (w - 2fs) / (w + 2fs)
     *
     * where w = 2*fs*tan(pi*fc/fs), n = 1/(w + 2*fs)
     *
     * ## Usage
     * @code
     * OnePole<Spec48000_512> hpf;
     * hpf.configure(OnePoleType::Highpass, 40.0f);
     * hpf.process(buffer, numSamples);
     * @endcode
     *
     * @tparam Spec Compile-time processor configuration
     */
    template <ConstexprSpec Spec = DefaultSpec>
    class OnePole : public ProcessorBase<OnePole<Spec>, OnePoleConfig, OnePoleState, Spec>
    {
    public:
        /**
         * @brief Processor template alias for StereoExpander.
         */
        template <ConstexprSpec S>
        using Processor = OnePole<S>;

        /**
         * @brief Set filter type.
         */
        constexpr void setType (OnePoleType type) noexcept
        {
            float const newValue = static_cast<float> (type);
            if (!Math::exactlyEquals (this->template getParam<kOnePoleType>(), newValue))
            {
                this->template setParam<kOnePoleType> (newValue);
                this->state_.dirty = true;
            }
        }

        /**
         * @brief Get filter type.
         */
        [[nodiscard]] constexpr OnePoleType getType() const noexcept
        {
            return static_cast<OnePoleType> (static_cast<int> (this->template getParam<kOnePoleType>()));
        }

        /**
         * @brief Set cutoff frequency in Hz.
         */
        constexpr void setFrequency (float frequency) noexcept
        {
            if (!Math::exactlyEquals (this->template getParam<kOnePoleFrequency>(), frequency))
            {
                this->template setParam<kOnePoleFrequency> (frequency);
                this->state_.dirty = true;
            }
        }

        /**
         * @brief Get cutoff frequency in Hz.
         */
        [[nodiscard]] constexpr float getFrequency() const noexcept
        {
            return this->template getParam<kOnePoleFrequency>();
        }

        /**
         * @brief Configure type and frequency, then update coefficients.
         *
         * @param type Filter type (Lowpass or Highpass)
         * @param frequency Cutoff frequency in Hz
         */
        constexpr void configure (OnePoleType type, float frequency) noexcept
        {
            setType (type);
            setFrequency (frequency);
            updateCoefficients();
        }

        /**
         * @brief Update filter coefficients after parameter changes.
         *
         * Uses dirty flag for fast path when parameters unchanged.
         */
        constexpr void updateCoefficients() noexcept
        {
            if (!this->state_.dirty)
                return;

            this->state_.dirty = false;

            double const sampleRate = this->getSampleRate().value;
            float const frequency = this->template getParam<kOnePoleFrequency>();
            OnePoleType const type = getType();

            auto& cache = this->state_.paramCache;
            if (Math::exactlyEquals (cache.frequency, frequency) && cache.type == type && Math::exactlyEquals (cache.sampleRate, sampleRate))
            {
                return;
            }

            cache.frequency = frequency;
            cache.type = type;
            cache.sampleRate = sampleRate;

            float const fs = static_cast<float> (sampleRate);

            float const w = 2.0f * fs * Math::tan (Math::pi<float> * frequency / fs);
            float const n = 1.0f / (w + 2.0f * fs);

            switch (type)
            {
                case OnePoleType::Lowpass:
                    this->state_.coeffs.b0 = w * n;
                    this->state_.coeffs.b1 = w * n;
                    this->state_.coeffs.a1 = (w - 2.0f * fs) * n;
                    break;

                case OnePoleType::Highpass:
                    this->state_.coeffs.b0 = 2.0f * fs * n;
                    this->state_.coeffs.b1 = -2.0f * fs * n;
                    this->state_.coeffs.a1 = (w - 2.0f * fs) * n;
                    break;
            }
        }

        /**
         * @brief Get magnitude response at a specific frequency.
         *
         * Evaluates |H(e^(jw))| using the current filter coefficients.
         *
         * @param frequencyHz Query frequency in Hz
         * @return Linear magnitude (1.0 = unity gain)
         */
        [[nodiscard]] constexpr float getMagnitudeAtFrequency (float frequencyHz) const noexcept
        {
            float const sampleRate = static_cast<float> (this->getSampleRate().value);
            float const omega = Math::twoPi<float> * frequencyHz / sampleRate;
            float const cosW = Math::cos (omega);
            float const sinW = Math::sin (omega);

            auto const& c = this->state_.coeffs;

            float const numReal = c.b0 + c.b1 * cosW;
            float const numImag = -c.b1 * sinW;

            float const denReal = 1.0f + c.a1 * cosW;
            float const denImag = -c.a1 * sinW;

            float const numMagSq = numReal * numReal + numImag * numImag;
            float const denMagSq = denReal * denReal + denImag * denImag;

            return Math::sqrt (numMagSq / denMagSq);
        }

        /**
         * @brief Get magnitude response in dB at a specific frequency.
         *
         * @param frequencyHz Query frequency in Hz
         * @return Magnitude in dB (0 dB = unity gain)
         */
        [[nodiscard]] constexpr float getMagnitudeDbAtFrequency (float frequencyHz) const noexcept
        {
            float const mag = getMagnitudeAtFrequency (frequencyHz);
            return 20.0f * Math::log (mag) / Math::ln10<float>;
        }

        /**
         * @brief Process audio through the filter.
         *
         * Applies first-order IIR filtering with per-block coefficient updates.
         */
        template <typename SampleType>
        constexpr void processImpl (BufferView<SampleType>& buffer, OnePoleState& state, std::size_t sampleCount) noexcept
        {
            updateCoefficients();

            auto* audio = buffer.getWritePointer (0);
            auto const& c = state.coeffs;

            for (std::size_t i = 0; i < sampleCount; ++i)
            {
                float const x0 = static_cast<float> (audio[i]);

                float const y0 = c.b0 * x0 + state.z1;
                state.z1 = c.b1 * x0 - c.a1 * y0;

                audio[i] = static_cast<SampleType> (y0);
            }
        }
    };

} // namespace PlayfulTones::DspToolbox::Processors
