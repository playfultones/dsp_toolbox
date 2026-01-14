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
     * @brief Biquad filter type enumeration.
     */
    enum class BiquadType {
        Lowpass = 0,
        Highpass = 1,
        Bandpass = 2,
        Peak = 3, ///< Parametric EQ band (uses gain)
        LowShelf = 4, ///< Low frequency shelf (uses gain)
        HighShelf = 5, ///< High frequency shelf (uses gain)
        Notch = 6, ///< Band-reject filter
        AllPass = 7 ///< Phase shift only, unity magnitude
    };

    /**
     * @brief Biquad IOConfig: mono audio in/out, no CV.
     *
     * Buffer layout: [Audio I/O]
     * - Channel 0: Audio input/output (in-place processing)
     */
    using BiquadConfig = IOConfig<1, 1, 0, 0>;

    /**
     * @brief Biquad filter coefficients (precomputed).
     */
    struct BiquadCoefficients
    {
        float b0 { 1.0f };
        float b1 { 0.0f };
        float b2 { 0.0f };
        float a1 { 0.0f };
        float a2 { 0.0f };
    };

    /**
     * @brief Cached parameter values for coefficient hoisting.
     *
     * Stores the parameter values used to compute coefficients, enabling
     * early-return when parameters haven't changed. Invalid initial values
     * force coefficient computation on first use.
     */
    struct BiquadParamCache
    {
        float frequency { -1.0f }; // Invalid initial value to force first update
        float q { -1.0f };
        float gainDb { -999.0f };
        BiquadType type { static_cast<BiquadType> (255) }; // Invalid type
        double sampleRate { 0.0 };
    };

    /**
     * @brief Parameter indices for Biquad.
     */
    enum BiquadParamIndex : std::size_t {
        kFrequency = 0,
        kQ = 1,
        kType = 2,
        kGainDb = 3, ///< Gain in dB (for Peak, LowShelf, HighShelf)
        kNumBiquadParams
    };

    /**
     * @brief Parameter descriptors for Biquad.
     */
    inline constexpr std::array<ParamDescriptor, kNumBiquadParams> BiquadParamDescriptors { {
        { "frequency", "Frequency", 20.0f, 20000.0f, 1000.0f, "Hz" },
        { "q", "Q", 0.1f, 20.0f, 0.707f, "" },
        { "type", "Type", 0.0f, 7.0f, 0.0f, "" }, // 0-7 for all filter types
        { "gain_db", "Gain", -24.0f, 24.0f, 0.0f, "dB" } // For Peak, LowShelf, HighShelf
    } };

    /**
     * @brief Biquad unified state with embedded parameters.
     *
     * Contains both serializable parameters (in params) and transient
     * filter state (delay lines, coefficients).
     */
    struct BiquadState
    {
        // Serializable parameters
        ParamSet<BiquadParamDescriptors> params;

        // Transient state (reset on reset(), not serialized)
        float x1 { 0.0f }; // x[n-1]
        float x2 { 0.0f }; // x[n-2]
        float y1 { 0.0f }; // y[n-1]
        float y2 { 0.0f }; // y[n-2]
        BiquadCoefficients coeffs {}; // Precomputed coefficients

        // Parameter cache for coefficient hoisting (enables per-sample change detection)
        BiquadParamCache paramCache {};

        // Dirty flag - true means coefficients need recalculation
        bool dirty { true }; // Initially dirty

        /**
         * @brief Prepare state for given sample rate.
         */
        constexpr void prepare (double sampleRate, std::size_t /*blockSize*/) noexcept
        {
            params.prepare (sampleRate);
            resetTransient();
        }

        /**
         * @brief Reset transient state only (preserves params and coefficients).
         *
         * Invalidates parameter cache to force coefficient recalculation on next process.
         */
        constexpr void resetTransient() noexcept
        {
            x1 = 0.0f;
            x2 = 0.0f;
            y1 = 0.0f;
            y2 = 0.0f;
            // Invalidate cache to force coefficient recalculation
            paramCache = {};
            // Mark as dirty to force coefficient recalculation
            dirty = true;
        }
    };

    /**
     * @brief Standard biquad (2nd-order IIR) filter.
     *
     * Implements Direct Form I biquad filter supporting common filter types.
     * Coefficients are calculated using standard Audio EQ Cookbook formulas.
     *
     * ## Transfer Function
     * H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
     *
     * ## Filter Types
     * - Lowpass: Passes frequencies below cutoff, attenuates above
     * - Highpass: Passes frequencies above cutoff, attenuates below
     * - Bandpass: Passes frequencies around center, attenuates others
     * - Peak: Parametric EQ band with boost/cut (uses gain parameter)
     * - LowShelf: Low frequency shelf with boost/cut (uses gain parameter)
     * - HighShelf: High frequency shelf with boost/cut (uses gain parameter)
     * - Notch: Band-reject filter, deep attenuation at center frequency
     * - AllPass: Phase shift only, unity magnitude response
     *
     * ## Parameter Access
     * Use index-based access:
     * - `setParam<kFrequency>(1000.0f)` - set cutoff frequency
     * - `setParam<kQ>(0.707f)` - set Q factor
     * - `setParam<kType>(0.0f)` - set type (see BiquadType enum)
     * - `setParam<kGainDb>(6.0f)` - set gain in dB (Peak, LowShelf, HighShelf only)
     *
     * ## Usage
     * @code
     * Biquad<MySpec> filter;
     * filter.configure(BiquadType::Peak, 1000.0f, 1.0f, 6.0f);  // +6dB peak at 1kHz
     * filter.process(buffer, numSamples);
     * @endcode
     *
     * @tparam Spec Compile-time processor configuration
     */
    template <ConstexprSpec Spec = DefaultSpec>
    class Biquad : public ProcessorBase<Biquad<Spec>, BiquadConfig, BiquadState, Spec>
    {
    public:
        /**
         * @brief Set filter type.
         */
        constexpr void setType (BiquadType type) noexcept
        {
            float const newValue = static_cast<float> (type);
            if (this->template getParam<kType>() != newValue)
            {
                this->template setParam<kType> (newValue);
                this->state_.dirty = true;
            }
        }

        /**
         * @brief Get filter type.
         */
        [[nodiscard]] constexpr BiquadType getType() const noexcept
        {
            return static_cast<BiquadType> (static_cast<int> (this->template getParam<kType>()));
        }

        /**
         * @brief Set cutoff/center frequency in Hz.
         */
        constexpr void setFrequency (float frequency) noexcept
        {
            if (this->template getParam<kFrequency>() != frequency)
            {
                this->template setParam<kFrequency> (frequency);
                this->state_.dirty = true;
            }
        }

        /**
         * @brief Get cutoff/center frequency in Hz.
         */
        [[nodiscard]] constexpr float getFrequency() const noexcept
        {
            return this->template getParam<kFrequency>();
        }

        /**
         * @brief Set Q factor (resonance).
         *
         * Q = 0.707 (sqrt(2)/2) is Butterworth (no resonance)
         * Q > 0.707 adds resonance
         */
        constexpr void setQ (float q) noexcept
        {
            if (this->template getParam<kQ>() != q)
            {
                this->template setParam<kQ> (q);
                this->state_.dirty = true;
            }
        }

        /**
         * @brief Get Q factor.
         */
        [[nodiscard]] constexpr float getQ() const noexcept
        {
            return this->template getParam<kQ>();
        }

        /**
         * @brief Set gain in dB (for Peak, LowShelf, HighShelf filters).
         *
         * Positive values boost, negative values cut.
         * Ignored for filter types that don't use gain.
         */
        constexpr void setGainDb (float gainDb) noexcept
        {
            if (this->template getParam<kGainDb>() != gainDb)
            {
                this->template setParam<kGainDb> (gainDb);
                this->state_.dirty = true;
            }
        }

        /**
         * @brief Get gain in dB.
         */
        [[nodiscard]] constexpr float getGainDb() const noexcept
        {
            return this->template getParam<kGainDb>();
        }

        /**
         * @brief Check if current filter type uses the gain parameter.
         *
         * @return true for Peak, LowShelf, HighShelf; false otherwise
         */
        [[nodiscard]] constexpr bool usesGain() const noexcept
        {
            BiquadType const type = getType();
            return type == BiquadType::Peak || type == BiquadType::LowShelf || type == BiquadType::HighShelf;
        }

        /**
         * @brief Get magnitude response at a specific frequency.
         *
         * Evaluates |H(e^(jω))| using the current filter coefficients.
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
            float const cos2W = Math::cos (2.0f * omega);
            float const sin2W = Math::sin (2.0f * omega);

            auto const& c = this->state_.coeffs;

            // Numerator: b0 + b1*e^(-jw) + b2*e^(-j2w)
            float const numReal = c.b0 + (c.b1 * cosW) + (c.b2 * cos2W);
            float const numImag = -(c.b1 * sinW) - (c.b2 * sin2W);

            // Denominator: 1 + a1*e^(-jw) + a2*e^(-j2w)
            float const denReal = 1.0f + (c.a1 * cosW) + (c.a2 * cos2W);
            float const denImag = -(c.a1 * sinW) - (c.a2 * sin2W);

            float const numMagSq = (numReal * numReal) + (numImag * numImag);
            float const denMagSq = (denReal * denReal) + (denImag * denImag);

            return Math::sqrt (numMagSq / denMagSq);
        }

        /**
         * @brief Get phase response at a specific frequency.
         *
         * Evaluates arg(H(e^(jω))) using the current filter coefficients.
         *
         * @param frequencyHz Query frequency in Hz
         * @return Phase in radians
         */
        [[nodiscard]] constexpr float getPhaseAtFrequency (float frequencyHz) const noexcept
        {
            float const sampleRate = static_cast<float> (this->getSampleRate().value);
            float const omega = Math::twoPi<float> * frequencyHz / sampleRate;
            float const cosW = Math::cos (omega);
            float const sinW = Math::sin (omega);
            float const cos2W = Math::cos (2.0f * omega);
            float const sin2W = Math::sin (2.0f * omega);

            auto const& c = this->state_.coeffs;

            float const numReal = c.b0 + (c.b1 * cosW) + (c.b2 * cos2W);
            float const numImag = -(c.b1 * sinW) - (c.b2 * sin2W);
            float const denReal = 1.0f + (c.a1 * cosW) + (c.a2 * cos2W);
            float const denImag = -(c.a1 * sinW) - (c.a2 * sin2W);

            float const numPhase = Math::atan2 (numImag, numReal);
            float const denPhase = Math::atan2 (denImag, denReal);

            return numPhase - denPhase;
        }

        /**
         * @brief Get magnitude response in decibels at a specific frequency.
         *
         * @param frequencyHz Query frequency in Hz
         * @return Magnitude in dB (0 dB = unity gain)
         */
        [[nodiscard]] constexpr float getMagnitudeDbAtFrequency (float frequencyHz) const noexcept
        {
            float const mag = getMagnitudeAtFrequency (frequencyHz);
            // 20 * log10(mag) = 20 * ln(mag) / ln(10)
            return 20.0f * Math::log (mag) / Math::ln10<float>;
        }

        /**
         * @brief Update filter coefficients after parameter changes.
         *
         * Uses dirty flag for fast path when parameters unchanged.
         */
        constexpr void updateCoefficients() noexcept
        {
            // Fast path: no dirty flag set
            if (!this->state_.dirty)
            {
                return;
            }

            this->state_.dirty = false;

            double const sampleRate = this->getSampleRate().value;
            float const frequency = this->template getParam<kFrequency>();
            float const q = this->template getParam<kQ>();
            float const gainDb = this->template getParam<kGainDb>();
            BiquadType const type = getType();

            // Coefficient hoisting: skip recalculation if parameters unchanged
            auto& cache = this->state_.paramCache;
            if (cache.frequency == frequency && cache.q == q && cache.gainDb == gainDb && cache.type == type && cache.sampleRate == sampleRate)
            {
                return; // Parameters unchanged, coefficients still valid
            }

            // Update cache with current parameters
            cache.frequency = frequency;
            cache.q = q;
            cache.gainDb = gainDb;
            cache.type = type;
            cache.sampleRate = sampleRate;

            float const omega = Math::twoPi<float> * frequency / static_cast<float> (sampleRate);
            auto const [sinOmega, cosOmega] = Math::sincos (omega);
            float const alpha = sinOmega / (2.0f * q);

            float b0 = 0.0f, b1 = 0.0f, b2 = 0.0f;
            float a0 = 0.0f, a1 = 0.0f, a2 = 0.0f;

            switch (type)
            {
                case BiquadType::Lowpass:
                    b0 = (1.0f - cosOmega) / 2.0f;
                    b1 = 1.0f - cosOmega;
                    b2 = (1.0f - cosOmega) / 2.0f;
                    a0 = 1.0f + alpha;
                    a1 = -2.0f * cosOmega;
                    a2 = 1.0f - alpha;
                    break;

                case BiquadType::Highpass:
                    b0 = (1.0f + cosOmega) / 2.0f;
                    b1 = -(1.0f + cosOmega);
                    b2 = (1.0f + cosOmega) / 2.0f;
                    a0 = 1.0f + alpha;
                    a1 = -2.0f * cosOmega;
                    a2 = 1.0f - alpha;
                    break;

                case BiquadType::Bandpass:
                    b0 = alpha;
                    b1 = 0.0f;
                    b2 = -alpha;
                    a0 = 1.0f + alpha;
                    a1 = -2.0f * cosOmega;
                    a2 = 1.0f - alpha;
                    break;

                case BiquadType::Peak:
                {
                    // A = sqrt(10^(dB/20)) = 10^(dB/40)
                    float const A = Math::exp10_40 (gainDb);
                    b0 = 1.0f + (alpha * A);
                    b1 = -2.0f * cosOmega;
                    b2 = 1.0f - (alpha * A);
                    a0 = 1.0f + (alpha / A);
                    a1 = -2.0f * cosOmega;
                    a2 = 1.0f - (alpha / A);
                    break;
                }

                case BiquadType::LowShelf:
                {
                    float const A = Math::exp10_40 (gainDb);
                    float const sqrtA = Math::sqrt (A);
                    float const sqrtA_alpha_2 = 2.0f * sqrtA * alpha;
                    float const Ap1 = A + 1.0f;
                    float const Am1 = A - 1.0f;

                    b0 = A * ((Ap1 - (Am1 * cosOmega)) + sqrtA_alpha_2);
                    b1 = 2.0f * A * ((Am1 - (Ap1 * cosOmega)));
                    b2 = A * ((Ap1 - (Am1 * cosOmega)) - sqrtA_alpha_2);
                    a0 = (Ap1 + (Am1 * cosOmega)) + sqrtA_alpha_2;
                    a1 = -2.0f * ((Am1 + (Ap1 * cosOmega)));
                    a2 = (Ap1 + (Am1 * cosOmega)) - sqrtA_alpha_2;
                    break;
                }

                case BiquadType::HighShelf:
                {
                    float const A = Math::exp10_40 (gainDb);
                    float const sqrtA = Math::sqrt (A);
                    float const sqrtA_alpha_2 = 2.0f * sqrtA * alpha;
                    float const Ap1 = A + 1.0f;
                    float const Am1 = A - 1.0f;

                    b0 = A * ((Ap1 + (Am1 * cosOmega)) + sqrtA_alpha_2);
                    b1 = -2.0f * A * ((Am1 + (Ap1 * cosOmega)));
                    b2 = A * ((Ap1 + (Am1 * cosOmega)) - sqrtA_alpha_2);
                    a0 = (Ap1 - (Am1 * cosOmega)) + sqrtA_alpha_2;
                    a1 = 2.0f * ((Am1 - (Ap1 * cosOmega)));
                    a2 = (Ap1 - (Am1 * cosOmega)) - sqrtA_alpha_2;
                    break;
                }

                case BiquadType::Notch:
                    b0 = 1.0f;
                    b1 = -2.0f * cosOmega;
                    b2 = 1.0f;
                    a0 = 1.0f + alpha;
                    a1 = -2.0f * cosOmega;
                    a2 = 1.0f - alpha;
                    break;

                case BiquadType::AllPass:
                    b0 = 1.0f - alpha;
                    b1 = -2.0f * cosOmega;
                    b2 = 1.0f + alpha;
                    a0 = 1.0f + alpha;
                    a1 = -2.0f * cosOmega;
                    a2 = 1.0f - alpha;
                    break;
            }

            // Normalize coefficients by a0
            float const invA0 = 1.0f / a0;
            this->state_.coeffs.b0 = b0 * invA0;
            this->state_.coeffs.b1 = b1 * invA0;
            this->state_.coeffs.b2 = b2 * invA0;
            this->state_.coeffs.a1 = a1 * invA0;
            this->state_.coeffs.a2 = a2 * invA0;
        }

        /**
         * @brief Configure all parameters at once and update coefficients.
         *
         * @param type Filter type
         * @param frequency Cutoff/center frequency in Hz
         * @param q Q factor (resonance)
         * @param gainDb Gain in dB (only used for Peak, LowShelf, HighShelf)
         */
        constexpr void configure (BiquadType type, float frequency, float q, float gainDb = 0.0f) noexcept
        {
            setType (type);
            setFrequency (frequency);
            setQ (q);
            setGainDb (gainDb);
            updateCoefficients();
        }

        /**
         * @brief Process audio through the filter with per-sample coefficient updates.
         *
         * Automatically detects parameter changes each sample and recalculates
         * coefficients when needed. This enables sample-accurate automation
         * response for DAW parameter changes.
         */
        template <typename SampleType>
        constexpr void processImpl (BufferView<SampleType>& buffer, BiquadState& state, std::size_t sampleCount) noexcept
        {
            auto* audio = buffer.getWritePointer (0);

            for (std::size_t i = 0; i < sampleCount; ++i)
            {
                updateCoefficients();

                auto const& c = state.coeffs;
                float const x0 = static_cast<float> (audio[i]);

                // Direct Form I: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
                float const y0 = c.b0 * x0 + c.b1 * state.x1 + c.b2 * state.x2 - c.a1 * state.y1 - c.a2 * state.y2;

                state.x2 = state.x1;
                state.x1 = x0;
                state.y2 = state.y1;
                state.y1 = y0;

                audio[i] = static_cast<SampleType> (y0);
            }
        }
    };

} // namespace PlayfulTones::DspToolbox::Processors
