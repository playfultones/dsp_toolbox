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
#include "dsp_toolbox/processors/filters/biquad.hpp"

#include <array>
#include <cstddef>

namespace PlayfulTones::DspToolbox::Processors
{

    /**
     * @brief MultibandEQ IOConfig: mono audio in/out, no CV.
     *
     * Buffer layout: [Audio I/O]
     * - Channel 0: Audio input/output (in-place processing)
     */
    using MultibandEQConfig = IOConfig<1, 1, 0, 0>;

    /**
     * @brief Parameter indices for a single EQ band.
     */
    enum MultibandEQBandParamIndex : std::size_t {
        kBandFrequency = 0,
        kBandQ = 1,
        kBandType = 2,
        kBandGainDb = 3,
        kNumParamsPerBand = 4
    };

    /**
     * @brief Generates parameter descriptors for all bands.
     *
     * Creates an array of ParamDescriptors for NumBands bands, each with
     * frequency, Q, type, and gain parameters.
     *
     * @tparam NumBands Number of EQ bands
     * @return Array of parameter descriptors
     */
    template <std::size_t NumBands>
    consteval auto makeMultibandEQParamDescriptors()
    {
        std::array<ParamDescriptor, NumBands * kNumParamsPerBand> descriptors {};

        for (std::size_t band = 0; band < NumBands; ++band)
        {
            std::size_t const baseIndex = band * kNumParamsPerBand;

            // Frequency parameter
            descriptors[baseIndex + kBandFrequency] = ParamDescriptor {
                "", // ID will be set at runtime if needed
                "", // Name
                5.0f, // Min
                20000.0f, // Max
                1000.0f, // Default
                "Hz" // Unit
            };

            // Q parameter
            descriptors[baseIndex + kBandQ] = ParamDescriptor {
                "",
                "",
                0.1f,
                20.0f,
                0.707f,
                ""
            };

            // Type parameter
            descriptors[baseIndex + kBandType] = ParamDescriptor {
                "",
                "",
                0.0f,
                9.0f,
                static_cast<float> (BiquadType::Peak), // Default to Peak for middle bands
                ""
            };

            // Gain parameter
            descriptors[baseIndex + kBandGainDb] = ParamDescriptor {
                "",
                "",
                -24.0f,
                24.0f,
                0.0f,
                "dB"
            };
        }

        return descriptors;
    }

    /**
     * @brief Per-band filter state (delay elements for Direct Form I).
     */
    struct EQBandDelayState
    {
        float x1 { 0.0f }; // x[n-1]
        float x2 { 0.0f }; // x[n-2]
        float y1 { 0.0f }; // y[n-1]
        float y2 { 0.0f }; // y[n-2]
    };

    /**
     * @brief Cached band parameter values for coefficient hoisting.
     *
     * Stores the parameter values used to compute coefficients, enabling
     * early-return in updateBandCoefficients() when parameters unchanged.
     */
    struct EQBandParamCache
    {
        float frequency { -1.0f }; // Invalid initial value to force first update
        float q { -1.0f };
        float gainDb { -999.0f };
        BiquadType type { static_cast<BiquadType> (255) }; // Invalid type
        double sampleRate { 0.0 };
    };

    /**
     * @brief MultibandEQ unified state with embedded parameters.
     *
     * Contains both serializable parameters and transient filter state.
     * shared = false ensures separate state per channel when used with
     * StereoExpander.
     *
     * @tparam NumBands Number of EQ bands
     */
    template <std::size_t NumBands>
    struct MultibandEQState
    {
        /// Shared state flag - false for independent channel processing
        static constexpr bool shared = false;

        /// Parameter descriptors for this band count
        static constexpr auto descriptors = makeMultibandEQParamDescriptors<NumBands>();

        /// Serializable parameters
        ParamSet<descriptors> params;

        /// Per-band coefficients (transient)
        std::array<BiquadCoefficients, NumBands> coeffs {};

        /// Per-band delay lines (transient)
        std::array<EQBandDelayState, NumBands> delayLines {};

        /// Per-band parameter cache for coefficient hoisting
        std::array<EQBandParamCache, NumBands> paramCache {};

        /// Dirty flag bitmap - bit N set means band N needs coefficient update
        std::uint8_t dirtyBands { 0xFF }; // Initially all dirty

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
         * Clears delay lines and invalidates parameter cache. Coefficients are
         * preserved to maintain frequency response after bypass toggle.
         */
        constexpr void resetTransient() noexcept
        {
            for (auto& d : delayLines)
            {
                d = {};
            }
            // Invalidate parameter cache to force coefficient recalculation on next update
            for (auto& cache : paramCache)
            {
                cache = {};
            }
            // Mark all bands dirty to force coefficient recalculation
            dirtyBands = 0xFF;
        }
    };

    /**
     * @brief Multiband parametric equalizer with compile-time band count.
     *
     * Chains NumBands biquad filters in series. Each band can be independently
     * configured as any BiquadType (Lowpass, Highpass, Bandpass, Peak, LowShelf,
     * HighShelf, Notch, AllPass).
     *
     * ## Typical Usage
     * - 4-8 bands for general-purpose parametric EQ
     * - Band 0 as LowShelf, middle bands as Peak, last band as HighShelf
     *
     * ## Parameters per Band
     * - Frequency: 20-20000 Hz
     * - Q: 0.1-20 (bandwidth/resonance)
     * - Type: BiquadType (0-7)
     * - Gain: -24 to +24 dB (for Peak, LowShelf, HighShelf)
     *
     * ## Example
     * @code
     * MultibandEQ<8> eq;
     * eq.setBandType(0, BiquadType::LowShelf);
     * eq.setBandFrequency(0, 80.0f);
     * eq.setBandGainDb(0, 3.0f);
     * eq.updateCoefficients();
     * eq.process(buffer, numSamples);
     * @endcode
     *
     * ## Stereo Usage
     * @code
     * using StereoEQ = StereoExpander<MultibandEQ<8>::template Processor, StereoEffectConfig, Spec>;
     * @endcode
     *
     * @tparam NumBands Number of EQ bands (compile-time constant)
     * @tparam Spec ConstexprSpec for sample rate and block size
     */
    template <std::size_t NumBands = 8, ConstexprSpec Spec = DefaultSpec>
    class MultibandEQ : public ProcessorBase<MultibandEQ<NumBands, Spec>, MultibandEQConfig, MultibandEQState<NumBands>, Spec>
    {
    public:
        /// Alias for use with StereoExpander
        template <ConstexprSpec S>
        using Processor = MultibandEQ<NumBands, S>;

        /// Number of bands (compile-time)
        static constexpr std::size_t bandCount = NumBands;

        /**
         * @brief Set filter type for a specific band.
         */
        constexpr void setBandType (std::size_t band, BiquadType type) noexcept
        {
            if (band < NumBands)
            {
                float const newValue = static_cast<float> (type);
                if (!Math::exactlyEquals (this->state_.params.get (bandParamIndex (band, kBandType)), newValue))
                {
                    this->state_.params.set (bandParamIndex (band, kBandType), newValue);
                    this->state_.dirtyBands |= static_cast<std::uint8_t> (1u << band);
                }
            }
        }

        /**
         * @brief Get filter type for a specific band.
         */
        [[nodiscard]] constexpr BiquadType getBandType (std::size_t band) const noexcept
        {
            if (band < NumBands)
            {
                return static_cast<BiquadType> (static_cast<int> (this->state_.params.get (bandParamIndex (band, kBandType))));
            }
            return BiquadType::Peak;
        }

        /**
         * @brief Set frequency for a specific band.
         */
        constexpr void setBandFrequency (std::size_t band, float frequency) noexcept
        {
            if (band < NumBands)
            {
                if (!Math::exactlyEquals (this->state_.params.get (bandParamIndex (band, kBandFrequency)), frequency))
                {
                    this->state_.params.set (bandParamIndex (band, kBandFrequency), frequency);
                    this->state_.dirtyBands |= static_cast<std::uint8_t> (1u << band);
                }
            }
        }

        /**
         * @brief Get frequency for a specific band.
         */
        [[nodiscard]] constexpr float getBandFrequency (std::size_t band) const noexcept
        {
            if (band < NumBands)
            {
                return this->state_.params.get (bandParamIndex (band, kBandFrequency));
            }
            return 1000.0f;
        }

        /**
         * @brief Set Q factor for a specific band.
         */
        constexpr void setBandQ (std::size_t band, float q) noexcept
        {
            if (band < NumBands)
            {
                if (!Math::exactlyEquals (this->state_.params.get (bandParamIndex (band, kBandQ)), q))
                {
                    this->state_.params.set (bandParamIndex (band, kBandQ), q);
                    this->state_.dirtyBands |= static_cast<std::uint8_t> (1u << band);
                }
            }
        }

        /**
         * @brief Get Q factor for a specific band.
         */
        [[nodiscard]] constexpr float getBandQ (std::size_t band) const noexcept
        {
            if (band < NumBands)
            {
                return this->state_.params.get (bandParamIndex (band, kBandQ));
            }
            return 0.707f;
        }

        /**
         * @brief Set gain in dB for a specific band.
         */
        constexpr void setBandGainDb (std::size_t band, float gainDb) noexcept
        {
            if (band < NumBands)
            {
                if (!Math::exactlyEquals (this->state_.params.get (bandParamIndex (band, kBandGainDb)), gainDb))
                {
                    this->state_.params.set (bandParamIndex (band, kBandGainDb), gainDb);
                    this->state_.dirtyBands |= static_cast<std::uint8_t> (1u << band);
                }
            }
        }

        /**
         * @brief Get gain in dB for a specific band.
         */
        [[nodiscard]] constexpr float getBandGainDb (std::size_t band) const noexcept
        {
            if (band < NumBands)
            {
                return this->state_.params.get (bandParamIndex (band, kBandGainDb));
            }
            return 0.0f;
        }

        /**
         * @brief Configure all parameters for a band at once.
         */
        constexpr void configureBand (std::size_t band, BiquadType type, float frequency, float q, float gainDb = 0.0f) noexcept
        {
            setBandType (band, type);
            setBandFrequency (band, frequency);
            setBandQ (band, q);
            setBandGainDb (band, gainDb);
        }

        /**
         * @brief Update coefficients for all bands.
         *
         * Must be called after changing any parameters.
         */
        constexpr void updateCoefficients() noexcept
        {
            // Fast path: no dirty bands
            if (this->state_.dirtyBands == 0)
            {
                return;
            }

            std::uint8_t dirty = this->state_.dirtyBands;
            this->state_.dirtyBands = 0;

            // Only update dirty bands
            for (std::size_t band = 0; band < NumBands; ++band)
            {
                if (dirty & (1u << band))
                {
                    updateBandCoefficients (band);
                }
            }
        }

        /**
         * @brief Update coefficients for a single band.
         *
         * Uses coefficient hoisting to skip recalculation when parameters
         * haven't changed since the last update.
         */
        constexpr void updateBandCoefficients (std::size_t band) noexcept
        {
            if (band >= NumBands)
            {
                return;
            }

            double const sampleRate = this->getSampleRate().value;
            float const frequency = getBandFrequency (band);
            float const q = getBandQ (band);
            float const gainDb = getBandGainDb (band);
            BiquadType const type = getBandType (band);

            // Coefficient hoisting: skip recalculation if parameters unchanged
            auto& cache = this->state_.paramCache[band];
            if (Math::exactlyEquals (cache.frequency, frequency) && Math::exactlyEquals (cache.q, q) && Math::exactlyEquals (cache.gainDb, gainDb) && cache.type == type && Math::exactlyEquals (cache.sampleRate, sampleRate))
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

                case BiquadType::LowShelfSlope:
                {
                    float const A = Math::exp10_40 (gainDb);
                    float const S = q; // Slope parameter
                    float const sqrtArg = std::max (0.0f, (A + 1.0f / A) * (1.0f / S - 1.0f) + 2.0f);
                    float const alphaSlope = sinOmega * 0.5f * Math::sqrt (sqrtArg);
                    float const sqrtA = Math::sqrt (A);
                    float const sqrtA_alpha_2 = 2.0f * sqrtA * alphaSlope;
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

                case BiquadType::HighShelfSlope:
                {
                    float const A = Math::exp10_40 (gainDb);
                    float const S = q; // Slope parameter
                    float const sqrtArg = std::max (0.0f, (A + 1.0f / A) * (1.0f / S - 1.0f) + 2.0f);
                    float const alphaSlope = sinOmega * 0.5f * Math::sqrt (sqrtArg);
                    float const sqrtA = Math::sqrt (A);
                    float const sqrtA_alpha_2 = 2.0f * sqrtA * alphaSlope;
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
            }

            float const invA0 = 1.0f / a0;
            this->state_.coeffs[band].b0 = b0 * invA0;
            this->state_.coeffs[band].b1 = b1 * invA0;
            this->state_.coeffs[band].b2 = b2 * invA0;
            this->state_.coeffs[band].a1 = a1 * invA0;
            this->state_.coeffs[band].a2 = a2 * invA0;
        }

        /**
         * @brief Reset transient state (delay lines) while preserving coefficients.
         */
        constexpr void reset() noexcept
        {
            for (auto& d : this->state_.delayLines)
            {
                d = {};
            }
        }

        /**
         * @brief Get combined magnitude response at a specific frequency.
         *
         * Computes the product of all band magnitudes.
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

            float totalMagnitude = 1.0f;

            for (std::size_t band = 0; band < NumBands; ++band)
            {
                auto const& c = this->state_.coeffs[band];

                float const numReal = c.b0 + (c.b1 * cosW) + (c.b2 * cos2W);
                float const numImag = -(c.b1 * sinW) - (c.b2 * sin2W);
                float const denReal = 1.0f + (c.a1 * cosW) + (c.a2 * cos2W);
                float const denImag = -(c.a1 * sinW) - (c.a2 * sin2W);

                float const numMagSq = (numReal * numReal) + (numImag * numImag);
                float const denMagSq = (denReal * denReal) + (denImag * denImag);

                totalMagnitude *= Math::sqrt (numMagSq / denMagSq);
            }

            return totalMagnitude;
        }

        /**
         * @brief Get combined magnitude response in dB at a specific frequency.
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
         * @brief Process audio through all EQ bands in series.
         *
         * Uses sample-oriented processing for better cache locality:
         * each sample passes through all bands before moving to next sample.
         * Automatically detects parameter changes each sample and recalculates
         * coefficients when needed for sample-accurate automation response.
         */
        template <typename SampleType>
        constexpr void processImpl (BufferView<SampleType>& buffer, MultibandEQState<NumBands>& state, std::size_t sampleCount) noexcept
        {
            auto* audio = buffer.getWritePointer (0);

            for (std::size_t i = 0; i < sampleCount; ++i)
            {
                updateCoefficients();

                float sample = static_cast<float> (audio[i]);

                for (std::size_t band = 0; band < NumBands; ++band)
                {
                    auto const& c = state.coeffs[band];
                    auto& d = state.delayLines[band];

                    // Direct Form I: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
                    float const y = (c.b0 * sample) + (c.b1 * d.x1) + (c.b2 * d.x2) - (c.a1 * d.y1) - (c.a2 * d.y2);

                    d.x2 = d.x1;
                    d.x1 = sample;
                    d.y2 = d.y1;
                    d.y1 = y;

                    sample = y;
                }

                audio[i] = static_cast<SampleType> (sample);
            }
        }

    private:
        /**
         * @brief Calculate parameter index for a band parameter.
         */
        [[nodiscard]] static constexpr std::size_t bandParamIndex (std::size_t band, std::size_t paramOffset) noexcept
        {
            return (band * kNumParamsPerBand) + paramOffset;
        }
    };

} // namespace PlayfulTones::DspToolbox::Processors
