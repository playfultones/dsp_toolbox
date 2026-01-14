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
#include "dsp_toolbox/processors/filters/multiband_eq.hpp"

#include <array>
#include <cstddef>

namespace PlayfulTones::DspToolbox::Processors
{

    /**
     * @brief StereoMultibandEQ IOConfig: stereo audio in/out, no CV.
     *
     * Buffer layout: [Audio I/O]
     * - Channel 0: Left input/output (in-place processing)
     * - Channel 1: Right input/output (in-place processing)
     */
    using StereoMultibandEQConfig = IOConfig<2, 2, 0, 0>;

    /**
     * @brief Per-channel delay state for stereo processing.
     */
    struct StereoEQBandDelayState
    {
        float x1L { 0.0f }; // x[n-1] left
        float x2L { 0.0f }; // x[n-2] left
        float y1L { 0.0f }; // y[n-1] left
        float y2L { 0.0f }; // y[n-2] left

        float x1R { 0.0f }; // x[n-1] right
        float x2R { 0.0f }; // x[n-2] right
        float y1R { 0.0f }; // y[n-1] right
        float y2R { 0.0f }; // y[n-2] right
    };

    /**
     * @brief StereoMultibandEQ unified state.
     *
     * Contains shared coefficients (one set for both channels) and
     * separate delay lines per channel. Designed for maximum stereo
     * processing performance.
     *
     * @tparam NumBands Number of EQ bands
     */
    template <std::size_t NumBands>
    struct StereoMultibandEQState
    {
        /// Shared state flag - true because this is a native stereo processor
        static constexpr bool shared = true;

        /// Parameter descriptors (reuse from mono MultibandEQ)
        static constexpr auto descriptors = makeMultibandEQParamDescriptors<NumBands>();

        /// Serializable parameters (shared between channels)
        ParamSet<descriptors> params;

        /// Per-band coefficients (shared between L/R channels)
        std::array<BiquadCoefficients, NumBands> coeffs {};

        /// Per-band delay lines (interleaved L/R for cache locality)
        std::array<StereoEQBandDelayState, NumBands> delayLines {};

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
     * @brief Native stereo multiband parametric equalizer.
     *
     * Optimized stereo implementation that processes L/R channels together
     * with shared coefficients. Provides better performance than wrapping
     * mono MultibandEQ in StereoExpander by:
     *
     * - Single coefficient calculation for both channels
     * - Interleaved L/R delay state for cache locality
     * - No StereoExpander abstraction overhead
     * - Potential for SIMD L/R interleaving
     *
     * ## Usage
     * @code
     * StereoMultibandEQ<8> eq;
     * eq.configureBand(0, BiquadType::LowShelf, 80.0f, 0.707f, 3.0f);
     * eq.updateCoefficients();
     * eq.process(stereoBuffer, numSamples);
     * @endcode
     *
     * @tparam NumBands Number of EQ bands (compile-time constant)
     * @tparam Spec ConstexprSpec for sample rate and block size
     */
    template <std::size_t NumBands = 8, ConstexprSpec Spec = DefaultSpec>
    class StereoMultibandEQ : public ProcessorBase<StereoMultibandEQ<NumBands, Spec>, StereoMultibandEQConfig, StereoMultibandEQState<NumBands>, Spec>
    {
    public:
        /// Number of bands (compile-time)
        static constexpr std::size_t bandCount = NumBands;

        constexpr void setBandType (std::size_t band, BiquadType type) noexcept
        {
            if (band < NumBands)
            {
                float const newValue = static_cast<float> (type);
                if (this->state_.params.get (bandParamIndex (band, kBandType)) != newValue)
                {
                    this->state_.params.set (bandParamIndex (band, kBandType), newValue);
                    this->state_.dirtyBands |= static_cast<std::uint8_t> (1u << band);
                }
            }
        }

        [[nodiscard]] constexpr BiquadType getBandType (std::size_t band) const noexcept
        {
            if (band < NumBands)
            {
                return static_cast<BiquadType> (static_cast<int> (this->state_.params.get (bandParamIndex (band, kBandType))));
            }
            return BiquadType::Peak;
        }

        constexpr void setBandFrequency (std::size_t band, float frequency) noexcept
        {
            if (band < NumBands)
            {
                if (this->state_.params.get (bandParamIndex (band, kBandFrequency)) != frequency)
                {
                    this->state_.params.set (bandParamIndex (band, kBandFrequency), frequency);
                    this->state_.dirtyBands |= static_cast<std::uint8_t> (1u << band);
                }
            }
        }

        [[nodiscard]] constexpr float getBandFrequency (std::size_t band) const noexcept
        {
            if (band < NumBands)
            {
                return this->state_.params.get (bandParamIndex (band, kBandFrequency));
            }
            return 1000.0f;
        }

        constexpr void setBandQ (std::size_t band, float q) noexcept
        {
            if (band < NumBands)
            {
                if (this->state_.params.get (bandParamIndex (band, kBandQ)) != q)
                {
                    this->state_.params.set (bandParamIndex (band, kBandQ), q);
                    this->state_.dirtyBands |= static_cast<std::uint8_t> (1u << band);
                }
            }
        }

        [[nodiscard]] constexpr float getBandQ (std::size_t band) const noexcept
        {
            if (band < NumBands)
            {
                return this->state_.params.get (bandParamIndex (band, kBandQ));
            }
            return 0.707f;
        }

        constexpr void setBandGainDb (std::size_t band, float gainDb) noexcept
        {
            if (band < NumBands)
            {
                if (this->state_.params.get (bandParamIndex (band, kBandGainDb)) != gainDb)
                {
                    this->state_.params.set (bandParamIndex (band, kBandGainDb), gainDb);
                    this->state_.dirtyBands |= static_cast<std::uint8_t> (1u << band);
                }
            }
        }

        [[nodiscard]] constexpr float getBandGainDb (std::size_t band) const noexcept
        {
            if (band < NumBands)
            {
                return this->state_.params.get (bandParamIndex (band, kBandGainDb));
            }
            return 0.0f;
        }

        constexpr void configureBand (std::size_t band, BiquadType type, float frequency, float q, float gainDb = 0.0f) noexcept
        {
            setBandType (band, type);
            setBandFrequency (band, frequency);
            setBandQ (band, q);
            setBandGainDb (band, gainDb);
        }

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
            if (cache.frequency == frequency && cache.q == q && cache.gainDb == gainDb && cache.type == type && cache.sampleRate == sampleRate)
            {
                return;
            }

            // Update cache
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
            }

            // Normalize coefficients
            float const invA0 = 1.0f / a0;
            this->state_.coeffs[band].b0 = b0 * invA0;
            this->state_.coeffs[band].b1 = b1 * invA0;
            this->state_.coeffs[band].b2 = b2 * invA0;
            this->state_.coeffs[band].a1 = a1 * invA0;
            this->state_.coeffs[band].a2 = a2 * invA0;
        }

        constexpr void reset() noexcept
        {
            for (auto& d : this->state_.delayLines)
            {
                d = {};
            }
        }

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

        [[nodiscard]] constexpr float getMagnitudeDbAtFrequency (float frequencyHz) const noexcept
        {
            float const mag = getMagnitudeAtFrequency (frequencyHz);
            return 20.0f * Math::log (mag) / Math::ln10<float>;
        }

        /**
         * @brief Process stereo audio through all EQ bands.
         *
         * Processes L/R channels together with shared coefficients for
         * maximum cache efficiency and potential SIMD optimization.
         * Automatically detects parameter changes each sample and recalculates
         * coefficients when needed for sample-accurate automation response.
         */
        template <typename SampleType>
        constexpr void processImpl (BufferView<SampleType>& buffer, StereoMultibandEQState<NumBands>& state, std::size_t sampleCount) noexcept
        {
            auto* audioL = buffer.getWritePointer (0);
            auto* audioR = buffer.getWritePointer (1);

            for (std::size_t i = 0; i < sampleCount; ++i)
            {
                updateCoefficients();

                float sampleL = static_cast<float> (audioL[i]);
                float sampleR = static_cast<float> (audioR[i]);

                for (std::size_t band = 0; band < NumBands; ++band)
                {
                    auto const& c = state.coeffs[band];
                    auto& d = state.delayLines[band];

                    // Direct Form I: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
                    float const yL = (c.b0 * sampleL) + (c.b1 * d.x1L) + (c.b2 * d.x2L) - (c.a1 * d.y1L) - (c.a2 * d.y2L);
                    float const yR = (c.b0 * sampleR) + (c.b1 * d.x1R) + (c.b2 * d.x2R) - (c.a1 * d.y1R) - (c.a2 * d.y2R);

                    d.x2L = d.x1L;
                    d.x1L = sampleL;
                    d.y2L = d.y1L;
                    d.y1L = yL;

                    d.x2R = d.x1R;
                    d.x1R = sampleR;
                    d.y2R = d.y1R;
                    d.y1R = yR;

                    sampleL = yL;
                    sampleR = yR;
                }

                audioL[i] = static_cast<SampleType> (sampleL);
                audioR[i] = static_cast<SampleType> (sampleR);
            }
        }

    private:
        [[nodiscard]] static constexpr std::size_t bandParamIndex (std::size_t band, std::size_t paramOffset) noexcept
        {
            return (band * kNumParamsPerBand) + paramOffset;
        }
    };

} // namespace PlayfulTones::DspToolbox::Processors
