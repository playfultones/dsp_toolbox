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
#include <cstdint>

namespace PlayfulTones::DspToolbox::Generators
{

    /**
     * @brief WhiteNoise IOConfig: 1 audio output, no inputs.
     *
     * Buffer layout: [Audio out]
     * - Channel 0: White noise audio output
     */
    using WhiteNoiseConfig = IOConfig<0, 1, 0, 0>;

    /**
     * @brief Parameter indices for WhiteNoise.
     */
    enum WhiteNoiseParamIndex : std::size_t {
        kGain = 0,
        kNumWhiteNoiseParams
    };

    /**
     * @brief Parameter descriptors for WhiteNoise.
     */
    inline constexpr std::array<ParamDescriptor, kNumWhiteNoiseParams> WhiteNoiseParamDescriptors { { { "gain", "Gain", 0.0f, 2.0f, 1.0f, "" } } };

    /**
     * @brief WhiteNoise unified state with embedded parameters.
     *
     * Contains both serializable parameters (in params) and transient
     * PRNG state (seed).
     */
    struct WhiteNoiseState
    {
        // Serializable parameters
        ParamSet<WhiteNoiseParamDescriptors> params;

        // Transient state (reset on reset(), not serialized)
        std::uint32_t seed { 12345 };

        /**
         * @brief Prepare state for given sample rate.
         */
        constexpr void prepare (double sampleRate, std::size_t /*blockSize*/) noexcept
        {
            params.prepare (sampleRate);
            // Note: seed is preserved on prepare, only reset via resetTransient()
        }

        /**
         * @brief Reset transient state only (preserves params).
         */
        constexpr void resetTransient() noexcept
        {
            seed = 12345;
        }
    };

    /**
     * @brief White noise generator using xorshift PRNG.
     *
     * Generates pseudo-random white noise suitable for percussive
     * synthesis (snares, hi-hats) and texture generation.
     *
     * ## Algorithm
     * Uses xorshift32 for constexpr-compatible PRNG:
     * - Fast and deterministic
     * - Reasonable randomness quality for audio
     * - Fully constexpr (no stdlib random)
     *
     * ## Output
     * - Range: -1.0 to +1.0 (scaled by gain)
     * - Distribution: uniform (flat spectrum = white noise)
     *
     * ## Parameter Access
     * Use index-based access:
     * - `setParam<kGain>(0.5f)` - set output gain
     * - `getParam<kGain>()` - get current gain
     *
     * @tparam Spec Compile-time processor configuration
     */
    template <ConstexprSpec Spec = DefaultSpec>
    class WhiteNoise : public ProcessorBase<WhiteNoise<Spec>, WhiteNoiseConfig, WhiteNoiseState, Spec>
    {
    public:
        /**
         * @brief Set output gain.
         */
        constexpr void setGain (float gain) noexcept
        {
            this->template setParam<kGain> (gain);
        }

        /**
         * @brief Get current output gain.
         */
        [[nodiscard]] constexpr float getGain() const noexcept
        {
            return this->template getParam<kGain>();
        }

        /**
         * @brief Set PRNG seed (for reproducible sequences).
         */
        constexpr void setSeed (std::uint32_t seed) noexcept
        {
            this->state_.seed = (seed != 0) ? seed : 1; // Seed must be non-zero
        }

        /**
         * @brief Get current PRNG seed.
         */
        [[nodiscard]] constexpr std::uint32_t getSeed() const noexcept
        {
            return this->state_.seed;
        }

        template <typename SampleType>
        constexpr void processImpl (BufferView<SampleType>& buffer, WhiteNoiseState& state, std::size_t sampleCount) noexcept
        {
            auto* audioOut = buffer.getWritePointer (0);
            float const gain = state.params.get (kGain);

            for (std::size_t i = 0; i < sampleCount; ++i)
            {
                // xorshift32 PRNG
                state.seed ^= (state.seed << 13);
                state.seed ^= (state.seed >> 17);
                state.seed ^= (state.seed << 5);

                // Convert to float [-1, 1]
                // uint32 max is 4294967295, divide to get [0, 1), then scale to [-1, 1]
                float const normalized = static_cast<float> (state.seed) / 4294967296.0f; // [0, 1)
                float const sample = (normalized * 2.0f - 1.0f) * gain;

                audioOut[i] = static_cast<SampleType> (sample);
            }
        }
    };

} // namespace PlayfulTones::DspToolbox::Generators
