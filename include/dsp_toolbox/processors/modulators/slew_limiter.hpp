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

#include <algorithm>
#include <array>
#include <cstddef>

namespace PlayfulTones::DspToolbox::Modulators
{

    /**
     * @brief SlewLimiter IOConfig: 1 CV in, 1 CV out (in-place on channel 0).
     *
     * Buffer layout:
     * - Channel 0: Input / Output (shared, read-before-write)
     */
    using SlewLimiterConfig = IOConfig<0, 0, 1, 1>;

    /**
     * @brief Parameter indices for SlewLimiter.
     */
    enum SlewLimiterParamIndex : std::size_t {
        kRiseRate = 0,
        kFallRate = 1,
        kNumSlewLimiterParams
    };

    /**
     * @brief Parameter descriptors for SlewLimiter.
     */
    inline constexpr std::array<ParamDescriptor, kNumSlewLimiterParams> SlewLimiterParamDescriptors { { { "riseRate", "Rise Rate", 0.0001f, 1.0f, 0.01f, "/sample" },
        { "fallRate", "Fall Rate", 0.0001f, 1.0f, 0.01f, "/sample" } } };

    /**
     * @brief SlewLimiter unified state with embedded parameters.
     *
     * Contains both serializable parameters (in params) and transient
     * current value state.
     */
    struct SlewLimiterState
    {
        ParamSet<SlewLimiterParamDescriptors> params;

        float current { 0.0f };

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
            current = 0.0f;
        }
    };

    /**
     * @brief Rate-of-change limiter with separate rise and fall rates.
     *
     * Limits how fast a signal can change per sample. Useful for smoothing
     * jumpy CPU signals into portamento-style glides.
     *
     * ## Parameter Access
     * Use `setRiseRate(rate)` / `setFallRate(rate)` or index-based access.
     *
     * @tparam Spec Compile-time processor configuration
     */
    template <ConstexprSpec Spec = DefaultSpec>
    class SlewLimiter : public ProcessorBase<SlewLimiter<Spec>, SlewLimiterConfig, SlewLimiterState, Spec>
    {
    public:
        /**
         * @brief Set maximum rise rate (units per sample).
         */
        constexpr void setRiseRate (float rate) noexcept
        {
            this->template setParam<kRiseRate> (rate);
        }

        /**
         * @brief Set maximum fall rate (units per sample).
         */
        constexpr void setFallRate (float rate) noexcept
        {
            this->template setParam<kFallRate> (rate);
        }

        /**
         * @brief Get current output value.
         */
        [[nodiscard]] constexpr float getCurrent() const noexcept
        {
            return this->state_.current;
        }

        template <typename SampleType>
        constexpr void processImpl (BufferView<SampleType>& buffer, SlewLimiterState& state, std::size_t sampleCount) noexcept
        {
            auto* data = buffer.getWritePointer (0);
            float const riseRate = state.params.template get<kRiseRate>();
            float const fallRate = state.params.template get<kFallRate>();

            for (std::size_t i = 0; i < sampleCount; ++i)
            {
                float const target = static_cast<float> (data[i]);
                float const diff = target - state.current;

                if (diff > 0.0f)
                    state.current += std::min (diff, riseRate);
                else
                    state.current += std::max (diff, -fallRate);

                data[i] = static_cast<SampleType> (state.current);
            }
        }
    };

} // namespace PlayfulTones::DspToolbox::Modulators
