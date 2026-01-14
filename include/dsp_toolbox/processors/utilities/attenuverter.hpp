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

namespace PlayfulTones::DspToolbox::Processors
{

    /**
     * @brief Attenuverter IOConfig: 1 CV in, 1 CV out.
     *
     * Buffer layout: [CV I/O]
     * - Channel 0: CV input/output (read-before-write)
     */
    using AttenuverterConfig = IOConfig<0, 0, 1, 1>;

    /**
     * @brief Parameter indices for Attenuverter.
     */
    enum AttenuverterParamIndex : std::size_t {
        kAmount = 0,
        kNumAttenuverterParams
    };

    /**
     * @brief Parameter descriptors for Attenuverter.
     */
    inline constexpr std::array<ParamDescriptor, kNumAttenuverterParams> AttenuverterParamDescriptors { { { "amount", "Amount", -2.0f, 2.0f, 1.0f, "" } } };

    /**
     * @brief Attenuverter unified state with embedded parameters.
     *
     * Contains serializable parameters only (no transient state).
     */
    struct AttenuverterState
    {
        // Serializable parameters
        ParamSet<AttenuverterParamDescriptors> params;

        /**
         * @brief Prepare state for given sample rate.
         */
        constexpr void prepare (double sampleRate, std::size_t /*blockSize*/) noexcept
        {
            params.prepare (sampleRate);
        }

        /**
         * @brief Reset transient state only (no transient state).
         */
        constexpr void resetTransient() noexcept
        {
            // No transient state
        }
    };

    /**
     * @brief CV signal attenuator/inverter.
     *
     * Scales CV signals by a configurable amount. Essential modular building
     * block for scaling envelope outputs before modulation destinations.
     *
     * ## CV Convention
     * - amount > 0 → scale (1.0 = unity, 2.0 = double)
     * - amount < 0 → invert and scale
     * - amount = 0 → output zero
     *
     * ## Buffer Layout
     * - Channel 0: CV input (read), CV output (write)
     *
     * ## Parameter Access
     * Use index-based access:
     * - `setParam<kAmount>(0.5f)` - set scaling amount
     * - `getParam<kAmount>()` - get current amount
     *
     * ## Use Cases
     * - Scale envelope 0→1 to pitch modulation 0→2 octaves
     * - Invert modulation polarity
     * - Attenuate LFO depth
     *
     * @tparam Spec Compile-time processor configuration
     */
    template <ConstexprSpec Spec = DefaultSpec>
    class Attenuverter : public ProcessorBase<Attenuverter<Spec>, AttenuverterConfig, AttenuverterState, Spec>
    {
    public:
        /**
         * @brief Set scaling amount.
         *
         * @param amount Scale factor (can be negative for inversion)
         */
        constexpr void setAmount (float amount) noexcept
        {
            this->template setParam<kAmount> (amount);
        }

        /**
         * @brief Get current scaling amount.
         */
        [[nodiscard]] constexpr float getAmount() const noexcept
        {
            return this->template getParam<kAmount>();
        }

        template <typename SampleType>
        constexpr void processImpl (BufferView<SampleType>& buffer, AttenuverterState& state, std::size_t sampleCount) noexcept
        {
            auto* cv = buffer.getWritePointer (0);
            float const amount = state.params.get (kAmount);

            for (std::size_t i = 0; i < sampleCount; ++i)
            {
                cv[i] = static_cast<SampleType> (static_cast<float> (cv[i]) * amount);
            }
        }
    };

} // namespace PlayfulTones::DspToolbox::Processors
