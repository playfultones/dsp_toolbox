/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/core/buffer_view.hpp"
#include "dsp_toolbox/core/io_config.hpp"
#include "dsp_toolbox/processors/core/processor_base.hpp"

#include <cstddef>

namespace PlayfulTones::DspToolbox::Processors
{

    /**
     * @brief VCA IOConfig: 1 audio in/out, 1 CV in (amplitude).
     *
     * Buffer layout: [Audio I/O, Amplitude CV in]
     * - Channel 0: Audio input/output (in-place processing)
     * - Channel 1: Amplitude CV input (0.0-1.0 typical, but can exceed)
     */
    using VCAConfig = IOConfig<1, 1, 1, 0>;

    /**
     * @brief Voltage Controlled Amplifier.
     *
     * Multiplies audio input by CV amplitude control.
     * Simple but essential modular building block.
     *
     * ## CV Convention
     * - CV = 0.0 → silence
     * - CV = 1.0 → unity gain (full volume)
     * - CV > 1.0 → boost (can clip)
     *
     * ## Buffer Layout
     * - Channel 0: Audio input (overwritten with output)
     * - Channel 1: Amplitude CV (wired from envelope or other modulator)
     *
     * @tparam Spec Compile-time processor configuration
     */
    template <ConstexprSpec Spec = DefaultSpec>
    class VCA : public ProcessorBase<VCA<Spec>, VCAConfig, std::monostate, Spec>
    {
    public:
        template <typename SampleType>
        constexpr void processImpl (BufferView<SampleType>& buffer, std::monostate&, std::size_t sampleCount) noexcept
        {
            auto* audio = buffer.getWritePointer (0);
            auto const* ampCV = buffer.getReadPointer (1);

            for (std::size_t i = 0; i < sampleCount; ++i)
            {
                audio[i] *= static_cast<SampleType> (ampCV[i]);
            }
        }
    };

} // namespace PlayfulTones::DspToolbox::Processors
