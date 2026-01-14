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
#include "dsp_toolbox/processors/core/processor_base.hpp"

#include <array>
#include <cmath>
#include <cstddef>

namespace PlayfulTones::DspToolbox::Processors
{

    //--------------------------------------------------------------------------
    // Panner IOConfig concept
    //--------------------------------------------------------------------------

    /**
     * @brief Concept for Panner-compatible IOConfigs.
     *
     * Valid configurations:
     * - 1 in, 2 out: Mono panning (positions mono source in stereo field)
     * - 2 in, 2 out: Stereo balance (adjusts L/R balance)
     */
    template <typename IO>
    concept PannerCompatibleIO = IOConfigLike<IO> && ((IO::inAudio == 1 && IO::outAudio == 2) || (IO::inAudio == 2 && IO::outAudio == 2));

    //--------------------------------------------------------------------------
    // Panner parameters
    //--------------------------------------------------------------------------

    /**
     * @brief Parameter indices for Panner.
     */
    enum PannerParamIndex : std::size_t {
        kPan = 0,
        kNumPannerParams
    };

    /**
     * @brief Parameter descriptors for Panner.
     *
     * Pan range: -1 (full left) to +1 (full right), 0 = center
     */
    inline constexpr std::array<ParamDescriptor, kNumPannerParams> PannerParamDescriptors { { { .id = "pan", .name = "Pan", .minValue = -1.0f, .maxValue = 1.0f, .defaultValue = 0.0f, .unit = "" } } };

    //--------------------------------------------------------------------------
    // Panner state
    //--------------------------------------------------------------------------

    /**
     * @brief Panner unified state.
     *
     * Note: `shared = true` because pan position applies to both channels
     * equally (linked stereo behavior).
     */
    struct PannerState
    {
        /// Shared state flag - pan position is shared across L/R
        static constexpr bool shared = true;

        /// Serializable parameters
        ParamSet<PannerParamDescriptors> params;

        /**
         * @brief Prepare state for given sample rate.
         */
        constexpr void prepare (double sampleRate, std::size_t /*blockSize*/) noexcept
        {
            params.prepare (sampleRate);
        }

        /**
         * @brief Reset transient state only.
         */
        constexpr void resetTransient() noexcept
        {
            // No transient state to reset
        }
    };

    //--------------------------------------------------------------------------
    // Static assertions for concept
    //--------------------------------------------------------------------------

    static_assert (PannerCompatibleIO<IOConfig<1, 2, 0, 0>>, "IOConfig<1,2,0,0> must satisfy PannerCompatibleIO");
    static_assert (PannerCompatibleIO<IOConfig<2, 2, 0, 0>>, "IOConfig<2,2,0,0> must satisfy PannerCompatibleIO");
    static_assert (PannerCompatibleIO<IOConfig<1, 2, 1, 0>>, "IOConfig<1,2,1,0> must satisfy PannerCompatibleIO (with CV)");

    //--------------------------------------------------------------------------
    // Panner processor
    //--------------------------------------------------------------------------

    /**
     * @brief Panner processor supporting mono-to-stereo and stereo balance.
     *
     * Automatically selects processing mode based on IOConfig:
     * - **Mono-to-stereo (1→2)**: Positions mono input in stereo field using
     *   equal-power panning for constant perceived loudness.
     * - **Stereo balance (2→2)**: Adjusts L/R balance by attenuating one side.
     *
     * ## Pan Parameter
     * - Range: -1 (full left) to +1 (full right)
     * - Center: 0 (equal L/R for mono panning, no change for balance)
     *
     * ## Usage
     * @code
     * // Mono-to-stereo panning
     * using MonoPanner = Panner<MonoToStereoEffectConfig, Spec>;
     * MonoPanner panner;
     * panner.setPan(-0.5f);  // Pan 50% left
     *
     * // With StereoExpander (for passthrough mode)
     * using StereoPanner = StereoExpander<Panner<MonoToStereoEffectConfig, Spec>>;
     * @endcode
     *
     * @tparam IO IOConfig satisfying PannerCompatibleIO
     * @tparam Spec ConstexprSpec for sample rate/block size
     */
    template <IOConfigLike IO, ConstexprSpec Spec = DefaultSpec>
        requires PannerCompatibleIO<IO>
    class Panner : public ProcessorBase<Panner<IO, Spec>, IO, PannerState, Spec>
    {
    public:
        /// Processor template alias for StereoExpander
        template <ConstexprSpec S>
        using Processor = Panner<IO, S>;

        /**
         * @brief Set pan position.
         *
         * @param pan Pan value: -1 (full left), 0 (center), +1 (full right)
         */
        constexpr void setPan (float pan) noexcept
        {
            this->state_.params.template setTarget<kPan> (pan);
        }

        /**
         * @brief Get current pan position.
         *
         * @return Current pan value (-1 to +1)
         */
        [[nodiscard]] constexpr float getPan() const noexcept
        {
            return this->template getParam<kPan>();
        }

        /**
         * @brief Process audio buffer.
         *
         * Dispatches to mono-to-stereo or balance processing based on IOConfig.
         */
        template <typename SampleType>
        constexpr void processImpl (BufferView<SampleType>& buffer, PannerState& state, std::size_t sampleCount) noexcept
        {
            if constexpr (IO::inAudio == 1 && IO::outAudio == 2)
            {
                processMonoToStereo (buffer, state, sampleCount);
            }
            else if constexpr (IO::inAudio == 2 && IO::outAudio == 2)
            {
                processStereoBalance (buffer, state, sampleCount);
            }
        }

    private:
        /**
         * @brief Process mono input to stereo output using equal-power panning.
         *
         * Uses cosine/sine for equal-power law, maintaining constant perceived
         * loudness across the stereo field.
         */
        template <typename SampleType>
        constexpr void processMonoToStereo (BufferView<SampleType>& buffer, PannerState& state, std::size_t sampleCount) noexcept
        {
            auto* monoIn = buffer.getWritePointer (0);
            auto* rightOut = buffer.getWritePointer (1);

            for (std::size_t i = 0; i < sampleCount; ++i)
            {
                float const pan = state.params.template tick<kPan>();
                SampleType const sample = monoIn[i];

                // Equal-power panning: map [-1, +1] to [0, pi/2] angle
                // left = cos(angle), right = sin(angle)
                float const angle = (pan + 1.0f) * 0.25f * Math::pi<float>;
                float const leftGain = std::cos (angle);
                float const rightGain = std::sin (angle);

                // Left output goes to channel 0 (in-place)
                monoIn[i] = static_cast<SampleType> (static_cast<float> (sample) * leftGain);
                // Right output goes to channel 1
                rightOut[i] = static_cast<SampleType> (static_cast<float> (sample) * rightGain);
            }
        }

        /**
         * @brief Process stereo input with balance adjustment.
         *
         * Attenuates one side based on pan position while leaving the other
         * at full volume. This maintains overall energy at extreme settings.
         */
        template <typename SampleType>
        constexpr void processStereoBalance (BufferView<SampleType>& buffer, PannerState& state, std::size_t sampleCount) noexcept
        {
            auto* left = buffer.getWritePointer (0);
            auto* right = buffer.getWritePointer (1);

            for (std::size_t i = 0; i < sampleCount; ++i)
            {
                float const pan = state.params.template tick<kPan>();

                // Balance: attenuate one side based on pan position
                // pan < 0: attenuate right, pan > 0: attenuate left
                float const leftGain = pan > 0.0f ? 1.0f - pan : 1.0f;
                float const rightGain = pan < 0.0f ? 1.0f + pan : 1.0f;

                left[i] = static_cast<SampleType> (static_cast<float> (left[i]) * leftGain);
                right[i] = static_cast<SampleType> (static_cast<float> (right[i]) * rightGain);
            }
        }
    };

} // namespace PlayfulTones::DspToolbox::Processors
