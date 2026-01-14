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

namespace PlayfulTones::DspToolbox::Modulators
{

    /**
     * @brief ADEnvelope IOConfig: 1 CV in (trigger), 1 CV out (envelope).
     *
     * Buffer layout: [CV I/O]
     * - Channel 0: Trigger input / Envelope output (shared, read-before-write)
     *
     * The graph wires the trigger CV to channel 0 before processing.
     * The envelope reads the trigger, then writes the envelope value to
     * the same channel for downstream nodes.
     */
    using ADEnvelopeConfig = IOConfig<0, 0, 1, 1>;

    /**
     * @brief Envelope phase state machine.
     */
    enum class ADEnvelopePhase {
        Idle,
        Attack,
        Decay
    };

    /**
     * @brief Parameter indices for ADEnvelope.
     */
    enum ADEnvelopeParamIndex : std::size_t {
        kAttackSamples = 0,
        kDecaySamples = 1,
        kNumADEnvelopeParams
    };

    /**
     * @brief Parameter descriptors for ADEnvelope.
     */
    inline constexpr std::array<ParamDescriptor, kNumADEnvelopeParams> ADEnvelopeParamDescriptors { { { "attackSamples", "Attack", 0.0f, 48000.0f, 48.0f, "samples" },
        { "decaySamples", "Decay", 0.0f, 480000.0f, 7200.0f, "samples" } } };

    /**
     * @brief ADEnvelope unified state with embedded parameters.
     *
     * Contains both serializable parameters (in params) and transient
     * envelope state (phase, position, edge detection).
     */
    struct ADEnvelopeState
    {
        // Serializable parameters
        ParamSet<ADEnvelopeParamDescriptors> params;

        // Transient state (reset on reset(), not serialized)
        ADEnvelopePhase phase { ADEnvelopePhase::Idle };
        std::size_t sampleIndex { 0 };
        float currentValue { 0.0f };
        float previousTrigger { 0.0f };

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
            phase = ADEnvelopePhase::Idle;
            sampleIndex = 0;
            currentValue = 0.0f;
            previousTrigger = 0.0f;
        }
    };

    /**
     * @brief Attack-Decay envelope generator with trigger CV input.
     *
     * Detects rising edge on trigger input and generates a linear
     * attack-decay envelope:
     * - Attack: ramps from 0 to 1 over attackSamples
     * - Decay: ramps from 1 to 0 over decaySamples
     * - Idle: outputs 0 until next trigger
     *
     * ## CV Convention
     * - Trigger input: > 0.5 is considered high, rising edge triggers envelope
     * - Envelope output: 0.0 to 1.0 unipolar CV
     *
     * ## Parameter Access
     * Use index-based access:
     * - `setParam<kAttackSamples>(480.0f)` - set attack time
     * - `setParam<kDecaySamples>(4800.0f)` - set decay time
     *
     * @tparam Spec Compile-time processor configuration
     */
    template <ConstexprSpec Spec = DefaultSpec>
    class ADEnvelope : public ProcessorBase<ADEnvelope<Spec>, ADEnvelopeConfig, ADEnvelopeState, Spec>
    {
    public:
        /**
         * @brief Set attack time in samples.
         */
        constexpr void setAttackSamples (std::size_t samples) noexcept
        {
            this->template setParam<kAttackSamples> (static_cast<float> (samples));
        }

        /**
         * @brief Set decay time in samples.
         */
        constexpr void setDecaySamples (std::size_t samples) noexcept
        {
            this->template setParam<kDecaySamples> (static_cast<float> (samples));
        }

        /**
         * @brief Get current envelope value.
         */
        [[nodiscard]] constexpr float getCurrentValue() const noexcept
        {
            return this->state_.currentValue;
        }

        /**
         * @brief Get current envelope phase.
         */
        [[nodiscard]] constexpr ADEnvelopePhase getPhase() const noexcept
        {
            return this->state_.phase;
        }

        /**
         * @brief Manually trigger the envelope (for non-CV use).
         */
        constexpr void trigger() noexcept
        {
            this->state_.phase = ADEnvelopePhase::Attack;
            this->state_.sampleIndex = 0;
        }

        template <typename SampleType>
        constexpr void processImpl (BufferView<SampleType>& buffer, ADEnvelopeState& state, std::size_t sampleCount) noexcept
        {
            auto* cv = buffer.getWritePointer (0);

            std::size_t const attackSamples = static_cast<std::size_t> (state.params.get (kAttackSamples));
            std::size_t const decaySamples = static_cast<std::size_t> (state.params.get (kDecaySamples));

            for (std::size_t i = 0; i < sampleCount; ++i)
            {
                float const triggerIn = static_cast<float> (cv[i]);

                bool const risingEdge = (triggerIn > 0.5f) && (state.previousTrigger <= 0.5f);
                state.previousTrigger = triggerIn;

                if (risingEdge)
                {
                    state.phase = ADEnvelopePhase::Attack;
                    state.sampleIndex = 0;
                }

                float value = 0.0f;

                switch (state.phase)
                {
                    case ADEnvelopePhase::Idle:
                        value = 0.0f;
                        break;

                    case ADEnvelopePhase::Attack:
                        if (attackSamples > 0)
                        {
                            value = static_cast<float> (state.sampleIndex) / static_cast<float> (attackSamples);
                        }
                        else
                        {
                            value = 1.0f;
                        }

                        ++state.sampleIndex;
                        if (state.sampleIndex >= attackSamples)
                        {
                            state.phase = ADEnvelopePhase::Decay;
                            state.sampleIndex = 0;
                        }
                        break;

                    case ADEnvelopePhase::Decay:
                        if (decaySamples > 0)
                        {
                            float const decayProgress = static_cast<float> (state.sampleIndex) / static_cast<float> (decaySamples);
                            value = 1.0f - decayProgress;
                        }
                        else
                        {
                            value = 0.0f;
                        }

                        ++state.sampleIndex;
                        if (state.sampleIndex >= decaySamples)
                        {
                            state.phase = ADEnvelopePhase::Idle;
                            state.sampleIndex = 0;
                        }
                        break;
                }

                state.currentValue = value;
                cv[i] = static_cast<SampleType> (value);
            }
        }
    };

} // namespace PlayfulTones::DspToolbox::Modulators
