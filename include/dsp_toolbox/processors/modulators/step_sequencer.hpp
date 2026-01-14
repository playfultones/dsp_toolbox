/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/core/buffer_view.hpp"
#include "dsp_toolbox/core/conversions.hpp"
#include "dsp_toolbox/core/io_config.hpp"
#include "dsp_toolbox/core/param.hpp"
#include "dsp_toolbox/processors/core/processor_base.hpp"

#include <array>
#include <cstddef>

namespace PlayfulTones::DspToolbox::Modulators
{

    /**
     * @brief Single step in a sequence.
     */
    struct Step
    {
        float pitchCv { 0.0f }; ///< 1V/Oct pitch CV (0 = base, 1 = +1 octave)
        bool gate { true }; ///< Whether this step triggers a note
    };

    /**
     * @brief StepSequencer IOConfig: 1 CV in (clock), 2 CV out (pitch, gate).
     *
     * Buffer layout: [Clock in -> Pitch CV out, Gate out]
     * - Channel 0: Clock input (read) / Pitch CV output (write) - read-before-write
     * - Channel 1: Gate output
     */
    using StepSequencerConfig = IOConfig<0, 0, 1, 2>;

    /**
     * @brief Parameter indices for StepSequencer.
     */
    enum StepSequencerParamIndex : std::size_t {
        kNumSteps = 0,
        kNumStepSequencerParams
    };

    /**
     * @brief Parameter descriptors for StepSequencer.
     *
     * Note: MaxSteps is set to 64 for the descriptor range, but actual limit
     * is determined by the template parameter.
     */
    inline constexpr std::array<ParamDescriptor, kNumStepSequencerParams> StepSequencerParamDescriptors { {
        { "numSteps", "Steps", 0.0f, 64.0f, 0.0f, "" } // 0 = use MaxSteps
    } };

    /**
     * @brief StepSequencer unified state with embedded parameters.
     *
     * Contains both serializable parameters (in params) and transient
     * sequencer state (current step, edge detection).
     */
    struct StepSequencerState
    {
        // Serializable parameters
        ParamSet<StepSequencerParamDescriptors> params;

        // Transient state (reset on reset(), not serialized)
        std::size_t currentStep { 0 };
        float prevTrigger { 0.0f };

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
            currentStep = 0;
            prevTrigger = 0.0f;
        }
    };

    /**
     * @brief Step sequencer that outputs pitch CV and gate on clock triggers.
     *
     * Advances through a sequence of steps on each incoming clock pulse.
     * Outputs 1V/Oct pitch CV and gate signals for downstream VCO/envelope.
     *
     * ## CV Convention
     * - Clock input: > 0.5 triggers rising edge detection
     * - Pitch CV output: 1V/Oct (0 = no transposition, 1 = +1 octave)
     * - Gate output: 1.0 if step has gate=true, 0.0 otherwise
     *
     * ## Parameter Access
     * Use index-based access:
     * - `setParam<kNumSteps>(8.0f)` - set number of active steps
     *
     * ## Usage
     * @code
     * constexpr std::array<Step, 4> melody = {{
     *     { semitonesToCv(0), true },   // Root note
     *     { semitonesToCv(3), true },   // +3 semitones (minor third)
     *     { semitonesToCv(7), true },   // +7 semitones (fifth)
     *     { 0.0f, false }               // Rest (gate off)
     * }};
     *
     * StepSequencer<4, Spec> seq;
     * seq.setSteps(melody);
     * @endcode
     *
     * @tparam MaxSteps Maximum number of steps (compile-time size)
     * @tparam Spec Compile-time processor configuration
     */
    template <std::size_t MaxSteps, ConstexprSpec Spec = DefaultSpec>
    class StepSequencer : public ProcessorBase<StepSequencer<MaxSteps, Spec>,
                              StepSequencerConfig,
                              StepSequencerState,
                              Spec>
    {
    public:
        static_assert (MaxSteps > 0, "StepSequencer requires at least 1 step");

        /**
         * @brief Set all steps from an array.
         */
        constexpr void setSteps (std::array<Step, MaxSteps> const& steps) noexcept
        {
            steps_ = steps;
            this->template setParam<kNumSteps> (static_cast<float> (MaxSteps));
        }

        /**
         * @brief Set a single step.
         *
         * @param index Step index (0-based)
         * @param step Step data
         */
        constexpr void setStep (std::size_t index, Step step) noexcept
        {
            if (index < MaxSteps)
            {
                steps_[index] = step;
            }
        }

        /**
         * @brief Set a step using semitones for pitch.
         *
         * @param index Step index (0-based)
         * @param semitones Pitch offset in semitones
         * @param gate Whether this step triggers a note
         */
        constexpr void setStep (std::size_t index, int semitones, bool gate = true) noexcept
        {
            if (index < MaxSteps)
            {
                steps_[index] = Step { semitonesToCv (semitones), gate };
            }
        }

        /**
         * @brief Set number of active steps (for patterns shorter than MaxSteps).
         *
         * @param num Number of steps to use (clamped to MaxSteps)
         */
        constexpr void setNumSteps (std::size_t num) noexcept
        {
            this->template setParam<kNumSteps> (static_cast<float> ((num <= MaxSteps) ? num : MaxSteps));
        }

        /**
         * @brief Get current step index.
         */
        [[nodiscard]] constexpr std::size_t getCurrentStep() const noexcept
        {
            return this->state_.currentStep;
        }

        /**
         * @brief Get maximum number of steps.
         */
        [[nodiscard]] static constexpr std::size_t getMaxSteps() noexcept
        {
            return MaxSteps;
        }

        /**
         * @brief Get active number of steps.
         */
        [[nodiscard]] constexpr std::size_t getNumSteps() const noexcept
        {
            std::size_t const n = static_cast<std::size_t> (this->template getParam<kNumSteps>());
            return (n > 0) ? n : MaxSteps;
        }

        /**
         * @brief Get step data at index.
         */
        [[nodiscard]] constexpr Step getStep (std::size_t index) const noexcept
        {
            return (index < MaxSteps) ? steps_[index] : Step {};
        }

        template <typename SampleType>
        constexpr void processImpl (BufferView<SampleType>& buffer,
            StepSequencerState& state,
            std::size_t sampleCount) noexcept
        {
            auto* ch0 = buffer.getWritePointer (0); // Clock in -> Pitch out
            auto* gateOut = buffer.getWritePointer (1);

            std::size_t const n = static_cast<std::size_t> (state.params.get (kNumSteps));
            std::size_t const numSteps = (n > 0) ? n : MaxSteps;

            for (std::size_t i = 0; i < sampleCount; ++i)
            {
                float const clockIn = static_cast<float> (ch0[i]);

                // Rising edge detection
                bool const risingEdge = (clockIn > 0.5f) && (state.prevTrigger <= 0.5f);
                state.prevTrigger = clockIn;

                if (risingEdge)
                {
                    // Advance to next step (wrapping)
                    state.currentStep = (state.currentStep + 1) % numSteps;
                }

                // Output current step's values
                Step const& step = steps_[state.currentStep];
                ch0[i] = static_cast<SampleType> (step.pitchCv);
                gateOut[i] = step.gate ? SampleType (1) : SampleType (0);
            }
        }

    private:
        std::array<Step, MaxSteps> steps_ {};
    };

} // namespace PlayfulTones::DspToolbox::Modulators
