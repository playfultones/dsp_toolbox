/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/core/buffer_view.hpp"
#include "dsp_toolbox/core/constexpr_spec.hpp"
#include "dsp_toolbox/core/process_spec.hpp"
#include "dsp_toolbox/math/functions.hpp"
#include "dsp_toolbox/processors/core/processor_base.hpp"

#include <any>
#include <cstddef>
#include <optional>
#include <type_traits>
#include <utility>
#include <variant>

namespace PlayfulTones::DspToolbox
{

    /**
     * @brief Abstract interface for runtime-polymorphic processor access.
     *
     * IProcessor provides a virtual interface for DSP processors, enabling
     * runtime polymorphism for dynamic graph systems (RuntimeGraph). This
     * interface should ONLY be used where runtime flexibility is required.
     *
     * For static/zero-overhead processing, use ProcessorBase<Derived> directly.
     *
     * Key design decisions:
     * - Virtual functions ONLY for graph-level indirection
     * - Uses BufferView<float> specifically (not templated) for ABI stability
     * - Optional introspection methods have default implementations
     *
     * @see ProcessorBase for CRTP zero-overhead abstraction
     * @see ProcessorWrapper for type-erasing adapter from ProcessorBase to IProcessor
     */
    class IProcessor
    {
    public:
        virtual ~IProcessor() = default;

        /**
         * @brief Process audio samples in-place.
         *
         * Called from the audio thread. Must be real-time safe:
         * - No memory allocation
         * - No blocking operations
         * - No exceptions
         *
         * @param buffer Audio buffer to process in-place
         */
        virtual void process (BufferView<float>& buffer) noexcept = 0;

        /**
         * @brief Prepare processor for playback.
         *
         * Called before processing starts to configure for the current audio
         * context. May allocate resources if spec.allowDynamicAllocation is true.
         *
         * @param spec Runtime configuration parameters
         */
        virtual void prepare (const ProcessSpec& spec) = 0;

        /**
         * @brief Reset processor state.
         *
         * Clears any internal state (filter history, phase accumulators, etc.)
         * without changing configuration. Called when playback stops or seeks.
         */
        virtual void reset() noexcept = 0;

        /**
         * @brief Get processing latency in samples.
         *
         * For processors that introduce latency (lookahead limiters, FFT-based
         * effects, etc.). Graph systems use this for latency compensation.
         *
         * @return Latency in samples (default: 0)
         */
        [[nodiscard]] virtual std::size_t getLatencySamples() const noexcept { return 0; }

        /**
         * @brief Check if processor is currently active.
         *
         * Inactive processors may be skipped by graph systems for efficiency.
         * For example, a gate that's fully closed, or a bypassed effect.
         *
         * @return true if processor should be included in processing (default: true)
         */
        [[nodiscard]] virtual bool isActive() const noexcept { return true; }
    };

    /**
     * @brief Concept for states that support migration from type-erased storage.
     *
     * States satisfying this concept can be migrated across spec changes via
     * a static migrateFromAny() method. This enables ProcessorWrapper to preserve
     * meaningful state (parameters, ring buffers, etc.) when switching between
     * specs with different state types.
     *
     * @tparam State The state type to check
     */
    template <typename State>
    concept HasMigrateFromAny = requires (const std::any& any) {
        { State::migrateFromAny (any) } -> std::same_as<State>;
    };

    /**
     * @brief Multi-spec type-erasing wrapper from ProcessorBase<T> to IProcessor.
     *
     * ProcessorWrapper adapts any CRTP-based processor template to the IProcessor
     * virtual interface, supporting multiple ConstexprSpec configurations at runtime.
     * When prepare() is called, the wrapper selects the appropriate pre-instantiated
     * variant based on the runtime ProcessSpec.
     *
     * ## Multi-Spec Support
     *
     * The processor template is instantiated for each spec in the SpecSet. At runtime,
     * prepare() matches the ProcessSpec to the best variant:
     * - Sample rate must match exactly
     * - Block size must be <= the spec's block size (can process smaller blocks)
     *
     * If no matching spec is found, the processor becomes unsupported (no-op processing).
     *
     * ## State Migration
     *
     * When prepare() switches between specs, state is migrated:
     * - State (same type): Trivially copied, then re-prepared for new sample rate
     * - State (different type): Reset to default, or custom migration via migrateStateFrom()
     *
     * Note: Parameters are embedded in State in the unified state pattern.
     *
     * ## Processor Definition
     *
     * Processors must be templates taking ConstexprSpec as parameter:
     * @code
     * template<ConstexprSpec Spec = DefaultSpec>
     * class MyFilter : public ProcessorBase<MyFilter<Spec>, StereoConfig, MyState, Spec> {
     *     // ...
     *
     *     // Optional: custom state migration for spec-dependent state
     *     template<ConstexprSpec FromSpec>
     *     static constexpr MyState migrateStateFrom(const typename MyFilter<FromSpec>::State& old);
     * };
     * @endcode
     *
     * ## Usage
     * @code
     * ProcessorWrapper<MyFilter> wrapped;
     *
     * // Prepare selects the right variant
     * wrapped.prepare(ProcessSpec{.sampleRate = 48000.0, .maxBlockSize = 512});
     *
     * if (!wrapped.isSupported()) {
     *     // Handle unsupported config
     * }
     *
     * // Process through IProcessor interface
     * wrapped.process(buffer);
     *
     * // Access underlying processor
     * wrapped.visit([](auto& proc) {
     *     proc.getState();  // Access state (which contains params)
     * });
     * @endcode
     *
     * @tparam ProcessorTemplate A class template taking ConstexprSpec, derived from ProcessorBase
     * @tparam SpecSetT A SpecSet<Specs...> defining which specs to instantiate (default: StandardSpecs)
     */
    template <template <ConstexprSpec> typename ProcessorTemplate, typename SpecSetT = StandardSpecs>
    class ProcessorWrapper;

    // Partial specialization to extract Specs from SpecSet
    template <template <ConstexprSpec> typename ProcessorTemplate, ConstexprSpec... Specs>
    class ProcessorWrapper<ProcessorTemplate, SpecSet<Specs...>> : public IProcessor
    {
    public:
        /** @brief Variant type holding all spec instantiations plus monostate for unsupported */
        using VariantType = std::variant<std::monostate, ProcessorTemplate<Specs>...>;

        // Use DefaultSpec to query shared types (State is spec-independent for simple cases)
        using ReferenceProcessor = ProcessorTemplate<DefaultSpec>;
        using StateType = typename ReferenceProcessor::State;

        /**
         * @brief Default construct in unsupported state.
         *
         * Call prepare() to select and construct the appropriate variant.
         */
        ProcessorWrapper() = default;

        //----------------------------------------------------------------------
        // IProcessor implementation
        //----------------------------------------------------------------------

        /**
         * @brief Process audio samples through the active variant.
         *
         * If no supported spec was found in prepare(), this is a no-op.
         *
         * @param buffer Audio buffer to process in-place
         */
        void process (BufferView<float>& buffer) noexcept override
        {
            std::visit (
                [&buffer] (auto& proc) {
                    if constexpr (!std::is_same_v<std::decay_t<decltype (proc)>, std::monostate>)
                    {
                        proc.process (buffer, buffer.getNumSamples());
                    }
                    // monostate = unsupported, no-op
                },
                processor_);
        }

        /**
         * @brief Prepare processor for the given spec.
         *
         * Matches the runtime spec to a pre-instantiated variant:
         * - Sample rate must match exactly
         * - Block size must be <= spec's block size
         *
         * If switching specs, state is migrated to the new variant:
         * - Exact type match: trivial copy
         * - Different types with migrateFromAny(): custom migration
         * - Otherwise: state remains default-initialized
         *
         * If no matching spec is found, the processor becomes unsupported.
         *
         * @param spec Runtime configuration parameters
         */
        void prepare (const ProcessSpec& spec) override
        {
            // Capture current state as type-erased std::any
            std::any migratedStateAny;

            std::visit (
                [&] (auto& proc) {
                    if constexpr (!std::is_same_v<std::decay_t<decltype (proc)>, std::monostate>)
                    {
                        using ProcState = typename std::decay_t<decltype (proc)>::State;
                        if constexpr (!std::is_same_v<ProcState, std::monostate>)
                        {
                            migratedStateAny = proc.getState();
                        }
                    }
                },
                processor_);

            // Try to find and construct matching variant
            processor_ = std::monostate {};
            (tryEmplace<Specs> (spec) || ...); // Fold expression - stops on first match

            // Migrate state to new processor
            if (migratedStateAny.has_value())
            {
                std::visit (
                    [&] (auto& proc) {
                        if constexpr (!std::is_same_v<std::decay_t<decltype (proc)>, std::monostate>)
                        {
                            using NewState = typename std::decay_t<decltype (proc)>::State;
                            if constexpr (!std::is_same_v<NewState, std::monostate>)
                            {
                                // Try exact type match first
                                if (auto* exact = std::any_cast<NewState> (&migratedStateAny))
                                {
                                    proc.setState (*exact);
                                }
                                // Try custom migration via migrateFromAny
                                else if constexpr (HasMigrateFromAny<NewState>)
                                {
                                    proc.setState (NewState::migrateFromAny (migratedStateAny));
                                }
                                // else: No migration possible, state remains default-initialized
                            }
                        }
                    },
                    processor_);
            }
        }

        /**
         * @brief Reset processor state.
         *
         * Clears internal state without changing configuration.
         * No-op if processor is unsupported.
         */
        void reset() noexcept override
        {
            std::visit (
                [] (auto& proc) {
                    if constexpr (!std::is_same_v<std::decay_t<decltype (proc)>, std::monostate>)
                    {
                        proc.reset();
                    }
                },
                processor_);
        }

        /**
         * @brief Get processing latency in samples.
         *
         * @return Latency from active processor, or 0 if unsupported
         */
        [[nodiscard]] std::size_t getLatencySamples() const noexcept override
        {
            return std::visit (
                [] (const auto& proc) -> std::size_t {
                    if constexpr (!std::is_same_v<std::decay_t<decltype (proc)>, std::monostate>)
                    {
                        return proc.getLatencySamples();
                    }
                    return 0;
                },
                processor_);
        }

        /**
         * @brief Check if a supported spec was found.
         *
         * @return true if prepare() found a matching spec, false if unsupported
         */
        [[nodiscard]] bool isSupported() const noexcept
        {
            return !std::holds_alternative<std::monostate> (processor_);
        }

        /**
         * @brief Visit the underlying processor variant.
         *
         * Allows access to processor-specific methods. The visitor receives
         * either std::monostate (if unsupported) or the active processor.
         *
         * @tparam Visitor Callable accepting auto& parameter
         * @param visitor Function to apply to the active processor
         * @return Result of visitor invocation
         */
        template <typename Visitor>
        decltype (auto) visit (Visitor&& visitor)
        {
            return std::visit (std::forward<Visitor> (visitor), processor_);
        }

        /**
         * @brief Visit the underlying processor variant (const).
         *
         * @tparam Visitor Callable accepting const auto& parameter
         * @param visitor Function to apply to the active processor
         * @return Result of visitor invocation
         */
        template <typename Visitor>
        decltype (auto) visit (Visitor&& visitor) const
        {
            return std::visit (std::forward<Visitor> (visitor), processor_);
        }

        /**
         * @brief Get pointer to processor if it matches the given spec.
         *
         * @tparam Spec The ConstexprSpec to query
         * @return Pointer to processor if active and matches Spec, nullptr otherwise
         */
        template <ConstexprSpec Spec>
        [[nodiscard]] ProcessorTemplate<Spec>* get() noexcept
        {
            return std::get_if<ProcessorTemplate<Spec>> (&processor_);
        }

        /**
         * @brief Get const pointer to processor if it matches the given spec.
         *
         * @tparam Spec The ConstexprSpec to query
         * @return Const pointer to processor if active and matches Spec, nullptr otherwise
         */
        template <ConstexprSpec Spec>
        [[nodiscard]] const ProcessorTemplate<Spec>* get() const noexcept
        {
            return std::get_if<ProcessorTemplate<Spec>> (&processor_);
        }

    private:
        VariantType processor_;

        /**
         * @brief Try to construct a variant for the given spec if it matches.
         *
         * For RuntimeSpec, always matches and calls prepareRuntime() to store values.
         * For static specs, sample rate must match exactly and block size must be <=.
         *
         * @tparam Candidate The ConstexprSpec to try
         * @param spec Runtime spec to match against
         * @return true if matched and constructed, false otherwise
         */
        template <ConstexprSpec Candidate>
        bool tryEmplace (const ProcessSpec& spec)
        {
            if constexpr (kIsRuntimeSpec<Candidate>)
            {
                // RuntimeSpec always matches - it's the universal fallback
                auto& proc = processor_.template emplace<ProcessorTemplate<Candidate>>();
                proc.prepareRuntime (spec);
                return true;
            }
            else
            {
                // Static spec: sample rate must match exactly
                // Block size must be <= candidate's block size (can process smaller blocks)
                if (Math::exactlyEquals (spec.sampleRate.value, Candidate.sampleRate.value) && spec.maxBlockSize.value <= Candidate.blockSize.value)
                {
                    processor_.template emplace<ProcessorTemplate<Candidate>>();
                    return true;
                }
                return false;
            }
        }
    };

} // namespace PlayfulTones::DspToolbox
