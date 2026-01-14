/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/core/buffer_view.hpp"
#include "dsp_toolbox/core/io_config.hpp"
#include "dsp_toolbox/processors/core/processor_base.hpp"

#include <type_traits>
#include <utility>
#include <variant>

namespace PlayfulTones::DspToolbox::Processors
{

    namespace detail
    {
        /**
         * @brief Detect if a State type declares shared=true.
         *
         * Detection priority:
         * 1. std::monostate -> true (stateless can share)
         * 2. State::shared member -> use its value
         * 3. Default -> false (separate state for safety)
         */
        template <typename State>
        constexpr bool detectStateShared()
        {
            if constexpr (std::is_same_v<State, std::monostate>)
            {
                return true; // Stateless processors can share
            }
            else if constexpr (requires { { State::shared } -> std::convertible_to<bool>; })
            {
                return State::shared; // Use processor's declared preference
            }
            else
            {
                return false; // Default: separate state for safety
            }
        }
    } // namespace detail

    /**
     * @brief State wrapper for StereoExpander.
     *
     * Chooses between shared or per-channel state based on the wrapped
     * processor's State::shared declaration.
     *
     * ## State Sharing Behavior
     * - `shared = true`: Single state instance for both L/R (linked stereo)
     * - `shared = false`: Separate L/R state instances (independent channels)
     *
     * @tparam MonoState The state type of the wrapped processor
     */
    template <typename MonoState>
    struct StereoExpanderState
    {
        /// Whether L/R channels share state (detected from MonoState::shared)
        static constexpr bool shared = detail::detectStateShared<MonoState>();

        /// Storage: single state if shared, pair if not
        std::conditional_t<shared, MonoState, std::pair<MonoState, MonoState>> storage {};

        /**
         * @brief Access left channel state.
         * @return Reference to left state (same as right if shared)
         */
        [[nodiscard]] constexpr MonoState& left() noexcept
        {
            if constexpr (shared)
            {
                return storage;
            }
            else
            {
                return storage.first;
            }
        }

        [[nodiscard]] constexpr const MonoState& left() const noexcept
        {
            if constexpr (shared)
            {
                return storage;
            }
            else
            {
                return storage.first;
            }
        }

        /**
         * @brief Access right channel state.
         * @return Reference to right state (same as left if shared)
         */
        [[nodiscard]] constexpr MonoState& right() noexcept
        {
            if constexpr (shared)
            {
                return storage; // Same instance for linked mode
            }
            else
            {
                return storage.second;
            }
        }

        [[nodiscard]] constexpr const MonoState& right() const noexcept
        {
            if constexpr (shared)
            {
                return storage;
            }
            else
            {
                return storage.second;
            }
        }

        constexpr void prepare (double sampleRate, std::size_t blockSize) noexcept
        {
            if constexpr (requires { left().prepare (sampleRate, blockSize); })
            {
                left().prepare (sampleRate, blockSize);
                if constexpr (!shared)
                {
                    right().prepare (sampleRate, blockSize);
                }
            }
        }

        constexpr void resetTransient() noexcept
        {
            if constexpr (requires { left().resetTransient(); })
            {
                left().resetTransient();
                if constexpr (!shared)
                {
                    right().resetTransient();
                }
            }
        }
    };

    /**
     * @brief Specialization for monostate (stateless processors).
     */
    template <>
    struct StereoExpanderState<std::monostate>
    {
        static constexpr bool shared = true;

        [[nodiscard]] static constexpr std::monostate left() noexcept { return {}; }
        [[nodiscard]] static constexpr std::monostate right() noexcept { return {}; }

        static constexpr void prepare (double, std::size_t) noexcept {}
        static constexpr void resetTransient() noexcept {}
    };

    /**
     * @brief Wraps a processor for stereo operation.
     *
     * StereoExpander automatically detects the appropriate mode based on the
     * processor's IOConfig and the target IOConfig:
     *
     * ## Modes (auto-detected)
     * - **DualMono**: Processor is mono (1→1), target is stereo (2→2)
     *   - Creates two processor instances for independent L/R processing
     * - **MonoToStereo**: Processor is 1→2, target is 1→2
     *   - Single processor instance producing stereo output
     * - **Passthrough**: Processor IOConfig matches target
     *   - Single processor instance handling full stereo buffer
     *
     * ## State Sharing
     * - `ProcessorType::State::shared = true`: Single state (linked stereo)
     * - `ProcessorType::State::shared = false`: Separate L/R state (default)
     *
     * ## Usage
     * @code
     * // Mono processor expanded to stereo (dual-mono mode)
     * using StereoGain = StereoExpander<Gain<Spec>, StereoEffectConfig>;
     *
     * // Mono-to-stereo panner (single instance, 1→2)
     * using MonoPanner = StereoExpander<Panner<MonoToStereoEffectConfig, Spec>>;
     *
     * // Apply parameters to both channels in dual-mono mode
     * StereoGain gain;
     * gain.forBoth([](auto& g) { g.setGain(0.5f); });
     * @endcode
     *
     * @tparam ProcessorType Instantiated processor type (e.g., Gain<Spec>)
     * @tparam TargetIO Target IOConfig (defaults to processor's IOConfig)
     * @tparam Spec ConstexprSpec for sample rate/block size
     */
    template <typename ProcessorType,
        IOConfigLike TargetIO = typename ProcessorType::IOConfig,
        ConstexprSpec Spec = DefaultSpec>
        requires StereoExpanderCompatibleIO<TargetIO>
    class StereoExpander : public ProcessorBase<
                               StereoExpander<ProcessorType, TargetIO, Spec>,
                               TargetIO,
                               StereoExpanderState<typename ProcessorType::State>,
                               Spec>
    {
    public:
        using ProcIO = typename ProcessorType::IOConfig;
        using MonoState = typename ProcessorType::State;
        using State = StereoExpanderState<MonoState>;
        using IOConfig = TargetIO;

        //----------------------------------------------------------------------
        // Mode detection (compile-time)
        //----------------------------------------------------------------------

        /// DualMono: Processor is mono (1→1), target is stereo (2→2) → two instances
        static constexpr bool kIsDualMono =
            (ProcIO::inAudio == 1 && ProcIO::outAudio == 1) && (TargetIO::inAudio == 2 && TargetIO::outAudio == 2);

        /// MonoToStereo: Processor is 1→2, target is 1→2 → single instance
        static constexpr bool kIsMonoToStereo =
            (ProcIO::inAudio == 1 && ProcIO::outAudio == 2) && (TargetIO::inAudio == 1 && TargetIO::outAudio == 2);

        /// Passthrough: Processor IOConfig matches target → single instance
        static constexpr bool kIsPassthrough = (ProcIO::inAudio == TargetIO::inAudio) && (ProcIO::outAudio == TargetIO::outAudio);

        static_assert (kIsDualMono || kIsMonoToStereo || kIsPassthrough,
            "Invalid IOConfig combination for StereoExpander. "
            "Supported: mono(1→1) to stereo(2→2), mono-to-stereo(1→2), or matching IOConfigs.");

        /// Processor template alias for ProcessorWrapper
        template <ConstexprSpec S>
        using Processor = StereoExpander<ProcessorType, TargetIO, S>;

    private:
        /// Storage: dual for dual-mono, single otherwise
        std::conditional_t<kIsDualMono, std::pair<ProcessorType, ProcessorType>, ProcessorType> processors_ {};

    public:
        //----------------------------------------------------------------------
        // Channel access (mode-dependent)
        //----------------------------------------------------------------------

        /**
         * @brief Get the single processor (non-dual-mono modes only).
         */
        [[nodiscard]] constexpr ProcessorType& getProcessor() noexcept
            requires (!kIsDualMono)
        {
            return processors_;
        }

        [[nodiscard]] constexpr const ProcessorType& getProcessor() const noexcept
            requires (!kIsDualMono)
        {
            return processors_;
        }

        /**
         * @brief Get the left channel processor (dual-mono mode only).
         */
        [[nodiscard]] constexpr ProcessorType& getLeftProcessor() noexcept
            requires kIsDualMono
        {
            return processors_.first;
        }

        [[nodiscard]] constexpr const ProcessorType& getLeftProcessor() const noexcept
            requires kIsDualMono
        {
            return processors_.first;
        }

        /**
         * @brief Get the right channel processor (dual-mono mode only).
         */
        [[nodiscard]] constexpr ProcessorType& getRightProcessor() noexcept
            requires kIsDualMono
        {
            return processors_.second;
        }

        [[nodiscard]] constexpr const ProcessorType& getRightProcessor() const noexcept
            requires kIsDualMono
        {
            return processors_.second;
        }

        /**
         * @brief Apply a function to both L/R processors (dual-mono mode only).
         *
         * Useful for parameter updates that should apply to both channels.
         *
         * @code
         * stereoEQ.forBoth([](auto& monoEQ) {
         *     monoEQ.setBandFrequency(0, 1000.0f);
         * });
         * @endcode
         */
        template <typename Func>
        constexpr void forBoth (Func&& func)
            requires kIsDualMono
        {
            func (processors_.first);
            func (processors_.second);
        }

        template <typename Func>
        constexpr void forBoth (Func&& func) const
            requires kIsDualMono
        {
            func (processors_.first);
            func (processors_.second);
        }

        //----------------------------------------------------------------------
        // Bypass forwarding
        //----------------------------------------------------------------------

        /**
         * @brief Set bypass for all processor instances.
         */
        constexpr void setBypass (bool bypass) noexcept
        {
            if constexpr (kIsDualMono)
            {
                processors_.first.setBypass (bypass);
                processors_.second.setBypass (bypass);
            }
            else
            {
                processors_.setBypass (bypass);
            }
        }

        //----------------------------------------------------------------------
        // Processing
        //----------------------------------------------------------------------

        /**
         * @brief Process buffer according to detected mode.
         *
         * - DualMono: Split buffer into L/R views, process independently
         * - MonoToStereo/Passthrough: Single processor handles full buffer
         */
        template <typename SampleType>
        constexpr void processImpl (BufferView<SampleType>& buffer, State& /*state*/, std::size_t sampleCount) noexcept
        {
            if constexpr (kIsDualMono)
            {
                // Two mono processors, L/R independent
                SampleType* leftChannel = buffer.getWritePointer (0);
                BufferView<SampleType> leftView (&leftChannel, 1, sampleCount);
                processors_.first.process (leftView, sampleCount);

                SampleType* rightChannel = buffer.getWritePointer (1);
                BufferView<SampleType> rightView (&rightChannel, 1, sampleCount);
                processors_.second.process (rightView, sampleCount);
            }
            else
            {
                // Single processor handles full buffer (mono-to-stereo or passthrough)
                processors_.process (buffer, sampleCount);
            }
        }
    };

} // namespace PlayfulTones::DspToolbox::Processors
