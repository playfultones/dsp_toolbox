/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/core/buffer_view.hpp"
#include "dsp_toolbox/core/constexpr_spec.hpp"
#include "dsp_toolbox/core/io_config.hpp"
#include "dsp_toolbox/core/param.hpp"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <variant>

namespace PlayfulTones::DspToolbox
{

    //----------------------------------------------------------------------
    // Predefined ConstexprSpec presets for ProcessorBase
    //----------------------------------------------------------------------

    /** @brief Default spec: 48kHz, 512 samples */
    inline constexpr ConstexprSpec DefaultSpec {};

    /** @brief CD quality: 44.1kHz, 512 samples */
    inline constexpr ConstexprSpec Spec44100 { .sampleRate = SampleRate { 44100.0 } };

    /** @brief High sample rate: 96kHz, 512 samples */
    inline constexpr ConstexprSpec Spec96000 { .sampleRate = SampleRate { 96000.0 } };

    /** @brief Low latency: 48kHz, 64 samples */
    inline constexpr ConstexprSpec LowLatencySpec { .sampleRate = SampleRate { 48000.0 }, .blockSize = Samples<std::uint32_t> { 64 } };

    //----------------------------------------------------------------------
    // Multi-spec support: Specs with various block sizes for runtime flexibility
    //----------------------------------------------------------------------

    // 44.1kHz variants
    inline constexpr ConstexprSpec Spec44100_64 { .sampleRate = SampleRate { 44100.0 }, .blockSize = Samples<std::uint32_t> { 64 } };
    inline constexpr ConstexprSpec Spec44100_128 { .sampleRate = SampleRate { 44100.0 }, .blockSize = Samples<std::uint32_t> { 128 } };
    inline constexpr ConstexprSpec Spec44100_256 { .sampleRate = SampleRate { 44100.0 }, .blockSize = Samples<std::uint32_t> { 256 } };
    inline constexpr ConstexprSpec Spec44100_512 { .sampleRate = SampleRate { 44100.0 }, .blockSize = Samples<std::uint32_t> { 512 } };
    inline constexpr ConstexprSpec Spec44100_1024 { .sampleRate = SampleRate { 44100.0 }, .blockSize = Samples<std::uint32_t> { 1024 } };
    inline constexpr ConstexprSpec Spec44100_2048 { .sampleRate = SampleRate { 44100.0 }, .blockSize = Samples<std::uint32_t> { 2048 } };

    // 48kHz variants
    inline constexpr ConstexprSpec Spec48000_64 { .sampleRate = SampleRate { 48000.0 }, .blockSize = Samples<std::uint32_t> { 64 } };
    inline constexpr ConstexprSpec Spec48000_128 { .sampleRate = SampleRate { 48000.0 }, .blockSize = Samples<std::uint32_t> { 128 } };
    inline constexpr ConstexprSpec Spec48000_256 { .sampleRate = SampleRate { 48000.0 }, .blockSize = Samples<std::uint32_t> { 256 } };
    inline constexpr ConstexprSpec Spec48000_512 { .sampleRate = SampleRate { 48000.0 }, .blockSize = Samples<std::uint32_t> { 512 } };
    inline constexpr ConstexprSpec Spec48000_1024 { .sampleRate = SampleRate { 48000.0 }, .blockSize = Samples<std::uint32_t> { 1024 } };
    inline constexpr ConstexprSpec Spec48000_2048 { .sampleRate = SampleRate { 48000.0 }, .blockSize = Samples<std::uint32_t> { 2048 } };

    // 96kHz variants
    inline constexpr ConstexprSpec Spec96000_64 { .sampleRate = SampleRate { 96000.0 }, .blockSize = Samples<std::uint32_t> { 64 } };
    inline constexpr ConstexprSpec Spec96000_128 { .sampleRate = SampleRate { 96000.0 }, .blockSize = Samples<std::uint32_t> { 128 } };
    inline constexpr ConstexprSpec Spec96000_256 { .sampleRate = SampleRate { 96000.0 }, .blockSize = Samples<std::uint32_t> { 256 } };
    inline constexpr ConstexprSpec Spec96000_512 { .sampleRate = SampleRate { 96000.0 }, .blockSize = Samples<std::uint32_t> { 512 } };
    inline constexpr ConstexprSpec Spec96000_1024 { .sampleRate = SampleRate { 96000.0 }, .blockSize = Samples<std::uint32_t> { 1024 } };
    inline constexpr ConstexprSpec Spec96000_2048 { .sampleRate = SampleRate { 96000.0 }, .blockSize = Samples<std::uint32_t> { 2048 } };

    /**
     * @brief Tag type to hold a set of ConstexprSpec values for ProcessorWrapper.
     *
     * Used to define which spec variants a ProcessorWrapper should instantiate.
     *
     * @tparam Specs The ConstexprSpec values to include in the set
     */
    template <ConstexprSpec... Specs>
    struct SpecSet
    {
    };

    /** @brief Standard spec set: 44.1kHz, 48kHz, 96kHz with 1024 block size */
    using StandardSpecs = SpecSet<Spec44100_1024, Spec48000_1024, Spec96000_1024>;

    /**
     * @brief Extended spec set for desktop/JUCE: common sample rates with various block sizes.
     *
     * Covers 18 combinations: 44.1kHz, 48kHz, 96kHz x 64, 128, 256, 512, 1024, 2048 block sizes.
     */
    using DesktopSpecs = SpecSet<
        Spec44100_64,
        Spec44100_128,
        Spec44100_256,
        Spec44100_512,
        Spec44100_1024,
        Spec44100_2048,
        Spec48000_64,
        Spec48000_128,
        Spec48000_256,
        Spec48000_512,
        Spec48000_1024,
        Spec48000_2048,
        Spec96000_64,
        Spec96000_128,
        Spec96000_256,
        Spec96000_512,
        Spec96000_1024,
        Spec96000_2048>;

    /**
     * @brief Desktop specs with RuntimeSpec fallback for any unsupported configuration.
     *
     * Tries static specs first for optimal performance, falls back to RuntimeSpec
     * for unusual sample rates (88.2kHz, 176.4kHz, etc.) or block sizes.
     */
    using DesktopSpecsWithFallback = SpecSet<
        Spec44100_64,
        Spec44100_128,
        Spec44100_256,
        Spec44100_512,
        Spec44100_1024,
        Spec44100_2048,
        Spec48000_64,
        Spec48000_128,
        Spec48000_256,
        Spec48000_512,
        Spec48000_1024,
        Spec48000_2048,
        Spec96000_64,
        Spec96000_128,
        Spec96000_256,
        Spec96000_512,
        Spec96000_1024,
        Spec96000_2048,
        RuntimeSpec>;

    /**
     * @brief CRTP base class for zero-overhead processor abstraction (v2).
     *
     * Provides the common interface for all DSP processors without virtual dispatch.
     * Processors are parameterized by their IOConfig for compile-time channel verification.
     *
     * ## Template Parameters
     *
     * | Parameter | Default | Description |
     * |-----------|---------|-------------|
     * | `Derived` | - | The derived class (CRTP pattern) |
     * | `IO` | `DefaultIOConfig` | I/O channel configuration |
     * | `StateT` | `std::monostate` | State struct type (may contain params) |
     * | `Spec` | `DefaultSpec` | Static sample rate and block size |
     *
     * ## Derived Class Requirements
     *
     * Derived classes must implement:
     * - `processImpl()`: `void processImpl(BufferView&, State&, std::size_t)`
     *
     * The base class owns a `state_` member. Use `[[no_unique_address]]`
     * automatically provides zero overhead when using `std::monostate`.
     *
     * ## Unified State Pattern
     *
     * Parameters should be embedded in the State struct via a ParamSet member:
     * @code
     * struct MyState {
     *     ParamSet<MyParamDescriptors> params;  // Serializable params
     *     float transientValue{};               // Transient state
     *
     *     void prepare(double sampleRate, std::size_t blockSize) noexcept {
     *         params.prepare(sampleRate);
     *         resetTransient();
     *     }
     *
     *     void resetTransient() noexcept {
     *         transientValue = 0.0f;  // Reset transient only
     *     }
     * };
     * @endcode
     *
     * ## Automatic Preparation at Construction
     *
     * If `State` has a `prepare()` method, it will be called automatically
     * at construction time (constexpr if the prepare method is constexpr).
     *
     * The `reset()` method calls `state_.resetTransient()` if available,
     * preserving parameters while clearing transient state.
     *
     * ## Example Usage (Stateful with Params)
     * @code
     * struct DCBlockerState {
     *     ParamSet<DCBlockerDescriptors> params;
     *     float x1{}, y1{};  // Transient
     *
     *     void prepare(double sr, std::size_t) { params.prepare(sr); resetTransient(); }
     *     void resetTransient() { x1 = y1 = 0.0f; }
     * };
     *
     * class DCBlocker : public ProcessorBase<DCBlocker, MonoConfig, DCBlockerState> {
     * public:
     *     template<typename SampleType>
     *     constexpr void processImpl(BufferView<SampleType>& buffer,
     *                                 DCBlockerState& state, std::size_t sampleCount) noexcept {
     *         auto* audio = buffer.getWritePointer(0);
     *         constexpr float R = 0.995f;
     *         for (std::size_t i = 0; i < sampleCount; ++i) {
     *             float x0 = audio[i];
     *             float y0 = x0 - state.x1 + R * state.y1;
     *             state.x1 = x0;
     *             state.y1 = y0;
     *             audio[i] = y0;
     *         }
     *     }
     * };
     * @endcode
     *
     * ## Example Usage (Stateless)
     * @code
     * class Passthrough : public ProcessorBase<Passthrough, MonoConfig> {
     * public:
     *     template<typename SampleType>
     *     constexpr void processImpl(BufferView<SampleType>&,
     *                                 std::monostate&, std::size_t) noexcept { }
     * };
     * @endcode
     *
     * @tparam Derived The derived class (CRTP pattern)
     * @tparam IO I/O configuration (default: DefaultIOConfig)
     * @tparam StateT State struct type (default: std::monostate)
     * @tparam Spec Static processor configuration (default: DefaultSpec)
     *
     * @note NO virtual functions - use ProcessorWrapper for runtime polymorphism.
     * @see ProcessorWrapper for type-erasing wrapper when polymorphism is needed.
     * @see IOConfig for channel configuration
     * @see ConstexprSpec for compile-time configuration
     */
    template <typename Derived, IOConfigLike IO = DefaultIOConfig, typename StateT = std::monostate, ConstexprSpec Spec = DefaultSpec>
    class ProcessorBase
    {
    public:
        /** @brief I/O configuration type */
        using IOConfig = IO;

        /** @brief State type */
        using State = StateT;

        //----------------------------------------------------------------------
        // Construction
        //----------------------------------------------------------------------

        /**
         * @brief Default constructor - prepares state automatically.
         *
         * Since sample rate and block size are compile-time constants from Spec,
         * preparation happens at construction (constexpr if State allows).
         */
        constexpr ProcessorBase() noexcept
        {
            prepareState();
        }

        //----------------------------------------------------------------------
        // Core processing
        //----------------------------------------------------------------------

        /**
         * @brief Process a buffer of audio samples.
         *
         * Dispatches to the derived class's processImpl() with State.
         * If bypassed, passes audio through unchanged (effects) or outputs silence (generators).
         *
         * @param buffer Audio buffer to process in-place
         * @param sampleCount Number of samples to process
         */
        template <typename SampleType = float>
        constexpr void process (BufferView<SampleType>& buffer, std::size_t sampleCount) noexcept
        {
            assert (buffer.getNumChannels() >= IO::totalChannels);

            if (bypassed_)
            {
                handleBypass (buffer, sampleCount);
                return;
            }

            derived().processImpl (buffer, state_, sampleCount);
        }

        /**
         * @brief Process a buffer using buffer's sample count.
         *
         * Convenience overload that uses buffer.getNumSamples().
         *
         * @param buffer Audio buffer to process in-place
         */
        template <typename SampleType = float>
        constexpr void process (BufferView<SampleType>& buffer) noexcept
        {
            process (buffer, buffer.getNumSamples());
        }

        //----------------------------------------------------------------------
        // Bypass control
        //----------------------------------------------------------------------

        /**
         * @brief Enable or disable bypass.
         *
         * When bypassed:
         * - Effects (inAudio > 0): pass audio through unchanged
         * - Generators (inAudio == 0): output silence
         *
         * When bypass is enabled (false -> true transition), processor state
         * is reset to prevent stale state from affecting audio when bypass
         * is later disabled.
         *
         * @param bypass true to bypass, false to process normally
         */
        constexpr void setBypass (bool bypass) noexcept
        {
            if (bypass && !bypassed_)
            {
                reset();
            }
            bypassed_ = bypass;
        }

        /**
         * @brief Check if processor is bypassed.
         */
        [[nodiscard]] constexpr bool isBypassed() const noexcept { return bypassed_; }

        /**
         * @brief Reset processor state (transient only, preserves params).
         *
         * Clears any internal state (filter history, phase accumulators, etc.)
         * without changing parameters. Called when playback stops or seeks.
         *
         * For stateless processors (State = std::monostate), this is a no-op.
         * If State has a resetTransient() method, that is called.
         * Otherwise, params are cached, state is reset via default construction,
         * params are restored, and state is re-prepared.
         */
        constexpr void reset() noexcept
        {
            if constexpr (!std::is_same_v<StateT, std::monostate>)
            {
                if constexpr (requires { state_.resetTransient(); })
                {
                    state_.resetTransient();
                }
                else
                {
                    // Cache params before reset if state has them
                    if constexpr (HasParams<StateT>)
                    {
                        auto cachedParams = state_.params;
                        state_ = {};
                        state_.params = cachedParams;
                    }
                    else
                    {
                        state_ = {};
                    }

                    if constexpr (requires { state_.prepare (getSampleRate().value, getMaxBlockSize().value); })
                    {
                        state_.prepare (getSampleRate().value, getMaxBlockSize().value);
                    }
                }
            }
        }

        /**
         * @brief Full reset (clears everything including params).
         *
         * Resets state to default-constructed values and re-prepares.
         * Unlike reset(), this also resets parameters to their defaults.
         */
        constexpr void resetAll() noexcept
        {
            state_ = {};
            prepareState();
        }

        //----------------------------------------------------------------------
        // Parameter access (for states with embedded ParamSet)
        //----------------------------------------------------------------------

        /**
         * @brief Set parameter value by compile-time index.
         *
         * Works when State has a params member satisfying ParamSetLike.
         *
         * @tparam Index Parameter index (bounds checked at compile time)
         * @param value Target value
         */
        template <std::size_t Index>
        constexpr void setParam (float value) noexcept
        {
            if constexpr (HasParams<StateT>)
            {
                state_.params.template set<Index> (value);
            }
        }

        /**
         * @brief Get parameter value by compile-time index.
         *
         * @tparam Index Parameter index (bounds checked at compile time)
         * @return Current parameter value
         */
        template <std::size_t Index>
        [[nodiscard]] constexpr float getParam() const noexcept
        {
            if constexpr (HasParams<StateT>)
            {
                return state_.params.template get<Index>();
            }
            else
            {
                return 0.0f;
            }
        }

        /**
         * @brief Set parameter value by runtime index.
         *
         * @param index Parameter index
         * @param value Target value
         */
        constexpr void setParam (std::size_t index, float value) noexcept
        {
            if constexpr (HasParams<StateT>)
            {
                state_.params.set (index, value);
            }
        }

        /**
         * @brief Get parameter value by runtime index.
         *
         * @param index Parameter index
         * @return Current parameter value
         */
        [[nodiscard]] constexpr float getParam (std::size_t index) const noexcept
        {
            if constexpr (HasParams<StateT>)
            {
                return state_.params.get (index);
            }
            else
            {
                return 0.0f;
            }
        }

        /**
         * @brief Get number of parameters.
         *
         * @return Number of parameters (0 if no ParamSet)
         */
        [[nodiscard]] static constexpr std::size_t numParams() noexcept
        {
            if constexpr (HasParams<StateT>)
            {
                return std::remove_cvref_t<decltype (std::declval<StateT>().params)>::NumParams;
            }
            else
            {
                return 0;
            }
        }

        //----------------------------------------------------------------------
        // Accessors
        //----------------------------------------------------------------------

        /**
         * @brief Get configured sample rate.
         *
         * For static specs, returns compile-time constant.
         * For RuntimeSpec, returns value set via prepareRuntime().
         */
        [[nodiscard]] constexpr SampleRate<double> getSampleRate() const noexcept
        {
            if constexpr (kIsRuntimeSpec<Spec>)
            {
                return runtimeSpec_.sampleRate;
            }
            else
            {
                return Spec.sampleRate;
            }
        }

        /**
         * @brief Get configured maximum block size.
         *
         * For static specs, returns compile-time constant.
         * For RuntimeSpec, returns value set via prepareRuntime().
         */
        [[nodiscard]] constexpr Samples<std::uint32_t> getMaxBlockSize() const noexcept
        {
            if constexpr (kIsRuntimeSpec<Spec>)
            {
                return runtimeSpec_.blockSize;
            }
            else
            {
                return Spec.blockSize;
            }
        }

        /**
         * @brief Get processing latency in samples.
         *
         * Used for latency compensation in graph systems.
         */
        [[nodiscard]] constexpr std::size_t getLatencySamples() const noexcept
        {
            return latencySamples_;
        }

        /**
         * @brief Get tail length in samples.
         *
         * Used for effects with release tails (reverb, delay).
         */
        [[nodiscard]] constexpr std::size_t getTailSamples() const noexcept
        {
            return tailSamples_;
        }

        //----------------------------------------------------------------------
        // State accessors (for migration between specs)
        //----------------------------------------------------------------------

        /**
         * @brief Get const reference to state.
         *
         * Used by ProcessorWrapper for state migration when switching specs.
         */
        [[nodiscard]] constexpr const State& getState() const noexcept { return state_; }

        /**
         * @brief Set state from external source and re-prepare.
         *
         * Copies state and calls prepare() with this spec's sample rate and
         * block size to recalculate any sample-rate dependent values.
         *
         * @param state State to copy
         */
        constexpr void setState (const State& state) noexcept
        {
            state_ = state;
            if constexpr (!std::is_same_v<StateT, std::monostate>)
            {
                if constexpr (requires { state_.prepare (getSampleRate().value, getMaxBlockSize().value); })
                {
                    state_.prepare (getSampleRate().value, getMaxBlockSize().value);
                }
            }
        }

        //----------------------------------------------------------------------
        // RuntimeSpec support
        //----------------------------------------------------------------------

        /**
         * @brief Prepare processor with runtime-provided spec values.
         *
         * Called by ProcessorWrapper when using RuntimeSpec variant.
         * Stores the sample rate and block size, then prepares state.
         *
         * @param spec Runtime configuration from JUCE prepareToPlay or similar
         */
        constexpr void prepareRuntime (const ProcessSpec& spec) noexcept
        {
            if constexpr (kIsRuntimeSpec<Spec>)
            {
                runtimeSpec_.sampleRate = spec.sampleRate;
                runtimeSpec_.blockSize = spec.maxBlockSize;
            }
            // Prepare state with the (now stored) runtime values
            if constexpr (!std::is_same_v<StateT, std::monostate>)
            {
                if constexpr (requires { state_.prepare (getSampleRate().value, getMaxBlockSize().value); })
                {
                    state_.prepare (getSampleRate().value, getMaxBlockSize().value);
                }
            }
        }

    protected:
        [[no_unique_address]] StateT state_ {};

        // Zero overhead when using static spec (monostate is empty)
        // 16 bytes when using RuntimeSpec to store sample rate and block size
        [[no_unique_address]] std::conditional_t<kIsRuntimeSpec<Spec>, RuntimeSpecStorage, std::monostate> runtimeSpec_ {};

        std::size_t latencySamples_ { 0 };
        std::size_t tailSamples_ { 0 };
        bool bypassed_ { false };

        //----------------------------------------------------------------------
        // Protected setters for derived classes
        //----------------------------------------------------------------------

        /**
         * @brief Set processing latency (call from State::prepare if needed).
         */
        constexpr void setLatency (std::size_t samples) noexcept
        {
            latencySamples_ = samples;
        }

        /**
         * @brief Set tail length (call from State::prepare if needed).
         */
        constexpr void setTail (std::size_t samples) noexcept
        {
            tailSamples_ = samples;
        }

    private:
        //----------------------------------------------------------------------
        // CRTP helpers
        //----------------------------------------------------------------------

        [[nodiscard]] constexpr Derived& derived() noexcept
        {
            return static_cast<Derived&> (*this);
        }

        [[nodiscard]] constexpr const Derived& derived() const noexcept
        {
            return static_cast<const Derived&> (*this);
        }

        /**
         * @brief Prepare state if it has a prepare() method.
         *
         * For static specs, uses compile-time constant values.
         * For RuntimeSpec, uses default values from runtimeSpec_ member
         * (which will be updated later via prepareRuntime()).
         */
        constexpr void prepareState() noexcept
        {
            if constexpr (!std::is_same_v<StateT, std::monostate>)
            {
                if constexpr (requires { state_.prepare (getSampleRate().value, getMaxBlockSize().value); })
                {
                    state_.prepare (getSampleRate().value, getMaxBlockSize().value);
                }
            }
        }

        /**
         * @brief Handle bypass for different IOConfig types.
         *
         * - Generators (inAudio == 0): output silence
         * - Mono-to-stereo (outAudio > inAudio): copy first input to extra outputs
         * - Effects (inAudio == outAudio): in-place, nothing to do
         */
        template <typename SampleType>
        constexpr void handleBypass (BufferView<SampleType>& buffer, std::size_t sampleCount) noexcept
        {
            if constexpr (IO::inAudio == 0)
            {
                // Generator: output silence
                for (std::size_t ch = 0; ch < IO::outAudio; ++ch)
                {
                    auto* ptr = buffer.getWritePointer (ch);
                    for (std::size_t i = 0; i < sampleCount; ++i)
                    {
                        ptr[i] = SampleType {};
                    }
                }
            }
            else if constexpr (IO::outAudio > IO::inAudio)
            {
                // Mono-to-stereo: copy first input to extra outputs
                auto const* src = buffer.getReadPointer (0);
                for (std::size_t ch = IO::inAudio; ch < IO::outAudio; ++ch)
                {
                    auto* dst = buffer.getWritePointer (ch);
                    for (std::size_t i = 0; i < sampleCount; ++i)
                    {
                        dst[i] = static_cast<SampleType> (src[i]);
                    }
                }
            }
            // else: in-place effect, input already in output location - nothing to do
        }
    };

} // namespace PlayfulTones::DspToolbox
