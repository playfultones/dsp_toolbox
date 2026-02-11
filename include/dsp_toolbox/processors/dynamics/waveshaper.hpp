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
#include <type_traits>

namespace PlayfulTones::DspToolbox::Processors
{

    /**
     * @brief Concept for waveshaper transfer functions.
     *
     * A transfer function must be callable with a float and return a float,
     * and must be noexcept for real-time safety.
     *
     * | Member | Type | Description |
     * |--------|------|-------------|
     * | `operator()(float)` | `float` | Applies the transfer function to a single sample |
     *
     * ## Example Implementation
     * @code
     * struct TanhShaper {
     *     constexpr float operator()(float x) const noexcept {
     *         return Math::tanh(x);
     *     }
     * };
     * static_assert(WaveshaperFunction<TanhShaper>);
     * @endcode
     */
    template <typename F>
    concept WaveshaperFunction = requires (F const f, float x) {
        { f (x) } noexcept -> std::convertible_to<float>;
    };

    /**
     * @brief Waveshaper IOConfig: mono audio in/out, no CV.
     *
     * Buffer layout: [Audio I/O]
     * - Channel 0: Audio input/output (in-place processing)
     */
    using WaveshaperConfig = IOConfig<1, 1, 0, 0>;

    /**
     * @brief Waveshaper state.
     *
     * Stores the transfer function instance. For stateless functors,
     * [[no_unique_address]] ensures zero overhead.
     *
     * @tparam TransferFunction The waveshaper function type
     */
    template <WaveshaperFunction TransferFunction>
    struct WaveshaperState
    {
        [[no_unique_address]] TransferFunction transferFunction {};

        constexpr void prepare (double /*sampleRate*/, std::size_t /*blockSize*/) noexcept
        {
        }

        constexpr void resetTransient() noexcept
        {
        }
    };

    /**
     * @brief Generic static nonlinear processor (waveshaper).
     *
     * Applies a user-provided memoryless transfer function to each sample.
     * The transfer function is a template parameter for zero-overhead
     * inlining — the compiler can fully inline the function into the
     * sample loop.
     *
     * ## Transfer Function
     * Any callable satisfying the WaveshaperFunction concept:
     * - Must accept a float and return a float
     * - Must be noexcept (real-time safe)
     * - Should be a pure function (same input -> same output)
     *
     * ## Usage
     * @code
     * // Define a transfer function
     * struct HardClip {
     *     float ceiling = 1.0f;
     *     constexpr float operator()(float x) const noexcept {
     *         return (x > ceiling) ? ceiling : (x < -ceiling) ? -ceiling : x;
     *     }
     * };
     *
     * // Create waveshaper
     * Waveshaper<HardClip> clipper;
     * clipper.process(buffer, numSamples);
     *
     * // Or with custom parameters
     * Waveshaper<HardClip, Spec48000_512> clipper2;
     * clipper2.getTransferFunction().ceiling = 0.5f;
     * @endcode
     *
     * @tparam TransferFunction Callable type satisfying WaveshaperFunction
     * @tparam Spec Compile-time processor configuration
     */
    template <WaveshaperFunction TransferFunction, ConstexprSpec Spec = DefaultSpec>
    class Waveshaper : public ProcessorBase<Waveshaper<TransferFunction, Spec>, WaveshaperConfig, WaveshaperState<TransferFunction>, Spec>
    {
    public:
        /**
         * @brief Processor template alias for StereoExpander and Oversampler.
         */
        template <ConstexprSpec S>
        using Processor = Waveshaper<TransferFunction, S>;

        /**
         * @brief Get mutable reference to the transfer function.
         *
         * Allows runtime configuration of transfer function parameters.
         *
         * @return Reference to the transfer function instance
         */
        [[nodiscard]] constexpr TransferFunction& getTransferFunction() noexcept
        {
            return this->state_.transferFunction;
        }

        /**
         * @brief Get const reference to the transfer function.
         */
        [[nodiscard]] constexpr TransferFunction const& getTransferFunction() const noexcept
        {
            return this->state_.transferFunction;
        }

        /**
         * @brief Process audio through the waveshaper.
         *
         * Applies the transfer function to each sample in-place.
         */
        template <typename SampleType>
        constexpr void processImpl (BufferView<SampleType>& buffer, WaveshaperState<TransferFunction>& state, std::size_t sampleCount) noexcept
        {
            auto* audio = buffer.getWritePointer (0);

            for (std::size_t i = 0; i < sampleCount; ++i)
            {
                audio[i] = static_cast<SampleType> (state.transferFunction (static_cast<float> (audio[i])));
            }
        }
    };

} // namespace PlayfulTones::DspToolbox::Processors
