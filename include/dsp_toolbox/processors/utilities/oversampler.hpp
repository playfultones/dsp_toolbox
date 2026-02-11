/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/core/buffer_view.hpp"
#include "dsp_toolbox/core/constexpr_spec.hpp"
#include "dsp_toolbox/core/io_config.hpp"
#include "dsp_toolbox/core/static_audio_buffer.hpp"
#include "dsp_toolbox/processors/core/processor_base.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace PlayfulTones::DspToolbox::Processors
{

    namespace detail
    {
        /**
         * @brief Half-band FIR filter coefficients for 2x oversampling.
         *
         * 12-tap symmetric half-band FIR designed via Parks-McClellan
         * (equiripple) method. Provides ~70 dB stopband rejection.
         *
         * Half-band property: every other coefficient (except center) is zero.
         * This allows efficient polyphase implementation.
         *
         * Filter specs:
         * - Order: 11 (12 taps)
         * - Passband: 0 to 0.25 * fs (Nyquist of original rate)
         * - Stopband: 0.25 * fs to 0.5 * fs
         * - Rejection: ~70 dB
         *
         * Non-zero coefficients (symmetric, only unique half listed):
         * h[0] = h[11], h[2] = h[9], h[4] = h[7], h[5] = h[6] = 0.5 (center)
         * h[1] = h[3] = h[8] = h[10] = 0 (half-band zeros)
         */
        struct HalfBandCoeffs
        {
            /// Non-zero wing coefficients: h[0], h[2], h[4]
            /// (h[5]=h[6]=0.5 is the center tap pair, applied separately)
            static constexpr std::array<float, 3> wing = {
                -0.0244032f, // h[0] = h[11]
                0.1384614f, // h[2] = h[9]
                -0.6139127f // h[4] = h[7]  (large due to proximity to center)
            };

            /// Center tap value (half-band: center = 0.5 for each polyphase arm)
            static constexpr float center = 0.5f;

            /// Total filter length (taps)
            static constexpr std::size_t length = 12;

            /// Group delay in samples at the upsampled rate
            static constexpr std::size_t groupDelay = (length - 1) / 2; // 5 at 2x rate

            /// Group delay as seen at the original sample rate
            /// (ceil division since we decimate by 2)
            static constexpr std::size_t groupDelayAtOriginalRate = (groupDelay + 1) / 2; // 3
        };

        /**
         * @brief Half-band FIR filter state for polyphase implementation.
         *
         * Stores the delay line for the FIR filter. Uses polyphase
         * decomposition: only non-zero coefficients are computed.
         */
        struct HalfBandState
        {
            /// Delay line for polyphase filtering
            /// We need (length - 1) delay elements
            std::array<float, HalfBandCoeffs::length> delayLine {};
            std::size_t writeIndex { 0 };

            constexpr void reset() noexcept
            {
                delayLine = {};
                writeIndex = 0;
            }

            /**
             * @brief Push a sample into the delay line.
             */
            constexpr void push (float sample) noexcept
            {
                delayLine[writeIndex] = sample;
                writeIndex = (writeIndex + 1) % HalfBandCoeffs::length;
            }

            /**
             * @brief Read from delay line at given offset from write position.
             *
             * offset=0 returns the most recently pushed sample.
             */
            [[nodiscard]] constexpr float read (std::size_t offset) const noexcept
            {
                // writeIndex points to next write position,
                // so most recent sample is at writeIndex - 1
                std::size_t const idx =
                    (writeIndex + HalfBandCoeffs::length - 1 - offset) % HalfBandCoeffs::length;
                return delayLine[idx];
            }

            /**
             * @brief Compute one output sample of the half-band filter.
             *
             * Exploits half-band symmetry: only non-zero taps are computed.
             * h[1], h[3], h[8], h[10] = 0 (half-band zeros)
             * h[5] = h[6] = 0.5 (center)
             * h[0] = h[11], h[2] = h[9], h[4] = h[7] (wing symmetry)
             */
            [[nodiscard]] constexpr float computeOutput() const noexcept
            {
                float output = 0.0f;

                output += HalfBandCoeffs::center * (read (5) + read (6));

                output += HalfBandCoeffs::wing[0] * (read (0) + read (11));
                output += HalfBandCoeffs::wing[1] * (read (2) + read (9));
                output += HalfBandCoeffs::wing[2] * (read (4) + read (7));

                return output;
            }
        };
    } // namespace detail

    /**
     * @brief Compute the upsampled ConstexprSpec (2x sample rate, 2x block size).
     *
     * @tparam Spec The original ConstexprSpec
     */
    template <ConstexprSpec Spec>
    inline constexpr ConstexprSpec UpsampledSpec2x {
        .sampleRate = SampleRate { Spec.sampleRate.value * 2.0 },
        .blockSize = Samples<std::uint32_t> { Spec.blockSize.value * 2 },
        .numChannels = Spec.numChannels
    };

    /**
     * @brief State for Oversampler2x.
     *
     * Contains the inner processor (instantiated at 2x spec), the
     * upsampled audio buffer, and the anti-aliasing FIR filter states.
     *
     * @tparam InnerProcessor Template that takes ConstexprSpec and produces a processor
     * @tparam Spec Original (non-upsampled) ConstexprSpec
     */
    template <template <ConstexprSpec> typename InnerProcessor, ConstexprSpec Spec>
    struct Oversampler2xState
    {
        /// The inner processor, instantiated at 2x sample rate/block size
        InnerProcessor<UpsampledSpec2x<Spec>> innerProcessor {};

        /// Upsampled audio buffer: 1 channel, 2x block size
        /// Uses std::max to ensure at least 1 sample for RuntimeSpec (blockSize == 0).
        static constexpr std::size_t kUpsampledBlockSize =
            Spec.blockSize.value > 0 ? Spec.blockSize.value * 2 : 1;
        StaticAudioBuffer<1, kUpsampledBlockSize> upsampledBuffer {};

        /// Anti-aliasing FIR state for upsampling
        detail::HalfBandState upsampleFilter {};

        /// Anti-aliasing FIR state for downsampling
        detail::HalfBandState downsampleFilter {};

        constexpr void prepare (double /*sampleRate*/, std::size_t /*blockSize*/) noexcept
        {
            upsampledBuffer.clear();
            upsampleFilter.reset();
            downsampleFilter.reset();
        }

        constexpr void resetTransient() noexcept
        {
            upsampledBuffer.clear();
            upsampleFilter.reset();
            downsampleFilter.reset();
            if constexpr (requires { innerProcessor.reset(); })
            {
                innerProcessor.reset();
            }
        }
    };

    /**
     * @brief 2x oversampling wrapper for mono processors.
     *
     * Wraps an inner processor with 2x oversampling:
     * 1. Upsample input by 2 (zero-stuff + half-band FIR anti-image filter)
     * 2. Process at 2x sample rate through the inner processor
     * 3. Downsample by 2 (half-band FIR anti-alias filter + decimate)
     *
     * The inner processor is automatically instantiated with a ConstexprSpec
     * that has 2x the sample rate and 2x the block size of the outer spec.
     *
     * ## Half-band FIR
     * Uses a 12-tap symmetric Parks-McClellan half-band filter with ~70 dB
     * stopband rejection. The half-band structure means every other coefficient
     * is zero, reducing computation by ~50%.
     *
     * ## Latency
     * The oversampler introduces latency equal to the half-band FIR group delay
     * (at the original sample rate): 3 samples. This is reported via
     * getLatencySamples().
     *
     * ## Usage
     * @code
     * // Create a 2x oversampled waveshaper
     * Oversampler2x<Waveshaper<TanhShaper>::Processor, Spec48000_512> os;
     * os.process(buffer, numSamples);
     *
     * // Access inner processor for parameter changes
     * os.getInnerProcessor().getTransferFunction().k = 50.0f;
     * @endcode
     *
     * @tparam InnerProcessor Template taking ConstexprSpec, producing a processor
     * @tparam Spec Compile-time configuration (original sample rate)
     */
    template <template <ConstexprSpec> typename InnerProcessor, ConstexprSpec Spec = DefaultSpec>
    class Oversampler2x : public ProcessorBase<
                              Oversampler2x<InnerProcessor, Spec>,
                              IOConfig<1, 1, 0, 0>,
                              Oversampler2xState<InnerProcessor, Spec>,
                              Spec>
    {
        using Base = ProcessorBase<
            Oversampler2x<InnerProcessor, Spec>,
            IOConfig<1, 1, 0, 0>,
            Oversampler2xState<InnerProcessor, Spec>,
            Spec>;

    public:
        using State = Oversampler2xState<InnerProcessor, Spec>;

        /// Processor template alias for StereoExpander and ProcessorWrapper
        template <ConstexprSpec S>
        using Processor = Oversampler2x<InnerProcessor, S>;

        /**
         * @brief Default constructor — reports latency.
         */
        constexpr Oversampler2x() noexcept
        {
            this->setLatency (detail::HalfBandCoeffs::groupDelayAtOriginalRate);
        }

        /**
         * @brief Get mutable reference to the inner processor.
         *
         * Use this to configure the inner processor's parameters.
         */
        [[nodiscard]] constexpr auto& getInnerProcessor() noexcept
        {
            return this->state_.innerProcessor;
        }

        /**
         * @brief Get const reference to the inner processor.
         */
        [[nodiscard]] constexpr auto const& getInnerProcessor() const noexcept
        {
            return this->state_.innerProcessor;
        }

        /**
         * @brief Process audio with 2x oversampling.
         *
         * Steps:
         * 1. Upsample: zero-stuff input and apply anti-image FIR
         * 2. Process upsampled buffer through inner processor
         * 3. Downsample: apply anti-alias FIR and decimate
         */
        template <typename SampleType>
        constexpr void processImpl (BufferView<SampleType>& buffer, State& state, std::size_t sampleCount) noexcept
        {
            auto const* input = buffer.getReadPointer (0);
            auto* output = buffer.getWritePointer (0);

            std::size_t const upsampledCount = sampleCount * 2;

            auto* upsampledData = state.upsampledBuffer.getWritePointer (0);

            for (std::size_t i = 0; i < sampleCount; ++i)
            {
                state.upsampleFilter.push (static_cast<float> (input[i]) * 2.0f);
                upsampledData[i * 2] = state.upsampleFilter.computeOutput();

                state.upsampleFilter.push (0.0f);
                upsampledData[i * 2 + 1] = state.upsampleFilter.computeOutput();
            }

            float* upsampledPtr = upsampledData;
            BufferView<float> upsampledView (&upsampledPtr, 1, upsampledCount);
            state.innerProcessor.process (upsampledView, upsampledCount);

            for (std::size_t i = 0; i < sampleCount; ++i)
            {
                state.downsampleFilter.push (upsampledData[i * 2]);
                state.downsampleFilter.push (upsampledData[i * 2 + 1]);

                output[i] = static_cast<SampleType> (state.downsampleFilter.computeOutput());
            }
        }
    };

} // namespace PlayfulTones::DspToolbox::Processors
