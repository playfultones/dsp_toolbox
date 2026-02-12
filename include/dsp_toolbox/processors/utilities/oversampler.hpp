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
         * 23-tap symmetric half-band FIR designed via Parks-McClellan
         * (equiripple) method, normalized to unity DC gain.
         * Provides ~45 dB stopband rejection.
         *
         * Half-band property: every other coefficient (except center) is zero.
         * This allows efficient polyphase implementation.
         *
         * Filter specs:
         * - Order: 22 (23 taps)
         * - Passband: 0 to 0.18 * fs_up (flat to < 0.1 dB)
         * - Transition: 0.18 to 0.32 * fs_up
         * - Stopband: 0.32 * fs_up to 0.5 * fs_up
         * - Rejection: ~45 dB
         * - DC gain: 1.0 (normalized)
         *
         * At 48 kHz source (96 kHz upsampled): flat to 18 kHz, -0.7 dB at 20 kHz
         * At 44.1 kHz source (88.2 kHz upsampled): flat to 16 kHz, -2.2 dB at 20 kHz
         *
         * Non-zero coefficients (symmetric, only unique half listed):
         * h[11] = center (single center tap)
         * h[10] = h[12] = wing[0], h[8] = h[14] = wing[1], etc.
         * All even-distance-from-center coefficients are zero (half-band zeros).
         */
        struct HalfBandCoeffs
        {
            /// Number of non-zero wing coefficient pairs (K = (N-3)/4 = 5)
            static constexpr std::size_t numWings = 5;

            /// Non-zero wing coefficients, innermost to outermost:
            /// wing[k] corresponds to h[center-(2k+1)] = h[center+(2k+1)]
            static constexpr std::array<float, numWings> wing = {
                0.3120253f, // h[10] = h[12]  (offset 1 from center)
                -0.0916125f, // h[8]  = h[14]  (offset 3)
                0.0422511f, // h[6]  = h[16]  (offset 5)
                -0.0197843f, // h[4]  = h[18]  (offset 7)
                0.0081895f // h[2]  = h[20]  (offset 9)
            };

            /// Center tap value (normalized, slightly below 0.5 for unity DC)
            static constexpr float center = 0.4978618f;

            /// Total filter length (taps)
            static constexpr std::size_t length = 23;

            /// Group delay in samples at the upsampled rate
            static constexpr std::size_t groupDelay = (length - 1) / 2; // 11 at 2x rate

            /// Group delay as seen at the original sample rate
            /// (ceil division since we decimate by 2)
            static constexpr std::size_t groupDelayAtOriginalRate = (groupDelay + 1) / 2; // 6
        };

        /**
         * @brief Half-band FIR filter state for polyphase implementation.
         *
         * Stores the delay line for the FIR filter. Uses polyphase
         * decomposition: only non-zero coefficients are computed.
         */
        struct HalfBandState
        {
            /// Delay line for the FIR filter
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
             * For N=23: center tap at h[11], wing pairs at odd offsets from center.
             * Even-offset taps are zero (half-band property).
             *
             * read() offsets map h[i] to read(N-1-i):
             *   center h[11] -> read(11)
             *   wing[0] h[10],h[12] -> read(12),read(10)
             *   wing[1] h[8],h[14]  -> read(14),read(8)
             *   wing[2] h[6],h[16]  -> read(16),read(6)
             *   wing[3] h[4],h[18]  -> read(18),read(4)
             *   wing[4] h[2],h[20]  -> read(20),read(2)
             */
            [[nodiscard]] constexpr float computeOutput() const noexcept
            {
                float output = HalfBandCoeffs::center * read (11);

                output += HalfBandCoeffs::wing[0] * (read (12) + read (10));
                output += HalfBandCoeffs::wing[1] * (read (14) + read (8));
                output += HalfBandCoeffs::wing[2] * (read (16) + read (6));
                output += HalfBandCoeffs::wing[3] * (read (18) + read (4));
                output += HalfBandCoeffs::wing[4] * (read (20) + read (2));

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
     * Uses a 23-tap symmetric Parks-McClellan half-band filter with ~45 dB
     * stopband rejection. The half-band structure means every other coefficient
     * is zero, reducing computation by ~50%.
     *
     * ## Latency
     * The oversampler introduces latency equal to the half-band FIR group delay
     * (at the original sample rate): 6 samples. This is reported via
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
