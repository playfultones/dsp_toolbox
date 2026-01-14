/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/simd/simd.hpp"

#include <array>
#include <concepts>
#include <cstddef>
#include <ranges>
#include <span>

namespace PlayfulTones::DspToolbox::simd
{

    /**
     * @brief Concept for block types compatible with VectorizedProcessor.
     *
     * A BlockLike type must provide:
     *
     * | Member | Type | Description |
     * |--------|------|-------------|
     * | `voices` | `static constexpr std::size_t` | Number of voices (M dimension) |
     * | `samples` | `static constexpr std::size_t` | Number of samples (N dimension) |
     * | `data` | `std::array<SampleType, M*N>` | Underlying storage |
     * | `voice(v)` | `std::span<SampleType, N>` | Row access (all samples for voice v) |
     * | `sample(s)` | `std::array<SampleType, M>` | Column access (all voices at sample s) |
     * | `setSample(s, arr)` | `void` | Set all voices at sample s |
     *
     * ## Example Custom Block
     * ```cpp
     * template<std::size_t M, std::size_t N>
     * struct MyBlock {
     *     static constexpr std::size_t voices = M;
     *     static constexpr std::size_t samples = N;
     *     std::array<float, M * N> data{};
     *
     *     std::span<float, N> voice(std::size_t v) { return {&data[v * N], N}; }
     *     std::array<float, M> sample(std::size_t s) const { ... }
     *     void setSample(std::size_t s, const std::array<float, M>& values) { ... }
     * };
     * static_assert(BlockLike<MyBlock<4, 8>>);
     * ```
     *
     * @tparam B Block type to check
     */
    template <typename B>
    concept BlockLike = requires (B b, const B cb, std::size_t idx) {
        { B::voices } -> std::convertible_to<std::size_t>;
        { B::samples } -> std::convertible_to<std::size_t>;

        { b.data } -> std::ranges::contiguous_range;
        typename decltype (b.data)::value_type;

        { b.voice (idx) };

        { cb.sample (idx) };

        { b.setSample (idx, cb.sample (idx)) } -> std::same_as<void>;
    };

    /**
     * @brief M×N block for vectorization along voice and time dimensions.
     *
     * Provides storage and access patterns for SIMD processing with
     * configurable parallelization strategies. Voice-major layout
     * (samples for each voice are contiguous) optimizes for time-parallel
     * processing, which is the most common case.
     *
     * @tparam Voices Number of voices (M dimension)
     * @tparam Samples Number of samples per voice (N dimension)
     * @tparam SampleType Floating-point sample type (float or double)
     *
     * ## Storage Layout (Voice-Major)
     *
     * ```
     * [Voice 0: s0, s1, s2, ..., sN-1]
     * [Voice 1: s0, s1, s2, ..., sN-1]
     * ...
     * [Voice M-1: s0, s1, s2, ..., sN-1]
     * ```
     *
     * ## Strategy Selection Guidelines
     *
     * | Operation Type          | Recommended Strategy | Reason                      |
     * |-------------------------|---------------------|------------------------------|
     * | Delay / Buffer playback | Time-parallel       | Sequential memory access     |
     * | IIR Filter (Biquad, SVF)| Voice-parallel      | State dependency in time     |
     * | Stateless (Gain, Clip)  | Either              | Both work well               |
     * | Oscillator phase        | Voice-parallel      | Phase accumulator is serial  |
     * | FFT                     | Time-parallel       | Algorithm requirement        |
     *
     * @see TimeParallel, VoiceParallel, BothParallel for convenience aliases
     */
    template <std::size_t Voices, std::size_t Samples, std::floating_point SampleType = float>
    struct Block
    {
        static_assert (Voices > 0, "Block must have at least one voice");
        static_assert (Samples > 0, "Block must have at least one sample");

        static constexpr std::size_t voices = Voices;
        static constexpr std::size_t samples = Samples;
        static constexpr std::size_t total = Voices * Samples;

        /// Aligned storage for SIMD access (64-byte alignment for AVX-512 compatibility)
        alignas (64) std::array<SampleType, total> data {};

        /**
         * @brief Access sample at specific voice and time index.
         *
         * @param voice Voice index [0, Voices)
         * @param sample Sample index [0, Samples)
         * @return Reference to the sample
         */
        [[nodiscard]] constexpr SampleType& at (std::size_t voice, std::size_t sample) noexcept
        {
            return data[voice * Samples + sample];
        }

        /**
         * @brief Access sample at specific voice and time index (const).
         *
         * @param voice Voice index [0, Voices)
         * @param sample Sample index [0, Samples)
         * @return Const reference to the sample
         */
        [[nodiscard]] constexpr const SampleType& at (std::size_t voice, std::size_t sample) const noexcept
        {
            return data[voice * Samples + sample];
        }

        /**
         * @brief Get all samples for a single voice.
         *
         * Provides contiguous access for time-parallel processing (SIMD along time).
         * Ideal for delays, buffer playback, and stateless operations.
         *
         * @param v Voice index [0, Voices)
         * @return Span of samples for the voice
         */
        [[nodiscard]] constexpr std::span<SampleType, Samples> voice (std::size_t v) noexcept
        {
            return std::span<SampleType, Samples> { &data[v * Samples], Samples };
        }

        /**
         * @brief Get all samples for a single voice (const).
         *
         * @param v Voice index [0, Voices)
         * @return Const span of samples for the voice
         */
        [[nodiscard]] constexpr std::span<const SampleType, Samples> voice (std::size_t v) const noexcept
        {
            return std::span<const SampleType, Samples> { &data[v * Samples], Samples };
        }

        /**
         * @brief Get pointer to voice data.
         *
         * @param v Voice index [0, Voices)
         * @return Pointer to first sample of the voice
         */
        [[nodiscard]] constexpr SampleType* voicePointer (std::size_t v) noexcept
        {
            return &data[v * Samples];
        }

        /**
         * @brief Get pointer to voice data (const).
         *
         * @param v Voice index [0, Voices)
         * @return Const pointer to first sample of the voice
         */
        [[nodiscard]] constexpr const SampleType* voicePointer (std::size_t v) const noexcept
        {
            return &data[v * Samples];
        }

        /**
         * @brief Get all voices for a single time sample.
         *
         * Returns a copy because voice-parallel data is not contiguous in
         * voice-major layout. Use for IIR filters, oscillators, and other
         * operations where state depends on previous samples.
         *
         * @param s Sample index [0, Samples)
         * @return Array of samples across all voices at time s
         */
        [[nodiscard]] constexpr std::array<SampleType, Voices> sample (std::size_t s) const noexcept
        {
            std::array<SampleType, Voices> col {};
            for (std::size_t v = 0; v < Voices; ++v)
            {
                col[v] = data[v * Samples + s];
            }
            return col;
        }

        /**
         * @brief Set all voices for a single time sample.
         *
         * @param s Sample index [0, Samples)
         * @param values Array of values to set for each voice at time s
         */
        constexpr void setSample (std::size_t s, const std::array<SampleType, Voices>& values) noexcept
        {
            for (std::size_t v = 0; v < Voices; ++v)
            {
                data[v * Samples + s] = values[v];
            }
        }

        /**
         * @brief Clear all samples to zero.
         *
         * Uses SIMD acceleration when available.
         */
        void clear() noexcept
        {
            simd::clear (data.data(), total);
        }

        /**
         * @brief Fill all samples with a value.
         *
         * @param value Value to fill with
         */
        constexpr void fill (SampleType value) noexcept
        {
            for (std::size_t i = 0; i < total; ++i)
            {
                data[i] = value;
            }
        }

        /**
         * @brief Multiply all samples by a factor.
         *
         * Uses SIMD acceleration when available.
         *
         * @param factor Multiplication factor
         */
        void multiply (SampleType factor) noexcept
        {
            simd::multiply (data.data(), total, factor);
        }

        /**
         * @brief Add samples from another block.
         *
         * Uses SIMD acceleration when available.
         *
         * @param other Block to add from
         */
        void addFrom (const Block& other) noexcept
        {
            simd::add (data.data(), other.data.data(), total);
        }

        /**
         * @brief Copy samples from another block.
         *
         * Uses SIMD acceleration when available.
         *
         * @param other Block to copy from
         */
        void copyFrom (const Block& other) noexcept
        {
            simd::copy (data.data(), other.data.data(), total);
        }

        [[nodiscard]] constexpr auto begin() noexcept { return data.begin(); }
        [[nodiscard]] constexpr auto end() noexcept { return data.end(); }
        [[nodiscard]] constexpr auto begin() const noexcept { return data.begin(); }
        [[nodiscard]] constexpr auto end() const noexcept { return data.end(); }
        [[nodiscard]] constexpr auto cbegin() const noexcept { return data.cbegin(); }
        [[nodiscard]] constexpr auto cend() const noexcept { return data.cend(); }
    };

    // Verify Block satisfies BlockLike (explicit interface documentation).
    // Dimensions are arbitrary - concept checks structural properties that hold for all M×N.
    static_assert (BlockLike<Block<4, 8, float>>, "Block must satisfy BlockLike concept");
    static_assert (BlockLike<Block<4, 8, double>>, "Block<double> must satisfy BlockLike concept");

    /**
     * @brief Time-parallel block: 1 voice × N samples.
     *
     * Use for operations that benefit from SIMD along the time axis:
     * - Delay lines (sequential memory reads)
     * - Buffer playback
     * - Stateless operations (gain, clip) on single voice
     *
     * @tparam N Number of samples
     * @tparam SampleType Sample type (float or double)
     */
    template <std::size_t N, std::floating_point SampleType = float>
    using TimeParallel = Block<1, N, SampleType>;
    static_assert (BlockLike<TimeParallel<16, float>>, "TimeParallel must satisfy BlockLike concept");

    /**
     * @brief Voice-parallel block: M voices × 1 sample.
     *
     * Use for operations that benefit from SIMD along the voice axis:
     * - IIR filters (Biquad, SVF) - state depends on previous samples
     * - Oscillators - phase accumulator is serial in time
     * - Any operation with time-dependent state
     *
     * @tparam M Number of voices
     * @tparam SampleType Sample type (float or double)
     */
    template <std::size_t M, std::floating_point SampleType = float>
    using VoiceParallel = Block<M, 1, SampleType>;
    static_assert (BlockLike<VoiceParallel<8, float>>, "VoiceParallel must satisfy BlockLike concept");

    /**
     * @brief Both-parallel block: M voices × N samples.
     *
     * Use for maximum throughput when both dimensions can be vectorized.
     * Typically combined with range hoisting to process SIMD-width chunks.
     *
     * @tparam M Number of voices
     * @tparam N Number of samples
     * @tparam SampleType Sample type (float or double)
     */
    template <std::size_t M, std::size_t N, std::floating_point SampleType = float>
    using BothParallel = Block<M, N, SampleType>;
    static_assert (BlockLike<BothParallel<4, 8, float>>, "BothParallel must satisfy BlockLike concept");

    /**
     * @brief Vectorization strategy selector.
     *
     * Determines how SIMD operations are applied to a block:
     *
     * - **TimeParallel**: SIMD along time axis. Process each voice with
     *   time-vectorized operations. Best for delays, buffers, stateless ops.
     *
     * - **VoiceParallel**: SIMD along voice axis. Process each sample across
     *   all voices. Best for IIR filters, oscillators, state-dependent ops.
     *
     * - **Hybrid**: Adaptive strategy based on operation type and block size.
     *   May use different strategies for different parts of processing.
     */
    enum class VectorizationStrategy {
        TimeParallel, ///< SIMD along time axis (process voices sequentially)
        VoiceParallel, ///< SIMD along voice axis (process samples sequentially)
        Hybrid ///< Adaptive based on operation and block size
    };

    /**
     * @brief Concept for processors compatible with TimeParallel strategy.
     *
     * A TimeParallelProcessor must implement:
     * ```cpp
     * template<std::size_t N>
     * void process(std::span<SampleType, N> voiceData);
     * ```
     *
     * This processes all N samples for a single voice. SIMD operations
     * should vectorize along the time (sample) axis.
     *
     * ## Example Implementation
     * ```cpp
     * struct GainProcessor {
     *     float gain = 1.0f;
     *
     *     template<std::size_t N>
     *     void process(std::span<float, N> voiceData) {
     *         for (auto& sample : voiceData) {
     *             sample *= gain;
     *         }
     *     }
     * };
     * static_assert(TimeParallelProcessor<GainProcessor, float>);
     * ```
     *
     * @tparam P Processor type to check
     * @tparam SampleType Sample type (float or double)
     */
    template <typename P, typename SampleType>
    concept TimeParallelProcessor = requires (P p, std::span<SampleType, 8> voiceData) {
        { p.process (voiceData) } -> std::same_as<void>;
    };

    /**
     * @brief Concept for processors compatible with VoiceParallel strategy.
     *
     * A VoiceParallelProcessor must implement:
     * ```cpp
     * template<std::size_t M>
     * void processVoices(std::array<SampleType, M>& samples);
     * ```
     *
     * This processes one sample across all M voices simultaneously.
     * SIMD operations should vectorize along the voice axis.
     *
     * ## Example Implementation
     * ```cpp
     * struct BiquadFilter {
     *     std::array<float, 8> z1{}, z2{};  // Per-voice state
     *
     *     template<std::size_t M>
     *     void processVoices(std::array<float, M>& samples) {
     *         // Process all voices for current sample
     *         for (std::size_t v = 0; v < M; ++v) {
     *             float input = samples[v];
     *             float output = b0*input + z1[v];
     *             z1[v] = b1*input - a1*output + z2[v];
     *             z2[v] = b2*input - a2*output;
     *             samples[v] = output;
     *         }
     *     }
     * };
     * static_assert(VoiceParallelProcessor<BiquadFilter, float>);
     * ```
     *
     * @tparam P Processor type to check
     * @tparam SampleType Sample type (float or double)
     */
    template <typename P, typename SampleType>
    concept VoiceParallelProcessor = requires (P p, std::array<SampleType, 8>& samples) {
        { p.processVoices (samples) } -> std::same_as<void>;
    };

    /**
     * @brief Concept for processors compatible with Hybrid strategy.
     *
     * A HybridProcessor must implement both interfaces:
     * - `process(std::span<SampleType, N>)` for time-parallel paths
     * - `processVoices(std::array<SampleType, M>&)` for voice-parallel paths
     *
     * This allows the VectorizedProcessor to choose the best strategy
     * at runtime or compile-time based on the operation characteristics.
     *
     * @tparam P Processor type to check
     * @tparam SampleType Sample type (float or double)
     */
    template <typename P, typename SampleType>
    concept HybridProcessor = TimeParallelProcessor<P, SampleType> && VoiceParallelProcessor<P, SampleType>;

    /**
     * @brief Processor wrapper that applies a vectorization strategy.
     *
     * Wraps a processor and routes block processing through the selected
     * vectorization strategy. The processor must satisfy the corresponding
     * concept for the chosen strategy:
     *
     * | Strategy      | Required Concept         | Required Method                    |
     * |---------------|--------------------------|-------------------------------------|
     * | TimeParallel  | TimeParallelProcessor    | `process(std::span<T, N>)`         |
     * | VoiceParallel | VoiceParallelProcessor   | `processVoices(std::array<T, M>&)` |
     * | Hybrid        | HybridProcessor          | Both methods                        |
     *
     * @tparam Strategy Vectorization strategy to apply
     * @tparam Processor The wrapped processor type (must satisfy strategy concept)
     *
     * @code
     * // Time-parallel delay processor
     * struct DelayProcessor {
     *     template<std::size_t N>
     *     void process(std::span<float, N> voiceData) { ... }
     * };
     * VectorizedProcessor<VectorizationStrategy::TimeParallel, DelayProcessor> delay;
     * delay.process(block, numVoices);
     *
     * // Voice-parallel filter processor
     * struct FilterProcessor {
     *     template<std::size_t M>
     *     void processVoices(std::array<float, M>& samples) { ... }
     * };
     * VectorizedProcessor<VectorizationStrategy::VoiceParallel, FilterProcessor> filter;
     * filter.process(block, numVoices);
     * @endcode
     */
    template <VectorizationStrategy Strategy, typename Processor>
    struct VectorizedProcessor
    {
        Processor processor;

        /**
         * @brief Process a block using the selected vectorization strategy.
         *
         * @tparam BlockType Block type (must satisfy BlockLike concept)
         * @param block Block to process in-place
         * @param numVoices Number of active voices to process
         */
        template <BlockLike BlockType>
        void process (BlockType& block, [[maybe_unused]] std::size_t numVoices)
        {
            using SampleType = typename decltype (block.data)::value_type;

            if constexpr (Strategy == VectorizationStrategy::TimeParallel)
            {
                static_assert (TimeParallelProcessor<Processor, SampleType>,
                    "Processor must implement: void process(std::span<SampleType, N> voiceData)");

                for (std::size_t v = 0; v < numVoices; ++v)
                {
                    processor.process (block.voice (v));
                }
            }
            else if constexpr (Strategy == VectorizationStrategy::VoiceParallel)
            {
                static_assert (VoiceParallelProcessor<Processor, SampleType>,
                    "Processor must implement: void processVoices(std::array<SampleType, M>& samples)");

                for (std::size_t s = 0; s < block.samples; ++s)
                {
                    auto voiceData = block.sample (s);
                    processor.processVoices (voiceData);
                    block.setSample (s, voiceData);
                }
            }
            else // Hybrid
            {
                static_assert (TimeParallelProcessor<Processor, SampleType>,
                    "Hybrid strategy requires: void process(std::span<SampleType, N> voiceData)");

                for (std::size_t v = 0; v < numVoices; ++v)
                {
                    processor.process (block.voice (v));
                }
            }
        }
    };

} // namespace PlayfulTones::DspToolbox::simd
