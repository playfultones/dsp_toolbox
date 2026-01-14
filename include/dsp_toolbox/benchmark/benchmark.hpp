/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/benchmark/clock.hpp"
#include "dsp_toolbox/core/buffer_view.hpp"

#include <concepts>
#include <cstddef>
#include <string>

namespace PlayfulTones::DspToolbox::Benchmark
{

    /**
     * @brief Result of a benchmark run.
     *
     * Contains timing metrics from benchmarking a processor or algorithm.
     */
    struct BenchmarkResult
    {
        double cyclesPerSample {}; ///< Primary metric: CPU cycles per sample
        double secondsPerSample {}; ///< Wall-clock time per sample
        double cpuPercent {}; ///< Estimated CPU usage at configured sample rate
        std::size_t iterations {}; ///< Number of iterations run
        std::size_t samplesProcessed {}; ///< Total samples processed
    };

    /**
     * @brief Configuration for benchmark execution.
     */
    struct BenchmarkConfig
    {
        std::size_t warmupIterations { 100 }; ///< Iterations to discard (cache warmup)
        std::size_t measureIterations { 1000 }; ///< Iterations to measure
        std::size_t blockSize { 512 }; ///< Samples per iteration (for reference)
        double sampleRate { 48000.0 }; ///< Sample rate for CPU% calculation
    };

    /**
     * @brief Concept for types that can be benchmarked.
     *
     * A Benchmarkable type must provide:
     * - `process(BufferView<float>&)` or `process(BufferView<float>&, std::size_t)`
     *
     * Most processors in dsp_toolbox satisfy this.
     */
    template <typename P>
    concept Benchmarkable = requires (P p, BufferView<float>& buffer) {
        { p.process (buffer) } -> std::same_as<void>;
    } || requires (P p, BufferView<float>& buffer, std::size_t count) {
        { p.process (buffer, count) } -> std::same_as<void>;
    };

    namespace detail
    {
        template <typename Processor>
        void invokeProcess (Processor& proc, BufferView<float>& buffer)
        {
            if constexpr (requires { proc.process (buffer); })
            {
                proc.process (buffer);
            }
            else
            {
                proc.process (buffer, buffer.getNumSamples());
            }
        }
    } // namespace detail

    /**
     * @brief Run a benchmark on a processor.
     *
     * Performs warmup iterations followed by timed measurement iterations.
     * Uses the specified clock for timing (CycleClock for cycles, ChronoClock for time).
     *
     * @tparam Clock Timing clock type (default: CycleClock)
     * @tparam Processor Processor type to benchmark (must satisfy Benchmarkable)
     * @param processor Prepared processor instance
     * @param buffer Buffer to process (modified in place each iteration)
     * @param config Benchmark configuration
     * @return BenchmarkResult with timing metrics
     *
     * @code
     * Gain<> gain;
     * gain.setGainDb(-6.0f);
     *
     * HeapAudioBuffer<float> buffer(1, 512);
     * auto view = buffer.getView();
     *
     * auto result = runBenchmark(gain, view);
     * std::cout << "Cycles/sample: " << result.cyclesPerSample << "\n";
     * @endcode
     */
    template <BenchmarkClock Clock = CycleClock, Benchmarkable Processor>
    [[nodiscard]] BenchmarkResult runBenchmark (
        Processor& processor,
        BufferView<float>& buffer,
        const BenchmarkConfig& config = {}) noexcept
    {
        std::size_t const numSamples = buffer.getNumSamples();

        // Warmup: fill caches, stabilize branch predictors
        for (std::size_t i = 0; i < config.warmupIterations; ++i)
        {
            benchmarkFence();
            detail::invokeProcess (processor, buffer);
            benchmarkFence();
        }

        // Measurement
        benchmarkFence();
        auto const start = Clock::now();

        for (std::size_t i = 0; i < config.measureIterations; ++i)
        {
            detail::invokeProcess (processor, buffer);
        }

        benchmarkFence();
        auto const end = Clock::now();

        double const elapsed = Clock::elapsed (start, end);
        std::size_t const totalSamples = config.measureIterations * numSamples;

        BenchmarkResult result;
        result.iterations = config.measureIterations;
        result.samplesProcessed = totalSamples;

        if constexpr (std::same_as<Clock, CycleClock>)
        {
            result.cyclesPerSample = elapsed / static_cast<double> (totalSamples);
            double const freq = CycleClock::estimatedFrequencyHz();
            result.secondsPerSample = result.cyclesPerSample / freq;
        }
        else
        {
            // ChronoClock returns seconds directly
            result.secondsPerSample = elapsed / static_cast<double> (totalSamples);
            double const freq = CycleClock::estimatedFrequencyHz();
            result.cyclesPerSample = result.secondsPerSample * freq;
        }

        // Calculate CPU% at given sample rate
        // CPU% = (time per sample) / (available time per sample) * 100
        double const availableTime = 1.0 / config.sampleRate;
        result.cpuPercent = (result.secondsPerSample / availableTime) * 100.0;

        return result;
    }

    /**
     * @brief Format a benchmark result as a string.
     *
     * @param result Benchmark result to format
     * @param name Optional name for the benchmark
     * @return Formatted string with metrics
     */
    [[nodiscard]] inline std::string formatResult (
        const BenchmarkResult& result,
        std::string_view name = "Benchmark")
    {
        std::string out;
        out.reserve (256);

        out += name;
        out += ":\n";
        out += "  Cycles/sample:  " + std::to_string (result.cyclesPerSample) + "\n";
        out += "  Seconds/sample: " + std::to_string (result.secondsPerSample) + "\n";
        out += "  CPU%:           " + std::to_string (result.cpuPercent) + "%\n";
        out += "  Iterations:     " + std::to_string (result.iterations) + "\n";
        out += "  Total samples:  " + std::to_string (result.samplesProcessed) + "\n";

        return out;
    }

    /**
     * @brief Compare two benchmark results.
     *
     * @param baseline Baseline result
     * @param optimized Optimized result
     * @return Improvement percentage (positive = faster)
     */
    [[nodiscard]] inline double compareResults (
        const BenchmarkResult& baseline,
        const BenchmarkResult& optimized) noexcept
    {
        if (baseline.cyclesPerSample <= 0.0)
            return 0.0;

        double const improvement = baseline.cyclesPerSample - optimized.cyclesPerSample;
        return (improvement / baseline.cyclesPerSample) * 100.0;
    }

} // namespace PlayfulTones::DspToolbox::Benchmark
