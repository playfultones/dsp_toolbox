/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/benchmark/benchmark.hpp"
#include "dsp_toolbox/benchmark/execution_plan.hpp"
#include "dsp_toolbox/core/buffer_view.hpp"
#include "dsp_toolbox/core/process_spec.hpp"

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <span>
#include <vector>

namespace PlayfulTones::DspToolbox::Benchmark
{

    /**
     * @brief Concept for types that can be executed with an ExecutionPlan.
     *
     * The Executable type must provide:
     * - `process(BufferView<float>&)` for basic processing
     * - `prepare(ProcessSpec)` for initialization (optional but recommended)
     *
     * Most processors in dsp_toolbox satisfy this concept.
     */
    template <typename G>
    concept Executable = Benchmarkable<G>;

    /**
     * @brief Executor that applies execution plans to a processor.
     *
     * Wraps a processor and provides benchmarking capabilities with
     * different execution strategies. Same algorithm, different execution -
     * enables empirical testing of which approach works best.
     *
     * @tparam Processor Processor type (must satisfy Executable)
     *
     * @code
     * Gain<> gain;
     * ParametricExecutor executor(gain);
     *
     * ProcessSpec spec{SampleRate{48000.0}, Samples{512u}, 1};
     * executor.prepare(spec);
     *
     * auto plans = generateCandidatePlans(spec);
     * auto results = executor.benchmarkPlans(plans);
     *
     * // Results sorted by speed
     * for (const auto& r : results) {
     *     std::cout << r.plan.describe() << ": "
     *               << r.timing.cyclesPerSample << " cycles\n";
     * }
     * @endcode
     */
    template <Executable Processor>
    class ParametricExecutor
    {
    public:
        /**
         * @brief Construct executor with a processor.
         *
         * @param processor Processor instance to wrap
         */
        explicit ParametricExecutor (Processor processor)
            : processor_ (std::move (processor))
        {
        }

        /**
         * @brief Prepare the processor for a given specification.
         *
         * @param spec Processing specification
         */
        void prepare (const ProcessSpec& spec)
        {
            spec_ = spec;
            if constexpr (requires { processor_.prepare (spec); })
            {
                processor_.prepare (spec);
            }
        }

        /**
         * @brief Set the current execution plan.
         *
         * @param plan Execution plan to use for subsequent processing
         */
        void setExecutionPlan (const ExecutionPlan& plan) noexcept
        {
            plan_ = plan;
        }

        /**
         * @brief Get the current execution plan.
         *
         * @return Reference to current plan
         */
        [[nodiscard]] const ExecutionPlan& getExecutionPlan() const noexcept
        {
            return plan_;
        }

        /**
         * @brief Process a buffer using the current execution plan.
         *
         * Currently delegates directly to the processor. Future versions
         * may apply plan-specific optimizations (chunking, SIMD strategy).
         *
         * @param buffer Buffer to process in-place
         */
        void process (BufferView<float>& buffer)
        {
            // For now, delegate directly to processor
            // Plan-specific execution can be added here
            if constexpr (requires { processor_.process (buffer); })
            {
                processor_.process (buffer);
            }
            else
            {
                processor_.process (buffer, buffer.getNumSamples());
            }
        }

        /**
         * @brief Benchmark multiple execution plans.
         *
         * Runs benchmarks for each plan and returns results sorted
         * by performance (fastest first).
         *
         * @param plans Plans to benchmark
         * @param config Benchmark configuration
         * @return Results sorted by cycles-per-sample (fastest first)
         */
        [[nodiscard]] std::vector<PlanBenchmarkResult> benchmarkPlans (
            std::span<const ExecutionPlan> plans,
            const BenchmarkConfig& config = {})
        {
            std::vector<PlanBenchmarkResult> results;
            results.reserve (plans.size());

            // Allocate buffer for benchmarking
            std::vector<float> audioData (spec_.maxBlockSize.value, 0.5f);
            std::array<float*, 1> channels { audioData.data() };
            BufferView<float> buffer { channels.data(), 1, spec_.maxBlockSize.value };

            for (const auto& plan : plans)
            {
                setExecutionPlan (plan);

                if constexpr (requires { processor_.reset(); })
                {
                    processor_.reset();
                }

                // Run benchmark
                auto timing = runBenchmark (*this, buffer, config);

                results.push_back ({ .plan = plan, .timing = timing });
            }

            // Sort by cycles per sample (fastest first)
            std::ranges::sort (results, [] (const auto& a, const auto& b) {
                return a.cyclesPerSample() < b.cyclesPerSample();
            });

            return results;
        }

        /**
         * @brief Access the underlying processor.
         *
         * @return Reference to the wrapped processor
         */
        [[nodiscard]] Processor& processor() noexcept { return processor_; }

        /**
         * @brief Access the underlying processor (const).
         *
         * @return Const reference to the wrapped processor
         */
        [[nodiscard]] const Processor& processor() const noexcept { return processor_; }

    private:
        Processor processor_;
        ExecutionPlan plan_ {};
        ProcessSpec spec_ {};
    };

    // Deduction guide
    template <typename P>
    ParametricExecutor (P) -> ParametricExecutor<P>;

} // namespace PlayfulTones::DspToolbox::Benchmark
