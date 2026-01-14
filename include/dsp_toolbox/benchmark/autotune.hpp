/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/benchmark/executor.hpp"
#include "dsp_toolbox/core/heap_audio_buffer.hpp"

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

namespace PlayfulTones::DspToolbox::Benchmark
{

    /**
     * @brief Configuration for auto-tuning.
     */
    struct AutoTuneConfig
    {
        BenchmarkConfig benchmark {}; ///< Benchmark configuration
        std::size_t maxPlansToTest { 10 }; ///< Maximum plans to benchmark
        double confidenceThreshold { 0.05 }; ///< Margin for "same" performance (5%)
    };

    /**
     * @brief Result of auto-tuning.
     */
    struct AutoTuneResult
    {
        ExecutionPlan bestPlan; ///< Best performing plan
        std::vector<PlanBenchmarkResult> allResults; ///< All benchmark results
        double improvementPercent {}; ///< Improvement vs. first candidate (baseline)
    };

    /**
     * @brief Find the optimal execution plan for a processor on current hardware.
     *
     * Generates candidate plans, benchmarks each, and returns the fastest.
     * The processor is prepared with the given spec before benchmarking.
     *
     * @tparam Processor Processor type (must satisfy Executable)
     * @param processor Processor instance (will be modified during benchmarking)
     * @param spec Processing specification
     * @param config Auto-tuning configuration
     * @return Best plan and all benchmark results
     *
     * @code
     * Gain<> gain;
     * ProcessSpec spec{SampleRate{48000.0}, Samples{512u}, 1};
     *
     * auto result = autoTune(gain, spec);
     *
     * std::cout << "Best plan: " << result.bestPlan.describe() << "\n";
     * std::cout << "Improvement: " << result.improvementPercent << "%\n";
     * @endcode
     */
    template <Executable Processor>
    [[nodiscard]] AutoTuneResult autoTune (
        Processor& processor,
        const ProcessSpec& spec,
        const AutoTuneConfig& config = {})
    {
        ParametricExecutor executor (processor);
        executor.prepare (spec);

        // Generate candidate plans
        auto candidates = generateCandidatePlans (spec);

        // Limit number of plans if requested
        if (candidates.size() > config.maxPlansToTest)
        {
            candidates.resize (config.maxPlansToTest);
        }

        // Benchmark all plans
        auto results = executor.benchmarkPlans (candidates, config.benchmark);

        AutoTuneResult result;
        result.allResults = std::move (results);

        if (result.allResults.empty())
        {
            return result;
        }

        // Best plan is first (already sorted by benchmarkPlans)
        result.bestPlan = result.allResults.front().plan;

        // Calculate improvement vs. baseline (last = slowest after sort)
        if (result.allResults.size() > 1)
        {
            double const baselineCycles = result.allResults.back().cyclesPerSample();
            double const bestCycles = result.allResults.front().cyclesPerSample();

            if (baselineCycles > 0.0)
            {
                result.improvementPercent =
                    ((baselineCycles - bestCycles) / baselineCycles) * 100.0;
            }
        }

        return result;
    }

    /**
     * @brief Format auto-tune results for display.
     *
     * @param result Auto-tune result to format
     * @return Formatted string with all results
     */
    [[nodiscard]] inline std::string formatAutoTuneResult (const AutoTuneResult& result)
    {
        std::string output;
        output.reserve (1024);

        output += "=== Auto-Tune Results ===\n\n";

        output += "Best Plan: " + result.bestPlan.describe() + "\n";
        output += "Improvement vs baseline: " + std::to_string (result.improvementPercent) + "%\n\n";

        output += "All Results (sorted by performance):\n";
        output += "------------------------------------\n";

        std::size_t rank = 1;
        for (const auto& r : result.allResults)
        {
            output += std::to_string (rank++) + ". ";
            output += r.plan.describe();
            output += "\n   Cycles/sample: " + std::to_string (r.timing.cyclesPerSample);
            output += ", CPU%: " + std::to_string (r.timing.cpuPercent) + "%\n";
        }

        return output;
    }

    /**
     * @brief Print auto-tune results to stdout.
     *
     * @param result Auto-tune result to print
     */
    inline void printAutoTuneResult (const AutoTuneResult& result)
    {
        std::cout << formatAutoTuneResult (result);
    }

} // namespace PlayfulTones::DspToolbox::Benchmark
