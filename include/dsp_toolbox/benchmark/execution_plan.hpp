/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/benchmark/benchmark.hpp"
#include "dsp_toolbox/core/process_spec.hpp"
#include "dsp_toolbox/simd/block.hpp"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace PlayfulTones::DspToolbox::Benchmark
{

    /**
     * @brief Describes a specific execution strategy for benchmarking.
     *
     * Execution plans are independent of the algorithm - they control
     * how the algorithm is executed (block sizes, SIMD strategy, etc.).
     * This enables empirical testing of which approach works best on
     * current hardware.
     */
    struct ExecutionPlan
    {
        /// Vectorization strategy (time-parallel, voice-parallel, hybrid)
        simd::VectorizationStrategy strategy { simd::VectorizationStrategy::TimeParallel };

        /// Processing block size in samples
        std::size_t blockSize { 512 };

        /// Number of voices to process together (for voice-parallel)
        std::size_t voiceGroupSize { 1 };

        /// Whether to use range hoisting for SIMD chunks
        bool useRangeHoisting { true };

        /// Chunk sizes for range hoisting (e.g., {8, 4, 2, 1})
        std::array<std::size_t, 4> hoistChunks { 8, 4, 2, 1 };

        /**
         * @brief Get human-readable description of this plan.
         *
         * @return String describing the plan's configuration
         */
        [[nodiscard]] std::string describe() const
        {
            std::string desc;
            desc.reserve (128);

            switch (strategy)
            {
                case simd::VectorizationStrategy::TimeParallel:
                    desc = "TimeParallel";
                    break;
                case simd::VectorizationStrategy::VoiceParallel:
                    desc = "VoiceParallel";
                    break;
                case simd::VectorizationStrategy::Hybrid:
                    desc = "Hybrid";
                    break;
            }

            desc += ", block=" + std::to_string (blockSize);
            desc += ", voices=" + std::to_string (voiceGroupSize);

            if (useRangeHoisting)
            {
                desc += ", hoist={";
                for (std::size_t i = 0; i < hoistChunks.size(); ++i)
                {
                    if (i > 0)
                        desc += ",";
                    desc += std::to_string (hoistChunks[i]);
                }
                desc += "}";
            }

            return desc;
        }

        constexpr bool operator== (const ExecutionPlan&) const noexcept = default;
    };

    /**
     * @brief Result of benchmarking a specific execution plan.
     */
    struct PlanBenchmarkResult
    {
        ExecutionPlan plan;
        BenchmarkResult timing;

        /**
         * @brief Get cycles per sample from this result.
         */
        [[nodiscard]] double cyclesPerSample() const noexcept
        {
            return timing.cyclesPerSample;
        }
    };

    /**
     * @brief Generate candidate execution plans for auto-tuning.
     *
     * Creates a set of reasonable plans to benchmark based on the
     * processing specification. Plans vary by strategy, block size,
     * and voice grouping.
     *
     * @param spec Processing specification (affects valid block sizes)
     * @return Vector of candidate plans to benchmark
     */
    [[nodiscard]] inline std::vector<ExecutionPlan> generateCandidatePlans (
        [[maybe_unused]] const ProcessSpec& spec)
    {
        std::vector<ExecutionPlan> plans;
        plans.reserve (15);

        // Block sizes to test (must not exceed spec.maxBlockSize)
        std::array<std::size_t, 3> const blockSizes { 128, 256, 512 };

        // TimeParallel variants
        for (auto blockSize : blockSizes)
        {
            if (blockSize <= spec.maxBlockSize.value)
            {
                plans.push_back ({
                    .strategy = simd::VectorizationStrategy::TimeParallel,
                    .blockSize = blockSize,
                    .voiceGroupSize = 1,
                    .useRangeHoisting = true,
                    .hoistChunks = { 8, 4, 2, 1 },
                });
            }
        }

        // VoiceParallel variants
        std::array<std::size_t, 2> const voiceGroups { 4, 8 };
        for (auto voiceGroup : voiceGroups)
        {
            plans.push_back ({
                .strategy = simd::VectorizationStrategy::VoiceParallel,
                .blockSize = spec.maxBlockSize.value,
                .voiceGroupSize = voiceGroup,
                .useRangeHoisting = true,
                .hoistChunks = { 4, 2, 1, 1 },
            });
        }

        // Hybrid variants
        plans.push_back ({
            .strategy = simd::VectorizationStrategy::Hybrid,
            .blockSize = spec.maxBlockSize.value,
            .voiceGroupSize = 4,
            .useRangeHoisting = true,
            .hoistChunks = { 8, 4, 2, 1 },
        });

        // No-hoisting variant for comparison
        plans.push_back ({
            .strategy = simd::VectorizationStrategy::TimeParallel,
            .blockSize = spec.maxBlockSize.value,
            .voiceGroupSize = 1,
            .useRangeHoisting = false,
            .hoistChunks = { 1, 1, 1, 1 },
        });

        return plans;
    }

    /**
     * @brief Generate plans for specific block sizes.
     *
     * Compile-time plan generation for when block sizes are known.
     *
     * @tparam BlockSizes Block sizes to generate plans for
     * @return Array of plans covering all strategies for each block size
     */
    template <std::size_t... BlockSizes>
    [[nodiscard]] constexpr auto generatePlansForBlockSizes()
    {
        constexpr std::size_t numSizes = sizeof...(BlockSizes);
        constexpr std::size_t numStrategies = 3; // Time, Voice, Hybrid
        std::array<ExecutionPlan, numSizes * numStrategies> plans {};

        std::array<std::size_t, numSizes> const sizes { BlockSizes... };
        std::size_t idx = 0;

        for (auto size : sizes)
        {
            plans[idx++] = {
                .strategy = simd::VectorizationStrategy::TimeParallel,
                .blockSize = size,
                .voiceGroupSize = 1,
                .useRangeHoisting = true,
                .hoistChunks = { 8, 4, 2, 1 },
            };
            plans[idx++] = {
                .strategy = simd::VectorizationStrategy::VoiceParallel,
                .blockSize = size,
                .voiceGroupSize = 4,
                .useRangeHoisting = true,
                .hoistChunks = { 4, 2, 1, 1 },
            };
            plans[idx++] = {
                .strategy = simd::VectorizationStrategy::Hybrid,
                .blockSize = size,
                .voiceGroupSize = 4,
                .useRangeHoisting = true,
                .hoistChunks = { 8, 4, 2, 1 },
            };
        }

        return plans;
    }

} // namespace PlayfulTones::DspToolbox::Benchmark
