/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include <chrono>
#include <concepts>
#include <cstdint>
#include <thread>

#if defined(__x86_64__) || defined(_M_X64)
    #include <x86intrin.h>
    #define DSP_BENCHMARK_HAS_RDTSC 1
#elif defined(__aarch64__) || defined(_M_ARM64)
    #define DSP_BENCHMARK_HAS_CNTVCT 1
#endif

namespace PlayfulTones::DspToolbox::Benchmark
{

    /**
     * @brief Concept for clock types usable in benchmarking.
     *
     * | Member | Type | Description |
     * |--------|------|-------------|
     * | `TimePoint` | type alias | Timestamp type |
     * | `now()` | `TimePoint` | Get current time |
     * | `elapsed(start, end)` | `double` | Elapsed value (cycles or seconds) |
     *
     * ## Example Implementation
     * ```cpp
     * struct MyClock {
     *     using TimePoint = std::uint64_t;
     *     static TimePoint now() noexcept { return 0; }
     *     static double elapsed(TimePoint start, TimePoint end) noexcept {
     *         return static_cast<double>(end - start);
     *     }
     * };
     * static_assert(BenchmarkClock<MyClock>);
     * ```
     */
    template <typename C>
    concept BenchmarkClock = requires (typename C::TimePoint tp) {
        { C::now() } noexcept -> std::same_as<typename C::TimePoint>;
        { C::elapsed (tp, tp) } noexcept -> std::convertible_to<double>;
    };

    /**
     * @brief High-resolution CPU cycle counter.
     *
     * Uses platform-specific intrinsics for accurate cycle counting:
     * - x86_64: RDTSCP instruction (serializing variant)
     * - ARM64: CNTVCT_EL0 register
     * - Fallback: std::chrono::high_resolution_clock
     *
     * Returns cycles directly on x86/ARM, nanoseconds on fallback.
     */
    struct CycleClock
    {
        using TimePoint = std::uint64_t;

        /**
         * @brief Get current timestamp.
         *
         * @return Cycle count (x86/ARM) or nanoseconds (fallback)
         */
        [[nodiscard]] static TimePoint now() noexcept
        {
#if defined(DSP_BENCHMARK_HAS_RDTSC)
            unsigned int aux;
            return __rdtscp (&aux);
#elif defined(DSP_BENCHMARK_HAS_CNTVCT)
            std::uint64_t val = 0;
            asm volatile ("mrs %0, cntvct_el0" : "=r" (val));
            return val;
#else
            return static_cast<TimePoint> (
                std::chrono::high_resolution_clock::now().time_since_epoch().count());
#endif
        }

        /**
         * @brief Calculate elapsed cycles between two timestamps.
         *
         * @param start Start timestamp
         * @param end End timestamp
         * @return Elapsed cycles (or nanoseconds on fallback)
         */
        [[nodiscard]] static double elapsed (TimePoint start, TimePoint end) noexcept
        {
            return static_cast<double> (end - start);
        }

        /**
         * @brief Estimate CPU/counter frequency in Hz.
         *
         * Performs a short calibration measurement. Results may vary
         * due to frequency scaling. Cache the result for repeated use.
         *
         * @return Estimated frequency in Hz
         */
        [[nodiscard]] static double estimatedFrequencyHz() noexcept
        {
#if defined(DSP_BENCHMARK_HAS_RDTSC) || defined(DSP_BENCHMARK_HAS_CNTVCT)
            // Measure cycles over a known time period
            constexpr auto calibrationTime = std::chrono::milliseconds (10);

            auto const chronoStart = std::chrono::high_resolution_clock::now();
            auto const cycleStart = now();

            std::this_thread::sleep_for (calibrationTime);

            auto const cycleEnd = now();
            auto const chronoEnd = std::chrono::high_resolution_clock::now();

            auto const elapsedCycles = static_cast<double> (cycleEnd - cycleStart);
            auto const elapsedSeconds = std::chrono::duration<double> (chronoEnd - chronoStart).count();

            return elapsedCycles / elapsedSeconds;
#else
            // Fallback clock returns nanoseconds
            return 1e9;
#endif
        }

        /**
         * @brief Check if this clock provides actual cycle counts.
         *
         * @return true if using rdtsc/cntvct, false if using chrono fallback
         */
        [[nodiscard]] static constexpr bool isCycleAccurate() noexcept
        {
#if defined(DSP_BENCHMARK_HAS_RDTSC) || defined(DSP_BENCHMARK_HAS_CNTVCT)
            return true;
#else
            return false;
#endif
        }
    };

    static_assert (BenchmarkClock<CycleClock>, "CycleClock must satisfy BenchmarkClock");

    /**
     * @brief std::chrono-based clock for portable measurements.
     *
     * Always returns time in seconds. More portable but typically
     * lower resolution than CycleClock.
     */
    struct ChronoClock
    {
        using TimePoint = std::chrono::high_resolution_clock::time_point;

        /**
         * @brief Get current timestamp.
         *
         * @return Current time point
         */
        [[nodiscard]] static TimePoint now() noexcept
        {
            return std::chrono::high_resolution_clock::now();
        }

        /**
         * @brief Calculate elapsed seconds between two timestamps.
         *
         * @param start Start timestamp
         * @param end End timestamp
         * @return Elapsed time in seconds
         */
        [[nodiscard]] static double elapsed (TimePoint start, TimePoint end) noexcept
        {
            return std::chrono::duration<double> (end - start).count();
        }

        /**
         * @brief Check if this clock provides actual cycle counts.
         *
         * @return Always false (chrono provides time, not cycles)
         */
        [[nodiscard]] static constexpr bool isCycleAccurate() noexcept
        {
            return false;
        }
    };

    static_assert (BenchmarkClock<ChronoClock>, "ChronoClock must satisfy BenchmarkClock");

    /**
     * @brief Memory fence to prevent instruction reordering around measurements.
     *
     * Call before and after the code being benchmarked to ensure
     * accurate timing by preventing compiler/CPU reordering.
     */
    inline void benchmarkFence() noexcept
    {
#if defined(DSP_BENCHMARK_HAS_RDTSC)
        _mm_mfence();
#elif defined(DSP_BENCHMARK_HAS_CNTVCT)
        asm volatile ("dmb sy" ::: "memory");
#else
        std::atomic_thread_fence (std::memory_order_seq_cst);
#endif
    }

} // namespace PlayfulTones::DspToolbox::Benchmark
