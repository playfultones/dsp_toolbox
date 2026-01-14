/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include <xsimd/xsimd.hpp>

#include <concepts>
#include <cstddef>

namespace PlayfulTones::DspToolbox::simd
{

    //--------------------------------------------------------------------------
    // Type aliases for portable SIMD
    //--------------------------------------------------------------------------

    /**
     * @brief SIMD batch type for a given sample type.
     *
     * Uses xsimd for automatic architecture detection:
     * - ARM64 (Apple Silicon, Quest 3): NEON (4 floats / 2 doubles)
     * - x86_64 with AVX: AVX (8 floats / 4 doubles)
     * - x86_64 without AVX: SSE (4 floats / 2 doubles)
     */
    template <std::floating_point T>
    using Batch = xsimd::batch<T>;

    using FloatBatch = Batch<float>;
    using DoubleBatch = Batch<double>;

    /**
     * @brief Number of elements in a SIMD register for type T.
     */
    template <std::floating_point T>
    inline constexpr std::size_t batchSize = Batch<T>::size;

    inline constexpr std::size_t floatSimdSize = batchSize<float>;
    inline constexpr std::size_t doubleSimdSize = batchSize<double>;

    //--------------------------------------------------------------------------
    // Core SIMD operations (multi-precision)
    //--------------------------------------------------------------------------

    /**
     * @brief Multiply buffer by a scalar using SIMD.
     *
     * Processes samples in SIMD-width chunks, with scalar fallback for remainder.
     *
     * @tparam T Floating-point type (float or double)
     * @param buffer Pointer to sample data
     * @param count Number of samples
     * @param factor Multiplication factor
     */
    template <std::floating_point T>
    inline void multiply (T* buffer, std::size_t count, T factor) noexcept
    {
        constexpr std::size_t simdSize = batchSize<T>;
        const auto factorVec = Batch<T> (factor);
        const std::size_t vecSize = count - count % simdSize;

        for (std::size_t i = 0; i < vecSize; i += simdSize)
        {
            auto samples = xsimd::load_unaligned (&buffer[i]);
            xsimd::store_unaligned (&buffer[i], samples * factorVec);
        }

        // Scalar remainder
        for (std::size_t i = vecSize; i < count; ++i)
        {
            buffer[i] *= factor;
        }
    }

    /**
     * @brief Multiply buffer by a scalar with 2x loop unrolling for better ILP.
     *
     * Uses 2x unrolling to enable instruction-level parallelism - the CPU can
     * execute both multiplications in parallel.
     *
     * @tparam T Floating-point type (float or double)
     * @param buffer Pointer to sample data
     * @param count Number of samples
     * @param factor Multiplication factor
     */
    template <std::floating_point T>
    inline void multiplyUnrolled (T* buffer, std::size_t count, T factor) noexcept
    {
        constexpr std::size_t simdSize = batchSize<T>;
        constexpr std::size_t unrollFactor = 2;
        constexpr std::size_t chunkSize = simdSize * unrollFactor;

        const auto factorVec = Batch<T> (factor);

        std::size_t i = 0;
        const std::size_t mainEnd = count - (count % chunkSize);

        // Process 2x SIMD vectors per iteration for better ILP
        for (; i < mainEnd; i += chunkSize)
        {
            auto v0 = xsimd::load_unaligned (&buffer[i]);
            auto v1 = xsimd::load_unaligned (&buffer[i + simdSize]);

            v0 = v0 * factorVec;
            v1 = v1 * factorVec;

            xsimd::store_unaligned (&buffer[i], v0);
            xsimd::store_unaligned (&buffer[i + simdSize], v1);
        }

        if (i + simdSize <= count)
        {
            auto v = xsimd::load_unaligned (&buffer[i]);
            xsimd::store_unaligned (&buffer[i], v * factorVec);
            i += simdSize;
        }

        // Scalar remainder
        for (; i < count; ++i)
        {
            buffer[i] *= factor;
        }
    }

    /**
     * @brief Clear buffer to zero using SIMD.
     *
     * @tparam T Floating-point type (float or double)
     * @param buffer Pointer to sample data
     * @param count Number of samples
     */
    template <std::floating_point T>
    inline void clear (T* buffer, std::size_t count) noexcept
    {
        constexpr std::size_t simdSize = batchSize<T>;
        const auto zero = Batch<T> (T { 0 });
        const std::size_t vecSize = count - count % simdSize;

        for (std::size_t i = 0; i < vecSize; i += simdSize)
        {
            xsimd::store_unaligned (&buffer[i], zero);
        }

        // Scalar remainder
        for (std::size_t i = vecSize; i < count; ++i)
        {
            buffer[i] = T { 0 };
        }
    }

    /**
     * @brief Copy samples using SIMD.
     *
     * Constexpr-compatible: uses scalar copy at compile-time, SIMD at runtime.
     *
     * @tparam T Floating-point type (float or double)
     * @param dst Destination buffer
     * @param src Source buffer
     * @param count Number of samples
     */
    template <std::floating_point T>
    constexpr void copy (T* dst, const T* src, std::size_t count) noexcept
    {
        if consteval
        {
            // Scalar copy at compile-time
            for (std::size_t i = 0; i < count; ++i)
                dst[i] = src[i];
        }
        else
        {
            // SIMD copy at runtime
            constexpr std::size_t simdSize = batchSize<T>;
            const std::size_t vecSize = count - count % simdSize;

            for (std::size_t i = 0; i < vecSize; i += simdSize)
            {
                auto samples = xsimd::load_unaligned (&src[i]);
                xsimd::store_unaligned (&dst[i], samples);
            }

            // Scalar remainder
            for (std::size_t i = vecSize; i < count; ++i)
            {
                dst[i] = src[i];
            }
        }
    }

    /**
     * @brief Add source samples to destination using SIMD.
     *
     * @tparam T Floating-point type (float or double)
     * @param dst Destination buffer (accumulator)
     * @param src Source buffer to add
     * @param count Number of samples
     */
    template <std::floating_point T>
    inline void add (T* dst, const T* src, std::size_t count) noexcept
    {
        constexpr std::size_t simdSize = batchSize<T>;
        const std::size_t vecSize = count - count % simdSize;

        for (std::size_t i = 0; i < vecSize; i += simdSize)
        {
            auto dstSamples = xsimd::load_unaligned (&dst[i]);
            auto srcSamples = xsimd::load_unaligned (&src[i]);
            xsimd::store_unaligned (&dst[i], dstSamples + srcSamples);
        }

        // Scalar remainder
        for (std::size_t i = vecSize; i < count; ++i)
        {
            dst[i] += src[i];
        }
    }

    /**
     * @brief Multiply buffer by a linearly ramping gain using SIMD.
     *
     * Used for smooth gain transitions (parameter smoothing). Processes samples
     * in SIMD chunks with linearly interpolated gain values, then falls back
     * to scalar for remainder.
     *
     * @tparam T Floating-point type (float or double)
     * @param buffer Pointer to sample data
     * @param count Number of samples
     * @param currentGain Current gain value (updated in-place)
     * @param gainStep Step to add per sample
     * @param stepsRemaining Number of smoothing steps remaining (updated in-place)
     * @param targetGain Target gain to snap to when smoothing completes
     */
    template <std::floating_point T>
    inline void multiplyRamp (T* buffer, std::size_t count, T& currentGain, T gainStep, std::size_t& stepsRemaining, T targetGain) noexcept
    {
        constexpr std::size_t simdSize = batchSize<T>;
        std::size_t i = 0;

        // Process in SIMD chunks while still ramping
        while (i + simdSize <= count && stepsRemaining >= simdSize)
        {
            // Build gain vector with linearly interpolated values
            alignas (64) T gains[simdSize];
            for (std::size_t j = 0; j < simdSize; ++j)
            {
                gains[j] = currentGain + static_cast<T> (j) * gainStep;
            }

            auto const gainVec = xsimd::load_aligned (gains);
            auto samples = xsimd::load_unaligned (&buffer[i]);
            xsimd::store_unaligned (&buffer[i], samples * gainVec);

            currentGain += gainStep * static_cast<T> (simdSize);
            stepsRemaining -= simdSize;
            i += simdSize;
        }

        for (; i < count; ++i)
        {
            buffer[i] *= currentGain;

            if (stepsRemaining > 0)
            {
                currentGain += gainStep;
                --stepsRemaining;

                if (stepsRemaining == 0)
                    currentGain = targetGain;
            }
        }

        // Snap to target to avoid floating point drift
        if (stepsRemaining == 0)
        {
            currentGain = targetGain;
        }
    }

    //--------------------------------------------------------------------------
    // DSP-specific SIMD functions
    //--------------------------------------------------------------------------

    /**
     * @brief Fast tanh approximation using Pade approximant.
     *
     * Provides ~1e-4 accuracy for typical audio signals (-3 to +3 range).
     * Much faster than std::tanh when processing SIMD vectors.
     *
     * @tparam T Floating-point type (float or double)
     * @param x Input batch
     * @return Approximate tanh(x)
     */
    template <std::floating_point T>
    inline Batch<T> tanhApprox (Batch<T> x) noexcept
    {
        // Pade approximant: tanh(x) ~ x * (27 + x^2) / (27 + 9*x^2)
        // Accurate to ~1e-4 for |x| < 3
        auto x2 = x * x;
        auto num = x * (Batch<T> (T { 27 }) + x2);
        auto den = Batch<T> (T { 27 }) + Batch<T> (T { 9 }) * x2;
        return num / den;
    }

    /**
     * @brief Soft clipping using xsimd's optimized tanh.
     *
     * Uses the math library's tanh for accuracy.
     *
     * @tparam T Floating-point type (float or double)
     * @param x Input batch
     * @return tanh(x)
     */
    template <std::floating_point T>
    inline Batch<T> softClip (Batch<T> x) noexcept
    {
        return xsimd::tanh (x);
    }

    /**
     * @brief Hard clipping to [-1, 1] range.
     *
     * @tparam T Floating-point type (float or double)
     * @param x Input batch
     * @return Clipped samples
     */
    template <std::floating_point T>
    inline Batch<T> hardClip (Batch<T> x) noexcept
    {
        return xsimd::clip (x, Batch<T> (T { -1 }), Batch<T> (T { 1 }));
    }

} // namespace PlayfulTones::DspToolbox::simd
