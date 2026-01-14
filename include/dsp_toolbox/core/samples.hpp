/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/core/concepts.hpp"

#include <cstdint>

namespace PlayfulTones::DspToolbox
{

    /**
     * @brief Sample count or position.
     *
     * Structural type (public member) to support use as NTTP.
     *
     * Use int64_t (default) for discrete sample counting.
     * Use double for fractional positions (e.g., wavetable interpolation).
     *
     * @tparam T Numeric type (default: int64_t)
     */
    template <Numeric T = int64_t>
    struct Samples
    {
        T value {};

        constexpr Samples() = default;
        explicit constexpr Samples (T count) noexcept : value (count) {}

        // Arithmetic
        constexpr Samples operator+ (Samples other) const noexcept
        {
            return Samples { value + other.value };
        }

        constexpr Samples& operator+= (Samples other) noexcept
        {
            value += other.value;
            return *this;
        }

        constexpr Samples operator- (Samples other) const noexcept
        {
            return Samples { value - other.value };
        }

        constexpr Samples& operator-= (Samples other) noexcept
        {
            value -= other.value;
            return *this;
        }

        constexpr Samples operator* (T scalar) const noexcept
        {
            return Samples { value * scalar };
        }

        constexpr Samples operator/ (T scalar) const noexcept
        {
            return Samples { value / scalar };
        }

        // Comparison
        constexpr bool operator== (const Samples& other) const noexcept = default;
        constexpr auto operator<=> (const Samples& other) const noexcept = default;
    };

} // namespace PlayfulTones::DspToolbox
