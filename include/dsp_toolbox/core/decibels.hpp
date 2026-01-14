/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/core/concepts.hpp"

namespace PlayfulTones::DspToolbox
{

    /**
 * @brief Relative level in decibels for gain/attenuation.
 * @tparam T Numeric type (default: float)
 */
    template <Numeric T = float>
    class Decibels
    {
        T value_ {};

    public:
        constexpr Decibels() = default;
        explicit constexpr Decibels (T dB) noexcept : value_ (dB) {}

        [[nodiscard]] constexpr T value() const noexcept { return value_; }

        // Explicit conversion to underlying type
        explicit constexpr operator T() const noexcept { return value_; }

        // Arithmetic
        constexpr Decibels operator-() const noexcept { return Decibels { -value_ }; }

        constexpr Decibels operator+ (Decibels other) const noexcept
        {
            return Decibels { value_ + other.value_ };
        }

        constexpr Decibels& operator+= (Decibels other) noexcept
        {
            value_ += other.value_;
            return *this;
        }

        constexpr Decibels operator- (Decibels other) const noexcept
        {
            return Decibels { value_ - other.value_ };
        }

        constexpr Decibels& operator-= (Decibels other) noexcept
        {
            value_ -= other.value_;
            return *this;
        }

        // Comparison
        constexpr bool operator== (const Decibels& other) const noexcept = default;
        constexpr auto operator<=> (const Decibels& other) const noexcept = default;
    };

} // namespace PlayfulTones::DspToolbox
