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
 * @brief Time duration in seconds.
 * @tparam T Numeric type (default: float)
 */
    template <Numeric T = float>
    class Seconds
    {
        T value_ {};

    public:
        constexpr Seconds() = default;
        explicit constexpr Seconds (T s) noexcept : value_ (s) {}

        [[nodiscard]] constexpr T value() const noexcept { return value_; }

        // Explicit conversion to underlying type
        explicit constexpr operator T() const noexcept { return value_; }

        // Arithmetic
        constexpr Seconds operator+ (Seconds other) const noexcept
        {
            return Seconds { value_ + other.value_ };
        }

        constexpr Seconds& operator+= (Seconds other) noexcept
        {
            value_ += other.value_;
            return *this;
        }

        constexpr Seconds operator- (Seconds other) const noexcept
        {
            return Seconds { value_ - other.value_ };
        }

        constexpr Seconds operator* (T scalar) const noexcept
        {
            return Seconds { value_ * scalar };
        }

        constexpr Seconds operator/ (T scalar) const noexcept
        {
            return Seconds { value_ / scalar };
        }

        // Comparison
        constexpr bool operator== (const Seconds& other) const noexcept = default;
        constexpr auto operator<=> (const Seconds& other) const noexcept = default;
    };

} // namespace PlayfulTones::DspToolbox
