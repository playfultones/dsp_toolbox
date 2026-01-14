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
 * @brief Time duration in milliseconds.
 * @tparam T Numeric type (default: float)
 */
    template <Numeric T = float>
    class Milliseconds
    {
        T value_ {};

    public:
        constexpr Milliseconds() = default;
        explicit constexpr Milliseconds (T ms) noexcept : value_ (ms) {}

        [[nodiscard]] constexpr T value() const noexcept { return value_; }

        // Explicit conversion to underlying type
        explicit constexpr operator T() const noexcept { return value_; }

        // Arithmetic
        constexpr Milliseconds operator+ (Milliseconds other) const noexcept
        {
            return Milliseconds { value_ + other.value_ };
        }

        constexpr Milliseconds& operator+= (Milliseconds other) noexcept
        {
            value_ += other.value_;
            return *this;
        }

        constexpr Milliseconds operator- (Milliseconds other) const noexcept
        {
            return Milliseconds { value_ - other.value_ };
        }

        constexpr Milliseconds operator* (T scalar) const noexcept
        {
            return Milliseconds { value_ * scalar };
        }

        constexpr Milliseconds operator/ (T scalar) const noexcept
        {
            return Milliseconds { value_ / scalar };
        }

        // Comparison
        constexpr bool operator== (const Milliseconds& other) const noexcept = default;
        constexpr auto operator<=> (const Milliseconds& other) const noexcept = default;
    };

} // namespace PlayfulTones::DspToolbox
