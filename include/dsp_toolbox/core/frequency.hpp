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
 * @brief Frequency in Hertz.
 * @tparam T Numeric type (default: float)
 */
    template <Numeric T = float>
    class Frequency
    {
        T value_ {};

    public:
        constexpr Frequency() = default;
        explicit constexpr Frequency (T hz) noexcept : value_ (hz) {}

        [[nodiscard]] constexpr T value() const noexcept { return value_; }

        // Explicit conversion to underlying type
        explicit constexpr operator T() const noexcept { return value_; }

        // Arithmetic with scalars
        constexpr Frequency operator* (T scalar) const noexcept
        {
            return Frequency { value_ * scalar };
        }

        constexpr Frequency& operator*= (T scalar) noexcept
        {
            value_ *= scalar;
            return *this;
        }

        constexpr Frequency operator/ (T scalar) const noexcept
        {
            return Frequency { value_ / scalar };
        }

        constexpr Frequency& operator/= (T scalar) noexcept
        {
            value_ /= scalar;
            return *this;
        }

        // Ratio of two frequencies
        constexpr T operator/ (Frequency other) const noexcept
        {
            return value_ / other.value_;
        }

        // Comparison
        constexpr bool operator== (const Frequency& other) const noexcept = default;
        constexpr auto operator<=> (const Frequency& other) const noexcept = default;
    };

    // Scalar * Frequency
    template <Numeric T>
    constexpr Frequency<T> operator* (T scalar, Frequency<T> freq) noexcept
    {
        return freq * scalar;
    }

} // namespace PlayfulTones::DspToolbox
