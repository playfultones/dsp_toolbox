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
 * @brief Linear gain multiplier (1.0 = unity gain).
 * @tparam T Numeric type (default: float)
 */
    template <Numeric T = float>
    class LinearGain
    {
        T value_ { T (1) };

    public:
        constexpr LinearGain() = default;
        explicit constexpr LinearGain (T gain) noexcept : value_ (gain) {}

        [[nodiscard]] constexpr T value() const noexcept { return value_; }

        // Explicit conversion to underlying type
        explicit constexpr operator T() const noexcept { return value_; }

        // Arithmetic - gains multiply
        constexpr LinearGain operator* (LinearGain other) const noexcept
        {
            return LinearGain { value_ * other.value_ };
        }

        constexpr LinearGain& operator*= (LinearGain other) noexcept
        {
            value_ *= other.value_;
            return *this;
        }

        // Comparison
        constexpr bool operator== (const LinearGain& other) const noexcept = default;
        constexpr auto operator<=> (const LinearGain& other) const noexcept = default;
    };

} // namespace PlayfulTones::DspToolbox
