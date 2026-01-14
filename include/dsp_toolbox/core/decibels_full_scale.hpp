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
 * @brief Absolute level in decibels full scale (0 dBFS = digital maximum).
 * @tparam T Numeric type (default: float)
 */
    template <Numeric T = float>
    class DecibelsFullScale
    {
        T value_ {};

    public:
        constexpr DecibelsFullScale() = default;
        explicit constexpr DecibelsFullScale (T dBFS) noexcept : value_ (dBFS) {}

        [[nodiscard]] constexpr T value() const noexcept { return value_; }

        // Explicit conversion to underlying type
        explicit constexpr operator T() const noexcept { return value_; }

        // Comparison
        constexpr bool operator== (const DecibelsFullScale& other) const noexcept = default;
        constexpr auto operator<=> (const DecibelsFullScale& other) const noexcept = default;
    };

} // namespace PlayfulTones::DspToolbox
