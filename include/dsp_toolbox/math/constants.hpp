/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include <type_traits>

namespace PlayfulTones::DspToolbox::Math
{

    /**
 * @brief Mathematical constant pi.
 * @tparam T Floating-point type (float or double)
 */
    template <typename T>
        requires std::is_floating_point_v<T>
    inline constexpr T pi = T (3.14159265358979323846264338327950288L);

    /**
 * @brief Mathematical constant 2*pi.
 * @tparam T Floating-point type (float or double)
 */
    template <typename T>
        requires std::is_floating_point_v<T>
    inline constexpr T twoPi = T (6.28318530717958647692528676655900577L);

    /**
 * @brief Mathematical constant pi/2.
 * @tparam T Floating-point type (float or double)
 */
    template <typename T>
        requires std::is_floating_point_v<T>
    inline constexpr T halfPi = T (1.57079632679489661923132169163975144L);

    /**
 * @brief Euler's number e.
 * @tparam T Floating-point type (float or double)
 */
    template <typename T>
        requires std::is_floating_point_v<T>
    inline constexpr T e = T (2.71828182845904523536028747135266250L);

    /**
 * @brief Natural logarithm of 2.
 * @tparam T Floating-point type (float or double)
 */
    template <typename T>
        requires std::is_floating_point_v<T>
    inline constexpr T ln2 = T (0.69314718055994530941723212145817657L);

    /**
 * @brief Natural logarithm of 10.
 * @tparam T Floating-point type (float or double)
 */
    template <typename T>
        requires std::is_floating_point_v<T>
    inline constexpr T ln10 = T (2.30258509299404568401799145468436421L);

    /**
 * @brief ln(10) / 40, used for efficient dB to linear conversion.
 *
 * Optimizes 10^(dB/40) = exp(dB * ln10/40) avoiding pow() overhead.
 * @tparam T Floating-point type (float or double)
 */
    template <typename T>
        requires std::is_floating_point_v<T>
    inline constexpr T ln10_over_40 = ln10<T> / T (40);

} // namespace PlayfulTones::DspToolbox::Math
