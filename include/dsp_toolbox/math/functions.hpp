/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/math/constants.hpp"

#include <limits>
#include <type_traits>
#include <utility>

namespace PlayfulTones::DspToolbox::Math
{

    /**
     * @brief Exact floating-point equality comparison.
     *
     * Use this when you intentionally want to compare floating-point values
     * for exact bit-pattern equality (e.g., checking for exactly 0.0 or 1.0
     * in mathematical edge cases).
     *
     * @tparam T Floating-point type
     * @param a First value
     * @param b Second value
     * @return true if a and b are exactly equal (same bit pattern)
     */
    template <typename T>
        requires std::is_floating_point_v<T>
    constexpr auto exactlyEquals (T a, T b) noexcept -> bool
    {
#if defined(__clang__) || defined(__GNUC__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wfloat-equal"
#endif
        return a == b;
#if defined(__clang__) || defined(__GNUC__)
    #pragma GCC diagnostic pop
#endif
    }

    /**
 * @brief Constexpr absolute value.
 * @tparam T Floating-point type
 * @param x Input value
 * @return Absolute value of x
 */
    template <typename T>
        requires std::is_floating_point_v<T>
    constexpr auto abs (T x) noexcept -> T
    {
        return x < T (0) ? -x : x;
    }

    /**
 * @brief Constexpr floor function.
 * @tparam T Floating-point type
 * @param x Input value
 * @return Largest integer not greater than x
 */
    template <typename T>
        requires std::is_floating_point_v<T>
    constexpr auto floor (T x) noexcept -> T
    {
        auto const truncated = static_cast<T> (static_cast<long long> (x));
        return (x < truncated) ? truncated - T (1) : truncated;
    }

    /**
 * @brief Constexpr floating-point modulo.
 * @tparam T Floating-point type
 * @param x Dividend
 * @param y Divisor
 * @return Remainder of x/y
 */
    template <typename T>
        requires std::is_floating_point_v<T>
    constexpr auto fmod (T x, T y) noexcept -> T
    {
        return x - floor (x / y) * y;
    }

    /**
 * @brief Constexpr square root using Newton-Raphson iteration.
 *
 * @tparam T Floating-point type
 * @param x Non-negative input value
 * @return Square root of x
 *
 * @note Returns 0 for x <= 0 (including handling of negative inputs).
 *       Accuracy target: < 1e-7 relative error.
 */
    template <typename T>
        requires std::is_floating_point_v<T>
    constexpr auto sqrt (T x) noexcept -> T
    {
        if (x != x) // NaN
        {
            return x;
        }
        if (x <= T (0))
        {
            return T (0);
        }
        if (x == std::numeric_limits<T>::infinity())
        {
            return std::numeric_limits<T>::infinity();
        }

        // Initial guess using bit manipulation approximation
        T guess = x;

        // Scale for better convergence on extreme values
        T scale = T (1);
        T scaledX = x;

        // Scale extreme values to [1e-10, 1e10] for Newton-Raphson convergence
        while (scaledX > T (1e10))
        {
            scaledX *= T (0.25);
            scale *= T (2);
        }
        while (scaledX < T (1e-10) && scaledX > T (0))
        {
            scaledX *= T (4);
            scale *= T (0.5);
        }

        guess = scaledX * T (0.5);

        // Newton-Raphson iterations
        constexpr int maxIterations = 20;
        for (int i = 0; i < maxIterations; ++i)
        {
            T const newGuess = T (0.5) * (guess + scaledX / guess);
            if (abs (newGuess - guess) < std::numeric_limits<T>::epsilon() * guess)
            {
                break;
            }
            guess = newGuess;
        }

        return guess * scale;
    }

    /**
 * @brief Constexpr exponential function using Taylor series with range reduction.
 *
 * @tparam T Floating-point type
 * @param x Input value
 * @return e^x
 *
 * @note Uses range reduction: e^x = e^k * e^r where k is integer and |r| < 0.5.
 *       Accuracy target: < 1e-6 relative error.
 */
    template <typename T>
        requires std::is_floating_point_v<T>
    constexpr auto exp (T x) noexcept -> T
    {
        if (x != x) // NaN
        {
            return x;
        }

        constexpr T overflowThreshold = std::is_same_v<T, float> ? T (88) : T (709);
        constexpr T underflowThreshold = std::is_same_v<T, float> ? T (-88) : T (-709);

        if (x > overflowThreshold)
        {
            return std::numeric_limits<T>::infinity();
        }
        if (x < underflowThreshold)
        {
            return T (0);
        }

        // Range reduction: e^x = 2^k * e^r, where r = x - k*ln(2)
        // and k = round(x / ln(2))
        T const k = floor (x / ln2<T> + T (0.5));
        T const r = x - k * ln2<T>;

        // Taylor series for e^r, |r| < ln(2)/2 ≈ 0.347
        // e^r = 1 + r + r^2/2! + r^3/3! + ...
        T result = T (1);
        T term = T (1);

        constexpr int maxTerms = 25;
        for (int n = 1; n < maxTerms; ++n)
        {
            term *= r / static_cast<T> (n);
            result += term;
            if (abs (term) < std::numeric_limits<T>::epsilon() * abs (result))
            {
                break;
            }
        }

        auto const intK = static_cast<int> (k);
        if (intK > 0)
        {
            for (int i = 0; i < intK; ++i)
            {
                result *= T (2);
            }
        }
        else if (intK < 0)
        {
            for (int i = 0; i < -intK; ++i)
            {
                result *= T (0.5);
            }
        }

        return result;
    }

    /**
 * @brief Constexpr natural logarithm using series expansion with range reduction.
 *
 * @tparam T Floating-point type
 * @param x Positive input value
 * @return Natural logarithm of x
 *
 * @note Returns -infinity for x <= 0.
 *       Uses range reduction: log(x) = log(m * 2^e) = log(m) + e*ln(2).
 *       Accuracy target: < 1e-6 relative error.
 */
    template <typename T>
        requires std::is_floating_point_v<T>
    constexpr auto log (T x) noexcept -> T
    {
        if (x != x) // NaN
        {
            return x;
        }
        if (x <= T (0))
        {
            return -std::numeric_limits<T>::infinity();
        }
        if (x == std::numeric_limits<T>::infinity())
        {
            return std::numeric_limits<T>::infinity();
        }
        if (exactlyEquals (x, T (1)))
        {
            return T (0);
        }

        // Range reduction: find e such that x = m * 2^e, where 0.5 <= m < 1
        // Actually, we'll reduce to 1/sqrt(2) <= m <= sqrt(2) for better convergence
        int exponent = 0;
        T mantissa = x;

        while (mantissa >= T (2))
        {
            mantissa *= T (0.5);
            ++exponent;
        }
        while (mantissa < T (0.5))
        {
            mantissa *= T (2);
            --exponent;
        }

        // Further adjust to center around 1
        // log(m) using the series: log((1+y)/(1-y)) = 2(y + y^3/3 + y^5/5 + ...)
        // where y = (m-1)/(m+1)
        T const y = (mantissa - T (1)) / (mantissa + T (1));
        T const y2 = y * y;

        T result = T (0);
        T term = y;

        constexpr int maxTerms = 50;
        for (int n = 1; n < maxTerms * 2; n += 2)
        {
            result += term / static_cast<T> (n);
            term *= y2;
            if (abs (term) < std::numeric_limits<T>::epsilon())
            {
                break;
            }
        }

        result *= T (2);

        // Add back the exponent contribution
        return result + static_cast<T> (exponent) * ln2<T>;
    }

    /**
 * @brief Constexpr power function.
 *
 * @tparam T Floating-point type
 * @param base Base value
 * @param exponent Exponent value
 * @return base^exponent
 *
 * @note Computed as exp(exponent * log(base)).
 *       Accuracy target: < 1e-5 relative error.
 */
    template <typename T>
        requires std::is_floating_point_v<T>
    constexpr auto pow (T base, T exponent) noexcept -> T
    {
        if (exactlyEquals (exponent, T (0)))
        {
            return T (1);
        }
        if (exactlyEquals (base, T (0)))
        {
            return exponent > T (0) ? T (0) : std::numeric_limits<T>::infinity();
        }
        if (exactlyEquals (base, T (1)))
        {
            return T (1);
        }

        if (base < T (0))
        {
            T const intPart = floor (exponent);
            if (exactlyEquals (exponent, intPart))
            {
                T const result = exp (exponent * log (-base));
                auto const intExp = static_cast<long long> (intPart);
                return (intExp % 2 != 0) ? -result : result;
            }
            return std::numeric_limits<T>::quiet_NaN();
        }

        return exp (exponent * log (base));
    }

    /**
 * @brief Constexpr sine function using Taylor series with range reduction.
 *
 * @tparam T Floating-point type
 * @param x Angle in radians
 * @return Sine of x
 *
 * @note Uses range reduction to [-pi, pi], then Taylor series.
 *       Accuracy target: < 1e-6 relative error.
 */
    template <typename T>
        requires std::is_floating_point_v<T>
    constexpr auto sin (T x) noexcept -> T
    {
        // Range reduction to [-pi, pi]
        x = fmod (x, twoPi<T>);
        if (x > pi<T>)
        {
            x -= twoPi<T>;
        }
        else if (x < -pi<T>)
        {
            x += twoPi<T>;
        }

        // Further reduce to [-pi/2, pi/2] using sin(x) = sin(pi - x)
        if (x > halfPi<T>)
        {
            x = pi<T> - x;
        }
        else if (x < -halfPi<T>)
        {
            x = -pi<T> - x;
        }

        // Taylor series: sin(x) = x - x^3/3! + x^5/5! - x^7/7! + ...
        T const x2 = x * x;
        T result = x;
        T term = x;

        constexpr int maxTerms = 15;
        for (int n = 1; n < maxTerms; ++n)
        {
            term *= -x2 / static_cast<T> ((2 * n) * (2 * n + 1));
            result += term;
            if (abs (term) < std::numeric_limits<T>::epsilon() * abs (result))
            {
                break;
            }
        }

        return result;
    }

    /**
 * @brief Constexpr cosine function.
 *
 * @tparam T Floating-point type
 * @param x Angle in radians
 * @return Cosine of x
 *
 * @note Computed as sin(x + pi/2).
 *       Accuracy target: < 1e-6 relative error.
 */
    template <typename T>
        requires std::is_floating_point_v<T>
    constexpr auto cos (T x) noexcept -> T
    {
        return sin (x + halfPi<T>);
    }

    /**
 * @brief Constexpr tangent function.
 *
 * @tparam T Floating-point type
 * @param x Angle in radians
 * @return Tangent of x
 *
 * @note Computed as sin(x) / cos(x).
 *       Accuracy target: < 1e-6 relative error.
 */
    template <typename T>
        requires std::is_floating_point_v<T>
    constexpr auto tan (T x) noexcept -> T
    {
        T const c = cos (x);
        if (abs (c) < std::numeric_limits<T>::epsilon())
        {
            return (sin (x) >= T (0)) ? std::numeric_limits<T>::infinity()
                                      : -std::numeric_limits<T>::infinity();
        }
        return sin (x) / c;
    }

    /**
 * @brief Constexpr hyperbolic tangent function.
 *
 * @tparam T Floating-point type
 * @param x Input value
 * @return Hyperbolic tangent of x
 *
 * @note Computed as (e^2x - 1) / (e^2x + 1).
 *       Accuracy target: < 1e-6 relative error.
 */
    template <typename T>
        requires std::is_floating_point_v<T>
    constexpr auto tanh (T x) noexcept -> T
    {
        // For large |x|, tanh approaches ±1
        if (x > T (20))
        {
            return T (1);
        }
        if (x < T (-20))
        {
            return T (-1);
        }

        // tanh(x) = (e^2x - 1) / (e^2x + 1)
        // For numerical stability, use: tanh(x) = (e^x - e^-x) / (e^x + e^-x)
        // Or equivalently: tanh(x) = 1 - 2/(e^2x + 1)
        T const e2x = exp (T (2) * x);
        return (e2x - T (1)) / (e2x + T (1));
    }

    /**
 * @brief Constexpr arctangent function using Taylor series.
 *
 * @tparam T Floating-point type
 * @param x Input value
 * @return Arctangent of x in radians
 *
 * @note Uses identity transformations for |x| > 1.
 *       Accuracy target: < 1e-6 relative error.
 */
    template <typename T>
        requires std::is_floating_point_v<T>
    constexpr auto atan (T x) noexcept -> T
    {
        if (exactlyEquals (x, T (0)))
        {
            return T (0);
        }

        bool const negate = x < T (0);
        if (negate)
        {
            x = -x;
        }

        bool const invert = x > T (1);
        if (invert)
        {
            x = T (1) / x;
        }

        // For better convergence, reduce further if x > tan(pi/12) ≈ 0.268
        // Using: atan(x) = pi/6 + atan((x - 1/sqrt(3)) / (1 + x/sqrt(3)))
        bool const reduce = x > T (0.268);
        if (reduce)
        {
            constexpr T sqrt3 = T (1.7320508075688772935L);
            constexpr T invSqrt3 = T (1) / sqrt3;
            x = (x - invSqrt3) / (T (1) + x * invSqrt3);
        }

        // Taylor series for |x| <= tan(pi/12)
        // atan(x) = x - x^3/3 + x^5/5 - x^7/7 + ...
        T const x2 = x * x;
        T result = x;
        T term = x;

        constexpr int maxTerms = 30;
        for (int n = 1; n < maxTerms; ++n)
        {
            term *= -x2;
            T const contribution = term / static_cast<T> (2 * n + 1);
            result += contribution;
            if (abs (contribution) < std::numeric_limits<T>::epsilon() * abs (result))
            {
                break;
            }
        }

        if (reduce)
        {
            constexpr T piOver6 = pi<T> / T (6);
            result += piOver6;
        }

        if (invert)
        {
            result = halfPi<T> - result;
        }

        return negate ? -result : result;
    }

    /**
 * @brief Constexpr two-argument arctangent function.
 *
 * @tparam T Floating-point type
 * @param y Y coordinate
 * @param x X coordinate
 * @return Angle in radians in the range [-pi, pi]
 *
 * @note Returns the angle of the point (x, y) from the positive x-axis.
 */
    template <typename T>
        requires std::is_floating_point_v<T>
    constexpr auto atan2 (T y, T x) noexcept -> T
    {
        if (x > T (0))
        {
            return atan (y / x);
        }
        if (x < T (0))
        {
            if (y >= T (0))
            {
                return atan (y / x) + pi<T>;
            }
            return atan (y / x) - pi<T>;
        }
        if (y > T (0))
        {
            return halfPi<T>;
        }
        if (y < T (0))
        {
            return -halfPi<T>;
        }
        return T (0);
    }

    /**
 * @brief Constexpr simultaneous sine and cosine computation.
 *
 * More efficient than calling sin() and cos() separately as it shares
 * range reduction and can exploit the identity cos(x) = sin(x + pi/2).
 *
 * @tparam T Floating-point type
 * @param x Angle in radians
 * @return Pair of (sin(x), cos(x))
 *
 * @note Accuracy target: < 1e-6 relative error.
 */
    template <typename T>
        requires std::is_floating_point_v<T>
    constexpr auto sincos (T x) noexcept -> std::pair<T, T>
    {
        // Range reduction to [-pi, pi]
        T xReduced = fmod (x, twoPi<T>);
        if (xReduced > pi<T>)
        {
            xReduced -= twoPi<T>;
        }
        else if (xReduced < -pi<T>)
        {
            xReduced += twoPi<T>;
        }

        // Further reduce to [-pi/2, pi/2] for sin
        T sinX = xReduced;
        if (sinX > halfPi<T>)
        {
            sinX = pi<T> - sinX;
        }
        else if (sinX < -halfPi<T>)
        {
            sinX = -pi<T> - sinX;
        }

        // Compute sin using Taylor series
        T const x2 = sinX * sinX;
        T sinResult = sinX;
        T term = sinX;

        constexpr int maxTerms = 15;
        for (int n = 1; n < maxTerms; ++n)
        {
            term *= -x2 / static_cast<T> ((2 * n) * (2 * n + 1));
            sinResult += term;
            if (abs (term) < std::numeric_limits<T>::epsilon() * abs (sinResult))
            {
                break;
            }
        }

        // Compute cos using identity: cos(x) = sin(x + pi/2)
        // Or equivalently: cos^2 + sin^2 = 1, so cos = sqrt(1 - sin^2) with sign correction
        // For efficiency, use the phase-shifted approach
        T cosX = xReduced + halfPi<T>;
        if (cosX > pi<T>)
        {
            cosX -= twoPi<T>;
        }
        if (cosX > halfPi<T>)
        {
            cosX = pi<T> - cosX;
        }
        else if (cosX < -halfPi<T>)
        {
            cosX = -pi<T> - cosX;
        }

        T const cx2 = cosX * cosX;
        T cosResult = cosX;
        T cterm = cosX;

        for (int n = 1; n < maxTerms; ++n)
        {
            cterm *= -cx2 / static_cast<T> ((2 * n) * (2 * n + 1));
            cosResult += cterm;
            if (abs (cterm) < std::numeric_limits<T>::epsilon() * abs (cosResult))
            {
                break;
            }
        }

        return { sinResult, cosResult };
    }

    /**
 * @brief Efficient computation of 10^(x/40) for dB to linear conversion.
 *
 * Used for shelf and peak filter gain calculations. Optimized by using
 * exp(x * ln10/40) instead of pow(10, x/40), avoiding the more expensive
 * pow() implementation.
 *
 * @tparam T Floating-point type
 * @param x Value in dB (typically -24 to +24)
 * @return 10^(x/40) = sqrt(sqrt(10^(x/10))) = sqrt(linear amplitude ratio)
 *
 * @note Accuracy target: < 1e-6 relative error.
 */
    template <typename T>
        requires std::is_floating_point_v<T>
    constexpr auto exp10_40 (T x) noexcept -> T
    {
        return exp (x * ln10_over_40<T>);
    }

} // namespace PlayfulTones::DspToolbox::Math
