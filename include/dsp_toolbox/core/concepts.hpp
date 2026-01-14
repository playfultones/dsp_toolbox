/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include <compare>
#include <concepts>

namespace PlayfulTones::DspToolbox
{

    /**
 * @brief Concept for numeric types suitable for DSP values.
 *
 * Accepts both integral and floating-point types.
 */
    template <typename T>
    concept Numeric = std::integral<T> || std::floating_point<T>;

} // namespace PlayfulTones::DspToolbox
