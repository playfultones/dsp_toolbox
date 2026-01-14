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
     * @brief Sample rate in samples per second.
     *
     * Structural type (public member) to support use as NTTP.
     *
     * @tparam T Numeric type (default: double for precision in calculations)
     */
    template <Numeric T = double>
    struct SampleRate
    {
        T value { T (48000) };

        constexpr SampleRate() = default;
        explicit constexpr SampleRate (T sps) noexcept : value (sps) {}

        // Comparison
        constexpr bool operator== (const SampleRate& other) const noexcept = default;
        constexpr auto operator<=> (const SampleRate& other) const noexcept = default;
    };

} // namespace PlayfulTones::DspToolbox
