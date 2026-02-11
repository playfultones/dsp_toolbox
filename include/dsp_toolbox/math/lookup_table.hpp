/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include <array>
#include <cstddef>

namespace PlayfulTones::DspToolbox::Math
{

    /**
     * @brief Uniformly-spaced lookup table with cubic Hermite (Catmull-Rom) interpolation.
     *
     * Maps a continuous input range [minInput, maxInput] to N table entries,
     * interpolating smoothly between them using Catmull-Rom splines.
     *
     * ## Interpolation Method
     * Catmull-Rom cubic Hermite interpolation provides C1 continuity (smooth
     * first derivative) with tangents estimated from neighboring points.
     * Boundary tangents use forward/backward differences.
     *
     * ## Usage
     * @code
     * constexpr LookupTable<3> table {
     *     .values = { 0.0f, 5.0f, 10.0f },
     *     .minInput = 0.0f,
     *     .maxInput = 1.0f
     * };
     * static_assert(table.interpolate(0.0f) == 0.0f);
     * static_assert(table.interpolate(1.0f) == 10.0f);
     * float mid = table.interpolate(0.5f); // Smooth interpolation ~5.0
     * @endcode
     *
     * @tparam N Number of table entries (must be >= 2)
     */
    template <std::size_t N>
    struct LookupTable
    {
        static_assert (N >= 2, "LookupTable requires at least 2 entries");

        /** @brief Table values, uniformly spaced across [minInput, maxInput]. */
        std::array<float, N> values {};

        /** @brief Lower bound of input range. */
        float minInput { 0.0f };

        /** @brief Upper bound of input range. */
        float maxInput { 1.0f };

        /**
         * @brief Look up a value with cubic Hermite (Catmull-Rom) interpolation.
         *
         * Inputs outside [minInput, maxInput] are clamped to the boundary values.
         *
         * @param input Continuous input value
         * @return Interpolated output value
         */
        [[nodiscard]] constexpr float interpolate (float input) const noexcept
        {
            if (input <= minInput)
                return values[0];
            if (input >= maxInput)
                return values[N - 1];

            float const range = maxInput - minInput;
            float const normalized = (input - minInput) / range;
            float const scaledPos = normalized * static_cast<float> (N - 1);

            auto idx = static_cast<std::size_t> (scaledPos);
            if (idx >= N - 1)
                idx = N - 2;

            float const t = scaledPos - static_cast<float> (idx);

            float const y0 = values[idx];
            float const y1 = values[idx + 1];

            float m0;
            float m1;

            if (idx == 0)
            {
                m0 = y1 - y0;
            }
            else
            {
                m0 = (values[idx + 1] - values[idx - 1]) * 0.5f;
            }

            if (idx == N - 2)
            {
                m1 = y1 - y0;
            }
            else
            {
                m1 = (values[idx + 2] - values[idx]) * 0.5f;
            }

            float const t2 = t * t;
            float const t3 = t2 * t;
            float const h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
            float const h10 = t3 - 2.0f * t2 + t;
            float const h01 = -2.0f * t3 + 3.0f * t2;
            float const h11 = t3 - t2;

            return h00 * y0 + h10 * m0 + h01 * y1 + h11 * m1;
        }

        /**
         * @brief Look up a value with linear interpolation.
         *
         * Faster than cubic Hermite but with C0 continuity only (discontinuous
         * first derivative at table entries).
         *
         * @param input Continuous input value
         * @return Linearly interpolated output value
         */
        [[nodiscard]] constexpr float interpolateLinear (float input) const noexcept
        {
            if (input <= minInput)
                return values[0];
            if (input >= maxInput)
                return values[N - 1];

            float const range = maxInput - minInput;
            float const normalized = (input - minInput) / range;
            float const scaledPos = normalized * static_cast<float> (N - 1);

            auto idx = static_cast<std::size_t> (scaledPos);
            if (idx >= N - 1)
                idx = N - 2;

            float const t = scaledPos - static_cast<float> (idx);

            return values[idx] + t * (values[idx + 1] - values[idx]);
        }

        /**
         * @brief Look up the nearest table value without interpolation.
         *
         * @param input Continuous input value
         * @return Nearest table value
         */
        [[nodiscard]] constexpr float nearest (float input) const noexcept
        {
            if (input <= minInput)
                return values[0];
            if (input >= maxInput)
                return values[N - 1];

            float const range = maxInput - minInput;
            float const normalized = (input - minInput) / range;
            float const scaledPos = normalized * static_cast<float> (N - 1);

            auto idx = static_cast<std::size_t> (scaledPos + 0.5f);
            if (idx >= N)
                idx = N - 1;

            return values[idx];
        }
    };

} // namespace PlayfulTones::DspToolbox::Math
