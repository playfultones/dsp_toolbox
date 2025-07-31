/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief A no-op function to mark unused parameters.
     *
     * This function can be used to avoid compiler warnings about unused parameters.
     * It does nothing and is intended for use in template code where some parameters
     * may not be used in every instantiation.
     *
     * @param args The unused parameters.
     */
    template <typename... Args>
    constexpr void markUsed (Args&&...) noexcept
    {
    }
} // namespace PlayfulTones::DspToolBox
