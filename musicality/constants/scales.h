/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/
#pragma once

#include <array>
#include <vector>

namespace PlayfulTones::DspToolBox::Scales
{
    /**
     * @brief Defines common musical scales as semitone intervals from the root note
     * 
     * Each value represents the number of semitones above the root note:
     * - 0 = root note (e.g., C in C major)
     * - 1 = minor second
     * - 2 = major second
     * - 3 = minor third
     * - 4 = major third
     * - 5 = perfect fourth
     * - 6 = diminished fifth/tritone
     * - 7 = perfect fifth
     * - 8 = minor sixth
     * - 9 = major sixth
     * - 10 = minor seventh
     * - 11 = major seventh
     */
    namespace Intervals
    {
        // Seven-note scales (heptatonic)
        inline constexpr std::array<uint8_t, 7> Major = { 0, 2, 4, 5, 7, 9, 11 };
        inline constexpr std::array<uint8_t, 7> NaturalMinor = { 0, 2, 3, 5, 7, 8, 10 };
        inline constexpr std::array<uint8_t, 7> HarmonicMinor = { 0, 2, 3, 5, 7, 8, 11 };
        inline constexpr std::array<uint8_t, 7> MelodicMinor = { 0, 2, 3, 5, 7, 9, 11 };
        inline constexpr std::array<uint8_t, 7> Dorian = { 0, 2, 3, 5, 7, 9, 10 };
        inline constexpr std::array<uint8_t, 7> Phrygian = { 0, 1, 3, 5, 7, 8, 10 };
        inline constexpr std::array<uint8_t, 7> Lydian = { 0, 2, 4, 6, 7, 9, 11 };
        inline constexpr std::array<uint8_t, 7> Mixolydian = { 0, 2, 4, 5, 7, 9, 10 };
        inline constexpr std::array<uint8_t, 7> Locrian = { 0, 1, 3, 5, 6, 8, 10 };

        // Five-note scales (pentatonic)
        inline constexpr std::array<uint8_t, 5> MajorPentatonic = { 0, 2, 4, 7, 9 };
        inline constexpr std::array<uint8_t, 5> MinorPentatonic = { 0, 3, 5, 7, 10 };

        // Six-note scales (hexatonic)
        inline constexpr std::array<uint8_t, 6> Blues = { 0, 3, 5, 6, 7, 10 };
        inline constexpr std::array<uint8_t, 6> WholeTone = { 0, 2, 4, 6, 8, 10 };

        // Eight-note scales (octatonic)
        inline constexpr std::array<uint8_t, 8> Diminished = { 0, 2, 3, 5, 6, 8, 9, 11 };
    }

    /**
     * @brief Utility functions for working with scales
     */
    namespace Utils
    {
        /**
         * @brief Converts a scale interval array to a vector for use with melody generators
         * @tparam Size The size of the input array
         * @param scale The scale intervals array to convert
         * @return std::vector<uint8_t> The scale intervals as a vector
         */
        template <size_t Size>
        [[nodiscard]] inline std::vector<uint8_t> toVector (const std::array<uint8_t, Size>& scale)
        {
            return std::vector<uint8_t> (scale.begin(), scale.end());
        }
    }
}
