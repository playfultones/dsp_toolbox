/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include <compare>
#include <cstdint>

namespace PlayfulTones::DspToolbox
{

    /**
 * @brief MIDI note number (0-127).
 *
 * Middle C (C4) = 60, Concert A (A4) = 69.
 * Not templated - MIDI is always uint8_t by spec.
 */
    class MIDINoteNumber
    {
        uint8_t value_ { 60 }; // Middle C default

    public:
        constexpr MIDINoteNumber() = default;
        explicit constexpr MIDINoteNumber (uint8_t note) noexcept : value_ (note) {}

        [[nodiscard]] constexpr uint8_t value() const noexcept { return value_; }

        // Explicit conversion to underlying type
        explicit constexpr operator uint8_t() const noexcept { return value_; }

        // Arithmetic for transposition
        constexpr MIDINoteNumber operator+ (int semitones) const noexcept
        {
            return MIDINoteNumber { static_cast<uint8_t> (value_ + semitones) };
        }

        constexpr MIDINoteNumber operator- (int semitones) const noexcept
        {
            return MIDINoteNumber { static_cast<uint8_t> (value_ - semitones) };
        }

        // Interval between notes
        constexpr int operator- (MIDINoteNumber other) const noexcept
        {
            return static_cast<int> (value_) - static_cast<int> (other.value_);
        }

        // Comparison
        constexpr bool operator== (const MIDINoteNumber& other) const noexcept = default;
        constexpr auto operator<=> (const MIDINoteNumber& other) const noexcept = default;
    };

} // namespace PlayfulTones::DspToolbox
