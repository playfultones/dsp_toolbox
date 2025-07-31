/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/
#pragma once

#include <cstdint>
#include <span>

namespace PlayfulTones::DspToolBox
{
    // Strong types for type safety
    struct MidiChannel
    {
        uint8_t value;
        constexpr explicit MidiChannel (uint8_t ch) noexcept : value (ch & 0x0F) {}
        constexpr operator uint8_t() const noexcept { return value; }
    };

    struct NoteNumber
    {
        uint8_t value;
        constexpr explicit NoteNumber (uint8_t note) noexcept : value (note & 0x7F) {}
        constexpr operator uint8_t() const noexcept { return value; }
    };

    struct Velocity
    {
        uint8_t value;
        constexpr explicit Velocity (uint8_t vel) noexcept : value (vel & 0x7F) {}
        constexpr operator uint8_t() const noexcept { return value; }
    };

    struct ControllerNumber
    {
        uint8_t value;
        constexpr explicit ControllerNumber (uint8_t cc) noexcept : value (cc & 0x7F) {}
        constexpr operator uint8_t() const noexcept { return value; }
    };

    struct ControllerValue
    {
        uint8_t value;
        constexpr explicit ControllerValue (uint8_t val) noexcept : value (val & 0x7F) {}
        constexpr operator uint8_t() const noexcept { return value; }
    };

    // MIDI Status Byte Types (most common message types)
    enum class MidiStatus : uint8_t {
        NoteOff = 0x80,
        NoteOn = 0x90,
        PolyphonicPressure = 0xA0,
        ControlChange = 0xB0,
        ProgramChange = 0xC0,
        ChannelPressure = 0xD0,
        PitchBend = 0xE0,
        SystemMessage = 0xF0
    };

    /**
     * Represents a MIDI message with strong type safety and real-time optimizations.
     */
    class MidiMessage
    {
    public:
        // Default constructor - creates a "null" MIDI message
        constexpr MidiMessage() noexcept : m_status(0), m_data1(0), m_data2(0) {}

        // Constructor for channel voice messages with strong types
        constexpr MidiMessage (MidiStatus status, MidiChannel channel, uint8_t data1, uint8_t data2 = 0) noexcept
            : m_status (static_cast<uint8_t> (status) | static_cast<uint8_t> (channel)), m_data1 (data1 & 0x7F), m_data2 (data2 & 0x7F)
        {
        }

        // Constructor for note messages
        constexpr MidiMessage (MidiStatus status, MidiChannel channel, NoteNumber note, Velocity velocity) noexcept
            : m_status (static_cast<uint8_t> (status) | static_cast<uint8_t> (channel)), m_data1 (static_cast<uint8_t> (note)), m_data2 (static_cast<uint8_t> (velocity))
        {
        }

        // Constructor for control change messages
        constexpr MidiMessage (MidiChannel channel, ControllerNumber controller, ControllerValue value) noexcept
            : m_status (static_cast<uint8_t> (MidiStatus::ControlChange) | static_cast<uint8_t> (channel)), m_data1 (static_cast<uint8_t> (controller)), m_data2 (static_cast<uint8_t> (value))
        {
        }

        // Constructor for system messages
        constexpr explicit MidiMessage (uint8_t status, uint8_t data1 = 0, uint8_t data2 = 0) noexcept
            : m_status (status), m_data1 (data1 & 0x7F), m_data2 (data2 & 0x7F)
        {
        }

        // Getters
        [[nodiscard]] constexpr uint8_t getStatus() const noexcept { return m_status; }
        [[nodiscard]] constexpr MidiChannel getChannel() const noexcept { return MidiChannel (m_status & 0x0F); }
        [[nodiscard]] constexpr uint8_t getData1() const noexcept { return m_data1; }
        [[nodiscard]] constexpr uint8_t getData2() const noexcept { return m_data2; }

        [[nodiscard]] constexpr NoteNumber getNoteNumber() const noexcept { return NoteNumber (m_data1); }
        [[nodiscard]] constexpr Velocity getVelocity() const noexcept { return Velocity (m_data2); }
        [[nodiscard]] constexpr ControllerNumber getControllerNumber() const noexcept { return ControllerNumber (m_data1); }
        [[nodiscard]] constexpr ControllerValue getControllerValue() const noexcept { return ControllerValue (m_data2); }

        // Utility methods - branch prediction friendly
        [[nodiscard]] constexpr bool isNoteOn() const noexcept
        {
            const uint8_t statusType = m_status & 0xF0;
            return statusType == static_cast<uint8_t> (MidiStatus::NoteOn) && m_data2 > 0;
        }

        [[nodiscard]] constexpr bool isNoteOff() const noexcept
        {
            const uint8_t statusType = m_status & 0xF0;
            return statusType == static_cast<uint8_t> (MidiStatus::NoteOff) || (statusType == static_cast<uint8_t> (MidiStatus::NoteOn) && m_data2 == 0);
        }

        [[nodiscard]] constexpr bool isControlChange() const noexcept
        {
            return (m_status & 0xF0) == static_cast<uint8_t> (MidiStatus::ControlChange);
        }

        [[nodiscard]] constexpr bool isPitchBend() const noexcept
        {
            return (m_status & 0xF0) == static_cast<uint8_t> (MidiStatus::PitchBend);
        }

        [[nodiscard]] constexpr bool isProgramChange() const noexcept
        {
            return (m_status & 0xF0) == static_cast<uint8_t> (MidiStatus::ProgramChange);
        }

        [[nodiscard]] constexpr bool isChannelPressure() const noexcept
        {
            return (m_status & 0xF0) == static_cast<uint8_t> (MidiStatus::ChannelPressure);
        }

        // Equality comparison for testing
        [[nodiscard]] constexpr bool operator== (const MidiMessage& other) const noexcept
        {
            return m_status == other.m_status && m_data1 == other.m_data1 && m_data2 == other.m_data2;
        }

        // Pack into a 32-bit word for efficient storage/transmission
        [[nodiscard]] constexpr uint32_t pack() const noexcept
        {
            return static_cast<uint32_t> (m_status) | (static_cast<uint32_t> (m_data1) << 8) | (static_cast<uint32_t> (m_data2) << 16);
        }

        // Unpack from a 32-bit word
        [[nodiscard]] static constexpr MidiMessage unpack (uint32_t packed) noexcept
        {
            return MidiMessage (
                static_cast<uint8_t> (packed & 0xFF),
                static_cast<uint8_t> ((packed >> 8) & 0xFF),
                static_cast<uint8_t> ((packed >> 16) & 0xFF));
        }

    private:
        uint8_t m_status; // Status byte (includes channel for channel messages)
        uint8_t m_data1; // First data byte
        uint8_t m_data2; // Second data byte (not used in all message types)
    };

    static_assert (sizeof (MidiMessage) <= 4, "MidiMessage should be compact for optimal cache performance");

} // namespace PlayfulTones::DspToolBox
