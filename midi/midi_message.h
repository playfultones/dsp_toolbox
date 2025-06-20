/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/
#pragma once

#include <cstdint>

namespace PlayfulTones::DspToolBox
{

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
     * Represents a MIDI message.
     */
    class MidiMessage
    {
    public:
        // Constructor for channel voice messages
        constexpr MidiMessage (MidiStatus status, uint8_t channel, uint8_t data1, uint8_t data2 = 0) noexcept
            : m_status (static_cast<uint8_t> (status) | (channel & 0x0F)), m_data1 (data1), m_data2 (data2)
        {
        }

        // Constructor for system messages
        constexpr explicit MidiMessage (uint8_t status, uint8_t data1 = 0, uint8_t data2 = 0) noexcept
            : m_status (status), m_data1 (data1), m_data2 (data2)
        {
        }

        // Getters
        [[nodiscard]] constexpr uint8_t getStatus() const noexcept { return m_status; }
        [[nodiscard]] constexpr uint8_t getChannel() const noexcept { return m_status & 0x0F; }
        [[nodiscard]] constexpr uint8_t getData1() const noexcept { return m_data1; }
        [[nodiscard]] constexpr uint8_t getData2() const noexcept { return m_data2; }

        // Utility methods
        [[nodiscard]] constexpr bool isNoteOn() const noexcept
        {
            return (m_status & 0xF0) == static_cast<uint8_t> (MidiStatus::NoteOn) && m_data2 > 0;
        }

        [[nodiscard]] constexpr bool isNoteOff() const noexcept
        {
            return (m_status & 0xF0) == static_cast<uint8_t> (MidiStatus::NoteOff) || ((m_status & 0xF0) == static_cast<uint8_t> (MidiStatus::NoteOn) && m_data2 == 0);
        }

        [[nodiscard]] constexpr bool isControlChange() const noexcept
        {
            return (m_status & 0xF0) == static_cast<uint8_t> (MidiStatus::ControlChange);
        }

        [[nodiscard]] constexpr bool isPitchBend() const noexcept
        {
            return (m_status & 0xF0) == static_cast<uint8_t> (MidiStatus::PitchBend);
        }

    private:
        uint8_t m_status; // Status byte (includes channel for channel messages)
        uint8_t m_data1; // First data byte
        uint8_t m_data2; // Second data byte (not used in all message types)
    };

} // namespace dsp
