/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once

#include "midi_sequence.h"
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <vector>

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief Utility class for writing MIDI files.
     * 
     * This class provides functionality to write MidiSequence objects
     * to Standard MIDI Files (SMF) format 0 (single track).
     */
    class MidiFile
    {
    private:
        // This is a static-only class, prevent instantiation
        MidiFile() = delete;
        ~MidiFile() = delete;
        MidiFile (const MidiFile&) = delete;
        MidiFile& operator= (const MidiFile&) = delete;
        MidiFile (MidiFile&&) = delete;
        MidiFile& operator= (MidiFile&&) = delete;

    public:
        /**
         * @brief Writes a MIDI sequence to a file in SMF format 0.
         * 
         * @param sequence The MIDI sequence to write
         * @param filePath The path where the MIDI file should be written
         * @param ticksPerQuarterNote The number of ticks per quarter note (default: 960)
         * @throws std::runtime_error if file cannot be opened or written
         */
        static void writeToFile (const MidiSequence& sequence, const std::string& filePath, uint16_t ticksPerQuarterNote = 960)
        {
            std::ofstream file (filePath, std::ios::binary);
            if (!file)
            {
                throw std::runtime_error ("Could not open file for writing: " + filePath);
            }

            // Write MIDI header chunk
            writeString (file, "MThd"); // Header chunk type
            writeUint32 (file, 6); // Header length (always 6)
            writeUint16 (file, 0); // Format type (0 = single track)
            writeUint16 (file, 1); // Number of tracks (1 for format 0)
            writeUint16 (file, ticksPerQuarterNote); // Time division

            // Start track chunk
            writeString (file, "MTrk");

            // We'll need to come back and write the track length
            auto trackLengthPos = file.tellp();
            writeUint32 (file, 0); // Placeholder for track length

            uint64_t runningTime = 0;
            const auto& messages = sequence.getAllMessages();

            // Write track events
            for (const auto& timedMsg : messages)
            {
                // Write delta time (time since last event)
                uint64_t delta = timedMsg.timestamp - runningTime;
                writeVariableLength (file, delta);
                runningTime = timedMsg.timestamp;

                // Write MIDI message
                file.put (timedMsg.message.getStatus());
                file.put (timedMsg.message.getData1());
                if (!isOneByteMessage (timedMsg.message.getStatus()))
                {
                    file.put (timedMsg.message.getData2());
                }
            }

            // Write end of track meta event
            writeVariableLength (file, 0); // Delta time
            file.put (0xFF); // Meta event
            file.put (0x2F); // End of track
            file.put (0x00); // Length

            // Go back and write the track length
            auto endPos = file.tellp();
            auto trackLength = static_cast<uint32_t> (endPos - trackLengthPos - 4);
            file.seekp (trackLengthPos);
            writeUint32 (file, trackLength);

            if (!file)
            {
                throw std::runtime_error ("Error writing to MIDI file: " + filePath);
            }
        }

    private:
        static void writeString (std::ofstream& file, const std::string& str)
        {
            file.write (str.c_str(), str.length());
        }

        static void writeUint16 (std::ofstream& file, uint16_t value)
        {
            file.put ((value >> 8) & 0xFF);
            file.put (value & 0xFF);
        }

        static void writeUint32 (std::ofstream& file, uint32_t value)
        {
            file.put ((value >> 24) & 0xFF);
            file.put ((value >> 16) & 0xFF);
            file.put ((value >> 8) & 0xFF);
            file.put (value & 0xFF);
        }

        static void writeVariableLength (std::ofstream& file, uint64_t value)
        {
            std::vector<uint8_t> bytes;
            bytes.push_back (value & 0x7F);

            while (value >>= 7)
            {
                bytes.push_back ((value & 0x7F) | 0x80);
            }

            for (auto it = bytes.rbegin(); it != bytes.rend(); ++it)
            {
                file.put (*it);
            }
        }

        static bool isOneByteMessage (uint8_t status)
        {
            // Program Change and Channel Pressure messages only have one data byte
            return ((status & 0xF0) == 0xC0) || ((status & 0xF0) == 0xD0);
        }
    };

} // namespace PlayfulTones::DspToolBox
