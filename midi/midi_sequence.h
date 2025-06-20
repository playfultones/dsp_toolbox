/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once

#include "midi_message.h"
#include <algorithm>
#include <cstdint>
#include <vector>

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief A class for storing and managing a sequence of MIDI messages with timestamps.
     * 
     * This class provides functionality to store, sort, and access MIDI messages
     * in chronological order. Each message is associated with a timestamp that
     * represents its position in the sequence.
     */
    class MidiSequence
    {
    public:
        /**
         * @brief Represents a timed MIDI message in the sequence.
         */
        struct TimedMessage
        {
            uint64_t timestamp; // Timestamp in ticks/samples
            MidiMessage message;

            constexpr TimedMessage (uint64_t time, const MidiMessage& msg) noexcept
                : timestamp (time), message (msg)
            {
            }

            // Allow sorting by timestamp
            constexpr bool operator< (const TimedMessage& other) const noexcept
            {
                return timestamp < other.timestamp;
            }
        };

        /**
         * @brief Adds a MIDI message to the sequence at the specified timestamp.
         * @param timestamp The time position of the message in ticks/samples
         * @param message The MIDI message to add
         */
        void addMessage (uint64_t timestamp, const MidiMessage& message)
        {
            m_messages.emplace_back (timestamp, message);
            // Keep the sequence sorted by timestamp
            std::sort (m_messages.begin(), m_messages.end());
        }

        /**
         * @brief Removes all messages from the sequence.
         */
        void clear() noexcept
        {
            m_messages.clear();
        }

        /**
         * @brief Returns the number of messages in the sequence.
         * @return The number of messages
         */
        [[nodiscard]] size_t size() const noexcept
        {
            return m_messages.size();
        }

        /**
         * @brief Checks if the sequence is empty.
         * @return true if the sequence contains no messages
         */
        [[nodiscard]] bool isEmpty() const noexcept
        {
            return m_messages.empty();
        }

        /**
         * @brief Gets all messages that occur at or before the specified timestamp.
         * @param timestamp The time position to check
         * @return A vector of messages that occur at or before the timestamp
         */
        [[nodiscard]] std::vector<MidiMessage> getMessagesUpTo (uint64_t timestamp) const
        {
            std::vector<MidiMessage> result;
            for (const auto& timedMsg : m_messages)
            {
                if (timedMsg.timestamp > timestamp)
                    break;

                result.push_back (timedMsg.message);
            }
            return result;
        }

        /**
         * @brief Gets the messages within a specific time range.
         * @param startTime The start of the time range (inclusive)
         * @param endTime The end of the time range (exclusive)
         * @return A vector of messages that occur within the specified range
         */
        [[nodiscard]] std::vector<MidiMessage> getMessagesBetween (uint64_t startTime, uint64_t endTime) const
        {
            std::vector<MidiMessage> result;
            for (const auto& timedMsg : m_messages)
            {
                if (timedMsg.timestamp >= endTime)
                    break;

                if (timedMsg.timestamp >= startTime)
                    result.push_back (timedMsg.message);
            }
            return result;
        }

        /**
         * @brief Gets the next message after the specified timestamp.
         * @param timestamp The time position to check from
         * @return A pointer to the next message, or nullptr if no message exists
         */
        [[nodiscard]] const TimedMessage* getNextMessage (uint64_t timestamp) const noexcept
        {
            auto it = std::find_if (m_messages.begin(), m_messages.end(), [timestamp] (const TimedMessage& msg) { return msg.timestamp > timestamp; });

            return it != m_messages.end() ? &(*it) : nullptr;
        }

        /**
         * @brief Gets all messages in the sequence.
         * @return A const reference to the vector of timed messages
         */
        [[nodiscard]] const std::vector<TimedMessage>& getAllMessages() const noexcept
        {
            return m_messages;
        }

    private:
        std::vector<TimedMessage> m_messages;
    };

} // namespace PlayfulTones::DspToolBox
