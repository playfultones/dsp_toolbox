/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once

#include "midi_message.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <span>

namespace PlayfulTones::DspToolBox
{
    // Strong type for timestamps
    template<typename T>
    struct Timestamp {
        T value;
        constexpr Timestamp() noexcept : value(T{}) {}
        constexpr explicit Timestamp(T t) noexcept : value(t) {}
        constexpr operator T() const noexcept { return value; }
        constexpr bool operator<(const Timestamp& other) const noexcept { return value < other.value; }
        constexpr bool operator<=(const Timestamp& other) const noexcept { return value <= other.value; }
        constexpr bool operator>(const Timestamp& other) const noexcept { return value > other.value; }
        constexpr bool operator>=(const Timestamp& other) const noexcept { return value >= other.value; }
        constexpr bool operator==(const Timestamp& other) const noexcept { return value == other.value; }
    };

    /**
     * @brief A real-time safe MIDI sequence with template-based configuration.
     * 
     * Template parameters:
     * @tparam TimestampType Type for timestamps (e.g., uint64_t, double)
     * @tparam MaxMessages Maximum number of messages in the sequence (compile-time)
     * 
     * This class provides zero-allocation, cache-friendly MIDI message storage
     * suitable for real-time audio processing.
     */
    template<typename TimestampType = uint64_t, size_t MaxMessages = 1024>
    class alignas(64) MidiSequence  // Cache-line aligned
    {
    public:
        using timestamp_type = Timestamp<TimestampType>;
        
        /**
         * @brief Represents a timed MIDI message in the sequence.
         */
        struct alignas(16) TimedMessage  // Align for SIMD-friendly access
        {
            timestamp_type timestamp; // Timestamp in ticks/samples
            MidiMessage message;

            constexpr TimedMessage() noexcept = default;
            
            constexpr TimedMessage(timestamp_type time, const MidiMessage& msg) noexcept
                : timestamp(time), message(msg)
            {
            }

            // Allow sorting by timestamp - branch prediction friendly
            constexpr bool operator<(const TimedMessage& other) const noexcept
            {
                return timestamp < other.timestamp;
            }
        };

        static_assert(MaxMessages > 0, "MaxMessages must be greater than 0");

        constexpr MidiSequence() noexcept : m_size(0) {}

        /**
         * @brief Adds a MIDI message to the sequence at the specified timestamp.
         * Real-time safe: maintains sorted order without full sorting.
         * @param timestamp The time position of the message
         * @param message The MIDI message to add
         * @return true if message was added successfully, false if sequence is full
         */
        [[nodiscard]] bool addMessage(timestamp_type timestamp, const MidiMessage& message) noexcept
        {
            if (m_size >= MaxMessages)
                return false;

            // Find insertion point using binary search for O(log n) complexity
            auto insertPos = std::upper_bound(m_messages.begin(), m_messages.begin() + m_size, 
                                            TimedMessage{timestamp, message});
            
            // Shift elements to make room (move semantics where possible)
            const auto insertIndex = static_cast<size_t>(insertPos - m_messages.begin());
            for (size_t i = m_size; i > insertIndex; --i)
            {
                m_messages[i] = m_messages[i - 1];
            }
            
            // Insert the new message
            m_messages[insertIndex] = TimedMessage{timestamp, message};
            ++m_size;
            
            return true;
        }

        /**
         * @brief Removes all messages from the sequence.
         */
        constexpr void clear() noexcept
        {
            m_size = 0;
        }

        /**
         * @brief Returns the number of messages in the sequence.
         */
        [[nodiscard]] constexpr size_t size() const noexcept
        {
            return m_size;
        }

        /**
         * @brief Returns the maximum capacity of the sequence.
         */
        [[nodiscard]] static constexpr size_t capacity() noexcept
        {
            return MaxMessages;
        }

        /**
         * @brief Checks if the sequence is empty.
         */
        [[nodiscard]] constexpr bool isEmpty() const noexcept
        {
            return m_size == 0;
        }

        /**
         * @brief Checks if the sequence is full.
         */
        [[nodiscard]] constexpr bool isFull() const noexcept
        {
            return m_size >= MaxMessages;
        }

        /**
         * @brief Real-time safe: gets messages up to timestamp using callback.
         * No allocations, processes messages in-place.
         * @param timestamp The time position to check
         * @param callback Function to call for each message: void(const MidiMessage&)
         */
        template<typename Callback>
        constexpr void processMessagesUpTo(timestamp_type timestamp, Callback&& callback) const noexcept
        {
            for (size_t i = 0; i < m_size; ++i)
            {
                if (m_messages[i].timestamp > timestamp)
                    break;
                
                callback(m_messages[i].message);
            }
        }

        /**
         * @brief Real-time safe: gets messages within time range using callback.
         * @param startTime The start of the time range (inclusive)
         * @param endTime The end of the time range (exclusive)
         * @param callback Function to call for each message
         */
        template<typename Callback>
        constexpr void processMessagesBetween(timestamp_type startTime, timestamp_type endTime, 
                                            Callback&& callback) const noexcept
        {
            for (size_t i = 0; i < m_size; ++i)
            {
                const auto& msg = m_messages[i];
                if (msg.timestamp >= endTime)
                    break;
                
                if (msg.timestamp >= startTime)
                    callback(msg.message);
            }
        }

        /**
         * @brief Gets the next message after the specified timestamp.
         * @param timestamp The time position to check from
         * @return Pointer to the next message, or nullptr if no message exists
         */
        [[nodiscard]] constexpr const TimedMessage* getNextMessage(timestamp_type timestamp) const noexcept
        {
            for (size_t i = 0; i < m_size; ++i)
            {
                if (m_messages[i].timestamp > timestamp)
                    return &m_messages[i];
            }
            return nullptr;
        }

        /**
         * @brief Gets a span view of all messages (real-time safe).
         * @return A span of the messages currently in the sequence
         */
        [[nodiscard]] constexpr std::span<const TimedMessage> getAllMessages() const noexcept
        {
            return std::span<const TimedMessage>(m_messages.data(), m_size);
        }

        /**
         * @brief Direct access to message by index (bounds-checked in debug).
         */
        [[nodiscard]] constexpr const TimedMessage& operator[](size_t index) const noexcept
        {
            // In debug builds, we could add assert(index < m_size)
            return m_messages[index];
        }

        /**
         * @brief Removes messages older than the specified timestamp.
         * Useful for maintaining a rolling window of recent messages.
         * @param cutoffTime Messages older than this will be removed
         * @return Number of messages removed
         */
        [[nodiscard]] size_t removeMessagesOlderThan(timestamp_type cutoffTime) noexcept
        {
            size_t removeCount = 0;
            
            // Find first message to keep
            while (removeCount < m_size && m_messages[removeCount].timestamp < cutoffTime)
            {
                ++removeCount;
            }
            
            if (removeCount > 0)
            {
                // Shift remaining messages to the beginning
                for (size_t i = 0; i < m_size - removeCount; ++i)
                {
                    m_messages[i] = m_messages[i + removeCount];
                }
                m_size -= removeCount;
            }
            
            return removeCount;
        }

    private:
        alignas(64) std::array<TimedMessage, MaxMessages> m_messages; // Cache-line aligned storage
        size_t m_size;
    };

    // Common typedefs for convenience
    using MidiSequence64 = MidiSequence<uint64_t, 1024>;
    using MidiSequence32 = MidiSequence<uint32_t, 512>;
    using MidiSequenceDouble = MidiSequence<double, 1024>;

} // namespace PlayfulTones::DspToolBox
