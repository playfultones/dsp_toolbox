/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <memory>
#include <type_traits>

namespace PlayfulTones::DspToolBox
{
#ifdef _MSC_VER
    #pragma warning(push)
    #pragma warning(disable : 4324) // Suppress structure padding warning
#endif
    /**
     * @brief A lock-free ring buffer (FIFO queue) for audio processing
     * 
     * This implementation is designed for single-producer, single-consumer scenarios.
     * It's lock-free and allocation-free with compile-time fixed capacity.
     * All critical audio path methods are marked noexcept for real-time safety.
     * 
     * @tparam T The type of elements stored in the ring buffer
     * @tparam Capacity The compile-time fixed capacity of the ring buffer
     */
    template <typename T, size_t Capacity>
    class alignas (64) RingBuffer // Cache-line aligned class
    {
    public:
        static_assert (Capacity > 0, "RingBuffer capacity must be greater than 0");

        // Verify that our atomic operations are lock-free
        static_assert (std::atomic<size_t>::is_always_lock_free,
            "std::atomic<size_t> must be lock-free for this implementation");
        /**
         * @brief Construct a new Ring Buffer with compile-time fixed capacity
         * 
         * No dynamic allocation occurs during construction.
         */
        constexpr RingBuffer() noexcept
            : data_ {}, readIndex_ (0), writeIndex_ (0)
        {
        }

        /**
         * @brief Clear the ring buffer, removing all elements
         * 
         * Marked noexcept for real-time safety requirements.
         */
        void clear() noexcept
        {
            readIndex_.store (0, std::memory_order_relaxed);
            writeIndex_.store (0, std::memory_order_relaxed);
        }

        /**
         * @brief Get the current number of elements in the ring buffer
         * 
         * @return size_t The number of elements
         */
        [[nodiscard]] constexpr size_t getSize() const noexcept
        {
            const auto read = readIndex_.load (std::memory_order_relaxed);
            const auto write = writeIndex_.load (std::memory_order_relaxed);
            return write - read;
        }

        /**
         * @brief Get the capacity of the ring buffer
         * 
         * @return size_t The capacity
         */
        [[nodiscard]] static constexpr size_t getCapacity() noexcept
        {
            return Capacity;
        }

        /**
         * @brief Check if the ring buffer is empty
         * 
         * @return true if empty, false otherwise
         */
        [[nodiscard]] constexpr bool isEmpty() const noexcept
        {
            return getSize() == 0;
        }

        /**
         * @brief Check if the ring buffer is full
         * 
         * @return true if full, false otherwise
         */
        [[nodiscard]] constexpr bool isFull() const noexcept
        {
            return getSize() == Capacity;
        }

        /**
         * @brief Push a new element into the ring buffer
         * 
         * This method is guaranteed to be allocation-free and lock-free.
         * Marked noexcept for real-time safety requirements.
         * 
         * @param value The value to push
         * @return true if push was successful, false if buffer is full
         */
        [[nodiscard]] bool push (const T& value) noexcept
        {
            const auto currentWrite = writeIndex_.load (std::memory_order_relaxed);
            const auto currentRead = readIndex_.load (std::memory_order_acquire);

            // Check if there is space - branch prediction friendly
            if (currentWrite - currentRead >= Capacity) [[unlikely]]
                return false;

            // Write the element
            const auto writePos = currentWrite % Capacity;
            data_[writePos] = value;

            // Update write index with release semantics to ensure visibility
            writeIndex_.store (currentWrite + 1, std::memory_order_release);
            return true;
        }

        /**
         * @brief Push a new element using move semantics
         * 
         * This method is guaranteed to be allocation-free and lock-free.
         * Marked noexcept for real-time safety requirements.
         * 
         * @param value The value to push (will be moved from)
         * @return true if push was successful, false if buffer is full
         */
        [[nodiscard]] bool push (T&& value) noexcept
        {
            const auto currentWrite = writeIndex_.load (std::memory_order_relaxed);
            const auto currentRead = readIndex_.load (std::memory_order_acquire);

            // Check if there is space - branch prediction friendly
            if (currentWrite - currentRead >= Capacity) [[unlikely]]
                return false;

            // Write the element
            const auto writePos = currentWrite % Capacity;
            data_[writePos] = std::move (value);

            // Update write index with release semantics to ensure visibility
            writeIndex_.store (currentWrite + 1, std::memory_order_release);
            return true;
        }

        /**
         * @brief Pop an element from the ring buffer
         * 
         * This method is guaranteed to be allocation-free and lock-free.
         * Marked noexcept for real-time safety requirements.
         * 
         * @param[out] value Reference to store the popped value
         * @return true if pop was successful, false if buffer is empty
         */
        [[nodiscard]] bool pop (T& value) noexcept
        {
            const auto currentRead = readIndex_.load (std::memory_order_relaxed);
            const auto currentWrite = writeIndex_.load (std::memory_order_acquire);

            // Check if there are elements to read - branch prediction friendly
            if (currentRead == currentWrite) [[unlikely]]
                return false;

            // Read the element
            const auto readPos = currentRead % Capacity;
            value = std::move (data_[readPos]);

            // Update read index with release semantics to ensure visibility
            readIndex_.store (currentRead + 1, std::memory_order_release);
            return true;
        }

        /**
         * @brief Peek at the next element without removing it
         * 
         * This method is guaranteed to be allocation-free and lock-free.
         * Marked noexcept for real-time safety requirements.
         * 
         * @param[out] value Reference to store the peeked value
         * @return true if peek was successful, false if buffer is empty
         */
        [[nodiscard]] bool peek (T& value) const noexcept
        {
            const auto currentRead = readIndex_.load (std::memory_order_relaxed);
            const auto currentWrite = writeIndex_.load (std::memory_order_acquire);

            // Check if there are elements to read - branch prediction friendly
            if (currentRead == currentWrite) [[unlikely]]
                return false;

            // Read the element without removing it
            const auto readPos = currentRead % Capacity;
            value = data_[readPos];
            return true;
        }

        /**
         * @brief Peek at an element at a specific offset from the read position
         * 
         * This method is guaranteed to be allocation-free and lock-free.
         * Marked noexcept for real-time safety requirements.
         * 
         * @param[out] value Reference to store the peeked value
         * @param offset The offset from the current read position
         * @return true if peek was successful, false if the position is invalid
         */
        [[nodiscard]] bool peekAt (T& value, size_t offset) const noexcept
        {
            const auto currentRead = readIndex_.load (std::memory_order_relaxed);
            const auto currentWrite = writeIndex_.load (std::memory_order_acquire);

            // Check if the position is within valid range - branch prediction friendly
            if (offset >= (currentWrite - currentRead)) [[unlikely]]
                return false;

            // Calculate the actual position to read from
            const auto readPos = (currentRead + offset) % Capacity;
            value = data_[readPos];
            return true;
        }

        /**
         * @brief Discard a number of elements from the front of the buffer
         * 
         * Marked noexcept for real-time safety requirements.
         * 
         * @param count Number of elements to discard
         * @return size_t The actual number of elements discarded
         */
        [[nodiscard]] size_t discard (size_t count) noexcept
        {
            const auto currentRead = readIndex_.load (std::memory_order_relaxed);
            const auto currentWrite = writeIndex_.load (std::memory_order_acquire);

            const auto available = currentWrite - currentRead;
            const auto toDiscard = std::min (count, available);

            if (toDiscard > 0) [[likely]]
                readIndex_.store (currentRead + toDiscard, std::memory_order_release);

            return toDiscard;
        }

        /**
         * @brief Write multiple elements to the buffer
         * 
         * Marked noexcept for real-time safety requirements.
         * 
         * @param data Pointer to the data to write
         * @param count Number of elements to write
         * @return size_t The number of elements actually written
         */
        [[nodiscard]] size_t writeMany (const T* data, size_t count) noexcept
        {
            const auto currentWrite = writeIndex_.load (std::memory_order_relaxed);
            const auto currentRead = readIndex_.load (std::memory_order_acquire);

            // Calculate available space
            const auto available = Capacity - (currentWrite - currentRead);
            const auto toWrite = std::min (count, available);

            for (size_t i = 0; i < toWrite; ++i)
            {
                const auto pos = (currentWrite + i) % Capacity;
                data_[pos] = data[i];
            }

            if (toWrite > 0) [[likely]]
                writeIndex_.store (currentWrite + toWrite, std::memory_order_release);

            return toWrite;
        }

        /**
         * @brief Read multiple elements from the buffer
         * 
         * Marked noexcept for real-time safety requirements.
         * 
         * @param data Pointer to the destination buffer
         * @param count Maximum number of elements to read
         * @return size_t The number of elements actually read
         */
        [[nodiscard]] size_t readMany (T* data, size_t count) noexcept
        {
            const auto currentRead = readIndex_.load (std::memory_order_relaxed);
            const auto currentWrite = writeIndex_.load (std::memory_order_acquire);

            // Calculate available elements
            const auto available = currentWrite - currentRead;
            const auto toRead = std::min (count, available);

            for (size_t i = 0; i < toRead; ++i)
            {
                const auto pos = (currentRead + i) % Capacity;
                data[i] = std::move (data_[pos]);
            }

            if (toRead > 0) [[likely]]
                readIndex_.store (currentRead + toRead, std::memory_order_release);

            return toRead;
        }

    private:
        std::array<T, Capacity> data_; // Fixed-size storage for ring buffer elements

        // Cache-line aligned atomic indices to prevent false sharing
        alignas (64) std::atomic<size_t> readIndex_; // Current read position
        alignas (64) std::atomic<size_t> writeIndex_; // Current write position
    };

#ifdef _MSC_VER
    #pragma warning(push)
    #pragma warning(disable : 4324) // Suppress structure padding warning
#endif
}
