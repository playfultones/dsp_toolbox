/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once

#include <atomic>
#include <memory>
#include <type_traits>
#include <vector>

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief A lock-free ring buffer (FIFO queue) for audio processing
     * 
     * This implementation is designed for single-producer, single-consumer scenarios.
     * It's lock-free and allocation-free during normal operation (push/pop).
     * 
     * @tparam T The type of elements stored in the ring buffer
     */
    template <typename T>
    class RingBuffer
    {
    public:
        static constexpr size_t DefaultCapacity = 1024; // Default initial capacity
        /**
         * @brief Construct a new Ring Buffer with the given capacity
         * 
         * @param initialCapacity The initial capacity of the ring buffer
         */
        explicit RingBuffer (size_t initialCapacity = DefaultCapacity)
            : data_ (initialCapacity), capacity_ (initialCapacity), readIndex_ (0), writeIndex_ (0)
        {
            // Ensure the capacity is a power of 2 for efficient wrapping
            if (!isPowerOfTwo (initialCapacity))
            {
                capacity_ = nextPowerOfTwo (initialCapacity);
                data_.resize (capacity_);
            }
        }

        /**
         * @brief Resize the ring buffer to a new capacity
         * 
         * This method will allocate new memory. It's not thread-safe and should
         * not be called while push/pop operations are in progress.
         * 
         * @param newCapacity The new capacity for the ring buffer
         * @return true if resize was successful, false otherwise
         */
        bool resize (size_t newCapacity)
        {
            if (newCapacity == 0)
                return false;

            // Make sure the new capacity is a power of 2
            if (!isPowerOfTwo (newCapacity))
                newCapacity = nextPowerOfTwo (newCapacity);

            // Create new storage
            std::vector<T> newData (newCapacity);

            // Copy existing elements if any
            const size_t size = getSize();
            for (size_t i = 0; i < size; ++i)
            {
                const size_t readPos = (readIndex_.load (std::memory_order_relaxed) + i) & (capacity_ - 1);
                const size_t writePos = i & (newCapacity - 1);
                newData[writePos] = std::move (data_[readPos]);
            }

            // Update internal state
            data_ = std::move (newData);
            capacity_ = newCapacity;
            readIndex_.store (0, std::memory_order_relaxed);
            writeIndex_.store (size, std::memory_order_relaxed);

            return true;
        }

        /**
         * @brief Clear the ring buffer, removing all elements
         */
        void clear()
        {
            readIndex_.store (0, std::memory_order_relaxed);
            writeIndex_.store (0, std::memory_order_relaxed);
        }

        /**
         * @brief Get the current number of elements in the ring buffer
         * 
         * @return size_t The number of elements
         */
        size_t getSize() const
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
        size_t getCapacity() const
        {
            return capacity_;
        }

        /**
         * @brief Check if the ring buffer is empty
         * 
         * @return true if empty, false otherwise
         */
        bool isEmpty() const
        {
            return getSize() == 0;
        }

        /**
         * @brief Check if the ring buffer is full
         * 
         * @return true if full, false otherwise
         */
        bool isFull() const
        {
            return getSize() == capacity_;
        }

        /**
         * @brief Push a new element into the ring buffer
         * 
         * @param value The value to push
         * @return true if push was successful, false if buffer is full
         */
        bool push (const T& value)
        {
            const auto currentWrite = writeIndex_.load (std::memory_order_relaxed);
            const auto currentRead = readIndex_.load (std::memory_order_acquire);

            // Check if there is space
            if (currentWrite - currentRead >= capacity_)
                return false;

            // Write the element
            const auto writePos = currentWrite & (capacity_ - 1);
            data_[writePos] = value;

            // Update write index with release semantics to ensure visibility
            writeIndex_.store (currentWrite + 1, std::memory_order_release);
            return true;
        }

        /**
         * @brief Push a new element using move semantics
         * 
         * @param value The value to push (will be moved from)
         * @return true if push was successful, false if buffer is full
         */
        bool push (T&& value)
        {
            const auto currentWrite = writeIndex_.load (std::memory_order_relaxed);
            const auto currentRead = readIndex_.load (std::memory_order_acquire);

            // Check if there is space
            if (currentWrite - currentRead >= capacity_)
                return false;

            // Write the element
            const auto writePos = currentWrite & (capacity_ - 1);
            data_[writePos] = std::move (value);

            // Update write index with release semantics to ensure visibility
            writeIndex_.store (currentWrite + 1, std::memory_order_release);
            return true;
        }

        /**
         * @brief Pop an element from the ring buffer
         * 
         * @param[out] value Reference to store the popped value
         * @return true if pop was successful, false if buffer is empty
         */
        bool pop (T& value)
        {
            const auto currentRead = readIndex_.load (std::memory_order_relaxed);
            const auto currentWrite = writeIndex_.load (std::memory_order_acquire);

            // Check if there are elements to read
            if (currentRead == currentWrite)
                return false;

            // Read the element
            const auto readPos = currentRead & (capacity_ - 1);
            value = std::move (data_[readPos]);

            // Update read index with release semantics to ensure visibility
            readIndex_.store (currentRead + 1, std::memory_order_release);
            return true;
        }

        /**
         * @brief Peek at the next element without removing it
         * 
         * @param[out] value Reference to store the peeked value
         * @return true if peek was successful, false if buffer is empty
         */
        bool peek (T& value) const
        {
            const auto currentRead = readIndex_.load (std::memory_order_relaxed);
            const auto currentWrite = writeIndex_.load (std::memory_order_acquire);

            // Check if there are elements to read
            if (currentRead == currentWrite)
                return false;

            // Read the element without removing it
            const auto readPos = currentRead & (capacity_ - 1);
            value = data_[readPos];
            return true;
        }

        /**
         * @brief Discard a number of elements from the front of the buffer
         * 
         * @param count Number of elements to discard
         * @return size_t The actual number of elements discarded
         */
        size_t discard (size_t count)
        {
            const auto currentRead = readIndex_.load (std::memory_order_relaxed);
            const auto currentWrite = writeIndex_.load (std::memory_order_acquire);

            const auto available = currentWrite - currentRead;
            const auto toDiscard = std::min (count, available);

            if (toDiscard > 0)
                readIndex_.store (currentRead + toDiscard, std::memory_order_release);

            return toDiscard;
        }

        /**
         * @brief Write multiple elements to the buffer
         * 
         * @param data Pointer to the data to write
         * @param count Number of elements to write
         * @return size_t The number of elements actually written
         */
        size_t writeMany (const T* data, size_t count)
        {
            const auto currentWrite = writeIndex_.load (std::memory_order_relaxed);
            const auto currentRead = readIndex_.load (std::memory_order_acquire);

            // Calculate available space
            const auto available = capacity_ - (currentWrite - currentRead);
            const auto toWrite = std::min (count, available);

            for (size_t i = 0; i < toWrite; ++i)
            {
                const auto pos = (currentWrite + i) & (capacity_ - 1);
                data_[pos] = data[i];
            }

            if (toWrite > 0)
                writeIndex_.store (currentWrite + toWrite, std::memory_order_release);

            return toWrite;
        }

        /**
         * @brief Read multiple elements from the buffer
         * 
         * @param data Pointer to the destination buffer
         * @param count Maximum number of elements to read
         * @return size_t The number of elements actually read
         */
        size_t readMany (T* data, size_t count)
        {
            const auto currentRead = readIndex_.load (std::memory_order_relaxed);
            const auto currentWrite = writeIndex_.load (std::memory_order_acquire);

            // Calculate available elements
            const auto available = currentWrite - currentRead;
            const auto toRead = std::min (count, available);

            for (size_t i = 0; i < toRead; ++i)
            {
                const auto pos = (currentRead + i) & (capacity_ - 1);
                data[i] = std::move (data_[pos]);
            }

            if (toRead > 0)
                readIndex_.store (currentRead + toRead, std::memory_order_release);

            return toRead;
        }

    private:
        // Helper functions for power-of-two calculations
        static bool isPowerOfTwo (size_t x)
        {
            return x > 0 && (x & (x - 1)) == 0;
        }

        static size_t nextPowerOfTwo (size_t x)
        {
            --x;
            x |= x >> 1;
            x |= x >> 2;
            x |= x >> 4;
            x |= x >> 8;
            x |= x >> 16;
            if constexpr (sizeof (size_t) == 8)
                x |= x >> 32;
            return x + 1;
        }

        std::vector<T> data_; // Storage for ring buffer elements
        size_t capacity_; // Current capacity (always a power of 2)
        std::atomic<size_t> readIndex_; // Current read position
        std::atomic<size_t> writeIndex_; // Current write position
    };
}
