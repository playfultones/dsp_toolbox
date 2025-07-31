/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#include "core/ring_buffer.h"
#include "helpers/compilationhelpers.h"
#include <cassert>
#include <concepts>
#include <iostream>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

using namespace PlayfulTones::DspToolBox;

// Utilities for testing allocation and lock-free properties
namespace detail
{
    // Custom allocator that will cause a compile error if used
    template <typename T>
    struct NoAllocAllocator
    {
        using value_type = T;

        NoAllocAllocator() = default;

        template <typename U>
        constexpr NoAllocAllocator (const NoAllocAllocator<U>&) noexcept
        {
        }

        [[nodiscard]] T* allocate (std::size_t)
        {
            // This will cause a compile error if instantiated
            static_assert (sizeof (T) == 0, "Allocation detected in no-alloc context");
            return nullptr;
        }

        void deallocate (T*, std::size_t) noexcept {}
    };

    template <typename T>
    bool operator== (const NoAllocAllocator<T>&, const NoAllocAllocator<T>&)
    {
        return true;
    }

    template <typename T>
    bool operator!= (const NoAllocAllocator<T>&, const NoAllocAllocator<T>&)
    {
        return false;
    }

    // Type that will cause a compile error if allocations happen
    template <typename T>
    using NoAllocVector = std::vector<T, NoAllocAllocator<T>>;

    // Runtime allocation tracking allocator - counts allocations
    template <typename T>
    struct CountingAllocator
    {
        using value_type = T;

        static inline std::atomic<size_t> allocationCount { 0 };
        static inline std::atomic<size_t> deallocationCount { 0 };

        CountingAllocator() = default;

        template <typename U>
        constexpr CountingAllocator (const CountingAllocator<U>&) noexcept
        {
        }

        [[nodiscard]] T* allocate (std::size_t n)
        {
            allocationCount.fetch_add (1, std::memory_order_relaxed);
            return static_cast<T*> (::operator new (n * sizeof (T)));
        }

        void deallocate (T* p, std::size_t) noexcept
        {
            deallocationCount.fetch_add (1, std::memory_order_relaxed);
            ::operator delete (p);
        }

        static void resetCounters()
        {
            allocationCount.store (0, std::memory_order_relaxed);
            deallocationCount.store (0, std::memory_order_relaxed);
        }

        static size_t getAllocationCount()
        {
            return allocationCount.load (std::memory_order_relaxed);
        }

        static size_t getDeallocationCount()
        {
            return deallocationCount.load (std::memory_order_relaxed);
        }
    };

    template <typename T>
    bool operator== (const CountingAllocator<T>&, const CountingAllocator<T>&)
    {
        return true;
    }

    template <typename T>
    bool operator!= (const CountingAllocator<T>&, const CountingAllocator<T>&)
    {
        return false;
    }

    // Type for runtime allocation tracking
    template <typename T>
    using CountingVector = std::vector<T, CountingAllocator<T>>;

    // C++20 concept to verify a method doesn't allocate and is noexcept
    template <typename F>
    concept NonAllocating = requires (F f) {
        { f() } noexcept;
    };
}

void testConstruction()
{
    std::cout << "Testing construction...\n";

    // Default construction with compile-time capacity
    RingBuffer<float, 1024> buffer1;
    assert (buffer1.getCapacity() == 1024);
    assert (buffer1.getSize() == 0);
    assert (buffer1.isEmpty());
    assert (!buffer1.isFull());

    // Custom capacity
    RingBuffer<int, 128> buffer2;
    assert (buffer2.getCapacity() == 128);
    assert (buffer2.getSize() == 0);

    // Different capacity
    RingBuffer<double, 256> buffer3;
    assert (buffer3.getCapacity() == 256);
    assert (buffer3.getSize() == 0);

    std::cout << "Construction tests passed!\n";
}

void testPushPop()
{
    std::cout << "Testing push/pop operations...\n";

    RingBuffer<int, 8> buffer;

    // Test push
    for (int i = 0; i < 8; ++i)
    {
        bool result = buffer.push (i);
        assert (result && "Push should succeed");
        assert (buffer.getSize() == static_cast<size_t> (i + 1));
        markUsed (result);
    }

    // Buffer should be full now
    assert (buffer.isFull());
    assert (!buffer.isEmpty());
    assert (!buffer.push (100)); // Should fail

    // Test pop
    for (int i = 0; i < 8; ++i)
    {
        int value;
        bool result = buffer.pop (value);
        assert (result);
        assert (value == i);
        assert (buffer.getSize() == static_cast<size_t> (7 - i));
        markUsed (result);
    }

    // Buffer should be empty now
    assert (buffer.isEmpty());
    assert (!buffer.isFull());

    int value;
    assert (!buffer.pop (value)); // Should fail
    markUsed (value);

    std::cout << "Push/pop tests passed!\n";
}

void testPeek()
{
    std::cout << "Testing peek operation...\n";

    RingBuffer<int, 4> buffer;

    // Test peek on empty buffer
    int value;
    assert (!buffer.peek (value));
    markUsed (value);

    // Add some elements
    markUsed (buffer.push (10));
    markUsed (buffer.push (20));

    // Peek should return the first element without removing it
    assert (buffer.peek (value));
    assert (value == 10);
    assert (buffer.getSize() == 2);

    // Peek again should return the same element
    assert (buffer.peek (value));
    assert (value == 10);

    // Pop should remove the first element
    assert (buffer.pop (value));
    assert (value == 10);
    assert (buffer.getSize() == 1);

    // Peek should now return the next element
    assert (buffer.peek (value));
    assert (value == 20);

    std::cout << "Peek tests passed!\n";
}

void testClear()
{
    std::cout << "Testing clear operation...\n";

    RingBuffer<int, 4> buffer;

    // Add some elements
    markUsed (buffer.push (10));
    markUsed (buffer.push (20));
    markUsed (buffer.push (30));

    assert (buffer.getSize() == 3);

    // Clear the buffer
    buffer.clear();

    assert (buffer.getSize() == 0);
    assert (buffer.isEmpty());
    assert (!buffer.isFull());

    // Should be able to push new elements
    assert (buffer.push (40));
    assert (buffer.getSize() == 1);

    std::cout << "Clear tests passed!\n";
}

void testCapacityConstraints()
{
    std::cout << "Testing capacity constraints...\n";

    RingBuffer<int, 4> buffer;

    // Add some elements
    markUsed (buffer.push (10));
    markUsed (buffer.push (20));
    markUsed (buffer.push (30));

    assert (buffer.getSize() == 3);
    assert (buffer.getCapacity() == 4);

    // Check elements are correct
    int value;
    assert (buffer.pop (value));
    assert (value == 10);
    assert (buffer.pop (value));
    assert (value == 20);
    assert (buffer.pop (value));
    assert (value == 30);
    markUsed (value);

    // Fill the buffer to capacity
    for (int i = 0; i < 4; ++i)
    {
        markUsed (buffer.push (i));
    }

    assert (buffer.isFull());
    assert (buffer.getCapacity() == 4);
    assert (buffer.getSize() == 4);

    std::cout << "Capacity constraint tests passed!\n";
}

void testWraparound()
{
    std::cout << "Testing wraparound behavior...\n";

    RingBuffer<int, 4> buffer;

    // Fill the buffer
    for (int i = 0; i < 4; ++i)
    {
        markUsed (buffer.push (i));
    }

    // Remove 2 elements
    int value;
    markUsed (buffer.pop (value));
    markUsed (buffer.pop (value));

    // Add 2 more elements (should wrap around)
    markUsed (buffer.push (100));
    markUsed (buffer.push (101));

    // Check elements in order
    markUsed (buffer.pop (value));
    assert (value == 2);
    markUsed (buffer.pop (value));
    assert (value == 3);
    markUsed (buffer.pop (value));
    assert (value == 100);
    markUsed (buffer.pop (value));
    assert (value == 101);

    std::cout << "Wraparound tests passed!\n";
}

void testBulkOperations()
{
    std::cout << "Testing bulk operations...\n";

    RingBuffer<int, 16> buffer;

    // Test writeMany
    std::vector<int> input = { 1, 2, 3, 4, 5, 6, 7, 8 };
    size_t written = buffer.writeMany (input.data(), input.size());

    assert (written == input.size());
    assert (buffer.getSize() == input.size());

    // Test readMany
    std::vector<int> output (4);
    size_t read = buffer.readMany (output.data(), output.size());

    assert (read == output.size());
    assert (buffer.getSize() == input.size() - output.size());
    markUsed (read);

    for (size_t i = 0; i < output.size(); ++i)
    {
        assert (output[i] == input[i]);
    }

    // Test discard
    size_t discarded = buffer.discard (2);
    assert (discarded == 2);
    assert (buffer.getSize() == 2);
    markUsed (discarded);

    // Read remaining elements
    markUsed (buffer.readMany (output.data(), 2));
    assert (output[0] == 7);
    assert (output[1] == 8);

    // Write many to fill buffer
    std::vector<int> large (16);
    for (int i = 0; i < 16; ++i)
        large[i] = i + 10;

    written = buffer.writeMany (large.data(), large.size());
    assert (written == 16);
    assert (buffer.isFull());

    // Try to write more
    written = buffer.writeMany (large.data(), 4);
    assert (written == 0); // No space left

    std::cout << "Bulk operations tests passed!\n";
}

// Define types for testing properties with fixed capacity
template <typename T, size_t Capacity = 64>
using NoAllocRingBuffer = RingBuffer<T, Capacity>;
template <typename T, size_t Capacity = 64>
using CountingRingBuffer = RingBuffer<T, Capacity>;

// This function checks method signatures with compile-time capacity
template <typename T>
void testNoAllocationInCriticalMethods()
{
    // Define a dummy array for method signatures that need it
    T arr[4];

    // Use declval to get references to objects without constructing them
    using Buffer = NoAllocRingBuffer<T, 64>;

    // Check signatures of critical methods using decltype and declval
    using PushMethod = decltype (std::declval<Buffer&>().push (std::declval<const T&>()));
    using PopMethod = decltype (std::declval<Buffer&>().pop (std::declval<T&>()));
    using PeekMethod = decltype (std::declval<Buffer&>().peek (std::declval<T&>()));
    using ReadManyMethod = decltype (std::declval<Buffer&>().readMany (arr, 4));
    using WriteManyMethod = decltype (std::declval<Buffer&>().writeMany (arr, 4));

    static_assert (std::is_same_v<PushMethod, bool>, "push method has incorrect signature");
    static_assert (std::is_same_v<PopMethod, bool>, "pop method has incorrect signature");
    static_assert (std::is_same_v<PeekMethod, bool>, "peek method has incorrect signature");
    static_assert (std::is_same_v<ReadManyMethod, size_t>, "readMany method has incorrect signature");
    static_assert (std::is_same_v<WriteManyMethod, size_t>, "writeMany method has incorrect signature");
}

// Runtime verification of no-allocation property for critical methods
void testRuntimeAllocationTracking()
{
    std::cout << "Testing runtime allocation tracking...\n";

    // Create a buffer with compile-time capacity (no allocation)
    CountingRingBuffer<int, 64> buffer;

    // Reset counters after initial construction (should be 0 anyway)
    detail::CountingAllocator<int>::resetCounters();

    // Push should not allocate
    int testValue = 42;
    markUsed (buffer.push (testValue));
    assert (detail::CountingAllocator<int>::getAllocationCount() == 0);

    // Pop should not allocate
    int outValue;
    markUsed (buffer.pop (outValue));
    assert (detail::CountingAllocator<int>::getAllocationCount() == 0);

    // Fill buffer
    for (int i = 0; i < 64; i++)
    {
        markUsed (buffer.push (i));
    }

    // Bulk operations should not allocate
    int values[10];
    markUsed (buffer.readMany (values, 10));
    assert (detail::CountingAllocator<int>::getAllocationCount() == 0);

    markUsed (buffer.writeMany (values, 10));
    assert (detail::CountingAllocator<int>::getAllocationCount() == 0);

    // Peek should not allocate
    markUsed (buffer.peek (outValue));
    assert (detail::CountingAllocator<int>::getAllocationCount() == 0);

    std::cout << "Runtime allocation tracking tests passed!\n";
}

// Compile-time verification tests
void testCompileTimeProperties()
{
    std::cout << "Testing compile-time properties...\n";

    // Verify atomic operations are lock-free
    static_assert (std::atomic<size_t>::is_always_lock_free,
        "std::atomic<size_t> must be lock-free");

    // Verify no allocation in critical methods (compile-time check)
    testNoAllocationInCriticalMethods<int>();
    testNoAllocationInCriticalMethods<float>();
    testNoAllocationInCriticalMethods<std::pair<int, float>>();

    // Ensure push operation is non-allocating and noexcept
    auto testPush = [buffer = RingBuffer<int, 16>{}, value = 42]() mutable noexcept {
        return buffer.push (value);
    };
    static_assert (detail::NonAllocating<decltype (testPush)>);

    // Ensure pop operation is non-allocating and noexcept
    auto testPop = [buffer = RingBuffer<int, 16>{}, value = 0]() mutable noexcept {
        return markUsed (buffer.pop (value));
    };
    static_assert (detail::NonAllocating<decltype (testPop)>);
    // Ensure peek operation is non-allocating and noexcept
    auto testPeek = [buffer = RingBuffer<int, 16>{}, value = 0]() mutable noexcept {
        return buffer.peek (value);
    };
    static_assert (detail::NonAllocating<decltype (testPeek)>);

    std::cout << "Compile-time property tests passed!\n";
}

void testMultithreading()
{
    std::cout << "Testing multithreading behavior...\n";

    RingBuffer<int, 1024> buffer;
    const int numItems = 10000;
    std::atomic<bool> producerDone (false);
    std::atomic<int> consumedItems (0);

    // Producer thread
    std::thread producer ([&]() {
        for (int i = 0; i < numItems; ++i)
        {
            while (!buffer.push (i))
            {
                // Buffer full, yield and retry
                std::this_thread::yield();
            }
        }
        producerDone.store (true);
    });

    // Consumer thread
    std::thread consumer ([&]() {
        int value;
        while (consumedItems < numItems)
        {
            if (buffer.pop (value))
            {
                assert (value == consumedItems);
                consumedItems++;
            }
            else if (!producerDone)
            {
                // Buffer empty but producer not done, yield and retry
                std::this_thread::yield();
            }
        }
    });

    producer.join();
    consumer.join();

    assert (consumedItems == numItems);
    assert (buffer.isEmpty());

    std::cout << "Multithreading tests passed!\n";
}

int main()
{
    try
    {
        testConstruction();
        testPushPop();
        testPeek();
        testClear();
        testCapacityConstraints();
        testWraparound();
        testBulkOperations();
        testMultithreading();
        testCompileTimeProperties();
        testRuntimeAllocationTracking();

        std::cout << "All tests passed!\n";
        return 0;
    } catch (const std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
}
