/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#include "core/ring_buffer.h"
#include <cassert>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace PlayfulTones::DspToolBox;

void testConstruction()
{
    std::cout << "Testing construction...\n";

    // Default construction
    RingBuffer<float> buffer1;
    assert (buffer1.getCapacity() == RingBuffer<float>::DefaultCapacity);
    assert (buffer1.getSize() == 0);
    assert (buffer1.isEmpty());
    assert (!buffer1.isFull());

    // Custom capacity
    RingBuffer<int> buffer2 (100);
    assert (buffer2.getCapacity() == 128); // Next power of 2
    assert (buffer2.getSize() == 0);

    // Power of 2 capacity
    RingBuffer<double> buffer3 (256);
    assert (buffer3.getCapacity() == 256);
    assert (buffer3.getSize() == 0);

    std::cout << "Construction tests passed!\n";
}

void testPushPop()
{
    std::cout << "Testing push/pop operations...\n";

    RingBuffer<int> buffer (8);

    // Test push
    for (int i = 0; i < 8; ++i)
    {
        bool result = buffer.push (i);
        assert (result);
        assert (buffer.getSize() == static_cast<size_t> (i + 1));
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
    }

    // Buffer should be empty now
    assert (buffer.isEmpty());
    assert (!buffer.isFull());

    int value;
    assert (!buffer.pop (value)); // Should fail

    std::cout << "Push/pop tests passed!\n";
}

void testPeek()
{
    std::cout << "Testing peek operation...\n";

    RingBuffer<int> buffer (4);

    // Test peek on empty buffer
    int value;
    assert (!buffer.peek (value));

    // Add some elements
    buffer.push (10);
    buffer.push (20);

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

    RingBuffer<int> buffer (4);

    // Add some elements
    buffer.push (10);
    buffer.push (20);
    buffer.push (30);

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

void testResize()
{
    std::cout << "Testing resize operation...\n";

    RingBuffer<int> buffer (4);

    // Add some elements
    buffer.push (10);
    buffer.push (20);
    buffer.push (30);

    assert (buffer.getSize() == 3);
    assert (buffer.getCapacity() == 4);

    // Resize to larger capacity
    assert (buffer.resize (8));
    assert (buffer.getCapacity() == 8);
    assert (buffer.getSize() == 3);

    // Check elements are preserved
    int value;
    assert (buffer.pop (value));
    assert (value == 10);
    assert (buffer.pop (value));
    assert (value == 20);
    assert (buffer.pop (value));
    assert (value == 30);

    // Fill the buffer
    for (int i = 0; i < 8; ++i)
    {
        buffer.push (i);
    }

    assert (buffer.isFull());

    // Resize to smaller capacity (still large enough for current elements)
    assert (buffer.resize (8));
    assert (buffer.getCapacity() == 8);
    assert (buffer.getSize() == 8);
    assert (buffer.isFull());

    std::cout << "Resize tests passed!\n";
}

void testWraparound()
{
    std::cout << "Testing wraparound behavior...\n";

    RingBuffer<int> buffer (4);

    // Fill the buffer
    for (int i = 0; i < 4; ++i)
    {
        buffer.push (i);
    }

    // Remove 2 elements
    int value;
    buffer.pop (value);
    buffer.pop (value);

    // Add 2 more elements (should wrap around)
    buffer.push (100);
    buffer.push (101);

    // Check elements in order
    buffer.pop (value);
    assert (value == 2);
    buffer.pop (value);
    assert (value == 3);
    buffer.pop (value);
    assert (value == 100);
    buffer.pop (value);
    assert (value == 101);

    std::cout << "Wraparound tests passed!\n";
}

void testBulkOperations()
{
    std::cout << "Testing bulk operations...\n";

    RingBuffer<int> buffer (16);

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

    for (size_t i = 0; i < output.size(); ++i)
    {
        assert (output[i] == input[i]);
    }

    // Test discard
    size_t discarded = buffer.discard (2);
    assert (discarded == 2);
    assert (buffer.getSize() == 2);

    // Read remaining elements
    buffer.readMany (output.data(), 2);
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

void testMultithreading()
{
    std::cout << "Testing multithreading behavior...\n";

    RingBuffer<int> buffer (1024);
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
        testResize();
        testWraparound();
        testBulkOperations();
        testMultithreading();

        std::cout << "All tests passed!\n";
        return 0;
    } catch (const std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
}
