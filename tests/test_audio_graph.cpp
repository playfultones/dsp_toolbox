/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#include "dynamics/gain.h"
#include "generators/noisegenerators.h"
#include "helpers/compilationhelpers.h"
#include "processors/graph/audio_graph_node.h"
#include "processors/graph/audio_processor_graph.h"
#include "processors/graph/graph_builder.h"
#include "processors/graph/output_processor.h"
#include "validators/buffer_validators.h"

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

using namespace PlayfulTones::DspToolBox;

// Mock processor for testing using new CRTP architecture
template <typename SampleType = float,
    size_t BlockSize = 512,
    size_t SampleRate = 44100,
    size_t NumChannels = 2>
class MockProcessor : public ProcessorBase<MockProcessor<SampleType, BlockSize, SampleRate, NumChannels>,
                          SampleType,
                          BlockSize,
                          SampleRate,
                          NumChannels>
{
public:
    using Base = ProcessorBase<MockProcessor, SampleType, BlockSize, SampleRate, NumChannels>;
    using AudioBuffer = typename Base::AudioBuffer;
    using sample_type = SampleType;

    MockProcessor() = default;

    void prepare_impl() noexcept
    {
        prepared_ = true;
    }

    void process_audio_impl (AudioBuffer& buffer) noexcept
    {
        processCallCount_++;
        lastProcessedChannels_ = Base::num_channels;
        lastProcessedFrames_ = Base::block_size;

        // Optional processing behavior for testing
        if (processingBehavior_)
            processingBehavior_ (buffer);
    }

    void reset_impl() noexcept
    {
        resetCallCount_++;
        prepared_ = false;
    }

    void process_control_impl() noexcept
    {
        // No control processing needed for mock
    }

    bool wasPrepared() const { return prepared_; }
    int getProcessCallCount() const { return processCallCount_; }
    int getResetCallCount() const { return resetCallCount_; }
    size_t getLastProcessedChannels() const { return lastProcessedChannels_; }
    size_t getLastProcessedFrames() const { return lastProcessedFrames_; }

    // For configuring test behavior
    void setProcessingBehavior (std::function<void (AudioBuffer&)> behavior)
    {
        processingBehavior_ = std::move (behavior);
    }

private:
    std::atomic<bool> prepared_ { false };
    std::atomic<int> processCallCount_ { 0 };
    std::atomic<int> resetCallCount_ { 0 };
    std::atomic<size_t> lastProcessedChannels_ { 0 };
    std::atomic<size_t> lastProcessedFrames_ { 0 };
    std::function<void (AudioBuffer&)> processingBehavior_;
};

// Mock processor that can be bypassed
template <typename SampleType = float,
    size_t BlockSize = 512,
    size_t SampleRate = 44100,
    size_t NumChannels = 2>
class BypassableProcessor : public ProcessorBase<BypassableProcessor<SampleType, BlockSize, SampleRate, NumChannels>,
                                SampleType,
                                BlockSize,
                                SampleRate,
                                NumChannels>
{
public:
    using Base = ProcessorBase<BypassableProcessor, SampleType, BlockSize, SampleRate, NumChannels>;
    using AudioBuffer = typename Base::AudioBuffer;
    using sample_type = SampleType;

    BypassableProcessor() = default;

    void prepare_impl() noexcept
    {
        prepared_ = true;
    }

    void process_audio_impl (AudioBuffer& buffer) noexcept
    {
        if (!bypassed_.load (std::memory_order_relaxed))
        {
            processCallCount_++;
            lastProcessedChannels_ = Base::num_channels;
            lastProcessedFrames_ = Base::block_size;

            // Optional processing behavior for testing
            if (processingBehavior_)
                processingBehavior_ (buffer);
        }
    }

    void reset_impl() noexcept
    {
        resetCallCount_++;
        prepared_ = false;
    }

    void process_control_impl() noexcept
    {
        // No control processing needed for mock
    }

    bool wasPrepared() const { return prepared_; }
    int getProcessCallCount() const { return processCallCount_; }
    int getResetCallCount() const { return resetCallCount_; }
    size_t getLastProcessedChannels() const { return lastProcessedChannels_; }
    size_t getLastProcessedFrames() const { return lastProcessedFrames_; }

    // For configuring test behavior
    void setProcessingBehavior (std::function<void (AudioBuffer&)> behavior)
    {
        processingBehavior_ = std::move (behavior);
    }

    void setBypassed (bool shouldBypass) { bypassed_.store (shouldBypass, std::memory_order_relaxed); }
    bool isBypassed() const { return bypassed_.load (std::memory_order_relaxed); }

private:
    std::atomic<bool> prepared_ { false };
    std::atomic<int> processCallCount_ { 0 };
    std::atomic<int> resetCallCount_ { 0 };
    std::atomic<size_t> lastProcessedChannels_ { 0 };
    std::atomic<size_t> lastProcessedFrames_ { 0 };
    std::function<void (AudioBuffer&)> processingBehavior_;
    std::atomic<bool> bypassed_ { false };
};

// Type aliases for convenience
using MockProcessorF32 = MockProcessor<float, 512, 44100, 2>;
using BypassableProcessorF32 = BypassableProcessor<float, 512, 44100, 2>;
using GraphBuilderF32 = GraphBuilder<float, 512, 44100, 2>;
using AudioProcessorGraphF32 = AudioProcessorGraph<float, 512, 44100, 2>;
using GainF32 = Gain<float, 512, 44100, 2>;

// ======== 1. NODE CREATION TESTS ========

void testNodeCreation()
{
    std::cout << "Testing node creation..." << std::endl;

    // Test valid node creation
    GraphBuilderF32 builder;

    // Create node with template method
    auto node1 = builder.addNode<MockProcessorF32>();
    assert (node1 != nullptr && "Node creation should succeed with template method");
    assert (node1->getId() > 0 && "Node should have a valid ID");

    // Create another node
    auto node2 = builder.addNode<MockProcessorF32>();
    assert (node2 != nullptr && "Node creation should succeed with template method");
    assert (node2->getId() != node1->getId() && "Nodes should have unique IDs");

    // Test that we can retrieve nodes by ID
    assert (builder.getNode (node1->getId()) == node1 && "Should retrieve correct node by ID");
    assert (builder.getNode (node2->getId()) == node2 && "Should retrieve correct node by ID");

    // Test invalid node retrieval
    assert (builder.getNode (999) == nullptr && "Should return nullptr for invalid ID");

    std::cout << "Node creation tests passed!" << std::endl;
}

// ======== 2. CONNECTION MANAGEMENT TESTS ========

void testConnectionManagement()
{
    std::cout << "Testing connection management..." << std::endl;

    GraphBuilderF32 builder;

    // Create nodes
    auto node1 = builder.addNode<MockProcessorF32>();
    auto node2 = builder.addNode<MockProcessorF32>();
    auto node3 = builder.addNode<MockProcessorF32>();

    // Test valid connection
    bool connected = builder.connect (node1->getId(), node2->getId());
    assert (connected && "Should connect valid nodes");

    // Test connection to output node
    auto outputNode = builder.getOutputNode();
    assert (outputNode != nullptr && "Output node should exist");
    connected = builder.connect (node2->getId(), outputNode->getId());
    assert (connected && "Should connect to output node");

    // Test multi-channel connections
    connected = builder.connect (node1->getId(), node3->getId(), 0, 1);
    assert (connected && "Should connect to different channels");

    // Test disconnection
    bool disconnected = builder.disconnect (node1->getId(), node2->getId());
    assert (disconnected && "Should disconnect existing connection");

    // Test duplicate connection
    connected = builder.connect (node1->getId(), node2->getId());
    assert (connected && "Should allow reconnection after disconnect");

    // Test disconnect with invalid nodes
    disconnected = builder.disconnect (999, node2->getId());
    assert (!disconnected && "Should fail to disconnect non-existent source");

    disconnected = builder.disconnect (node1->getId(), 999);
    assert (!disconnected && "Should fail to disconnect non-existent target");

    std::cout << "Connection management tests passed!" << std::endl;
}

// ======== 3. ROUTING AND BUFFER HANDLING TESTS ========

void testRoutingAndBufferHandling()
{
    std::cout << "Testing routing and buffer handling..." << std::endl;

    constexpr size_t numChannels = 2;
    constexpr size_t numFrames = 512;

    AudioProcessorGraphF32 graph;

    // Create processors that will modify the buffer in specific ways
    auto gainNode1Id = graph.addProcessor<GainF32> (0.5f);
    auto gainNode2Id = graph.addProcessor<GainF32> (2.0f);

    // Connect in series to create a chain
    graph.connect (gainNode1Id, gainNode2Id);
    graph.connect (gainNode2Id, graph.getOutputNode()->getId());

    // Prepare the graph
    graph.prepare();

    // Create template-based audio buffer
    std::array<std::array<float, numFrames>, numChannels> bufferData;

    // Initialize buffer spans and fill with known values
    AudioProcessorGraphF32::AudioBuffer buffer {
        std::span<float, numFrames> (bufferData[0]),
        std::span<float, numFrames> (bufferData[1])
    };
    for (size_t ch = 0; ch < numChannels; ++ch)
    {
        for (size_t i = 0; i < numFrames; ++i)
        {
            buffer[ch][i] = 1.0f; // Fill with ones
        }
    }

    // Process the buffer
    graph.process_audio (buffer);

    // Verify the processing: 1.0 * 0.5 * 2.0 = 1.0
    for (size_t ch = 0; ch < numChannels; ++ch)
    {
        bool allCorrect = true;
        for (size_t i = 0; i < numFrames; ++i)
        {
            if (std::abs (buffer[ch][i] - 1.0f) >= 0.00001f)
            {
                allCorrect = false;
                break;
            }
        }
        assert (allCorrect && "Buffer values should be 1.0 after processing");
    }

    std::cout << "Routing and buffer handling tests passed!" << std::endl;
}

// ======== 4. BASIC SIGNAL FLOW TEST ========

void testBasicSignalFlow()
{
    std::cout << "Testing basic signal flow..." << std::endl;

    constexpr float kGain = 0.5f;
    constexpr size_t numChannels = 2;
    constexpr size_t numFrames = 512; // Use block size for template compatibility

    // Create audio processor graph
    AudioProcessorGraphF32 graph;

    // Add a gain processor node and get its ID
    auto gainNodeId = graph.addProcessor<GainF32> (kGain);

    // Connect gain to output
    graph.connect (gainNodeId, graph.getOutputNode()->getId());

    // Prepare graph
    graph.prepare();

    // Create template-based audio buffers
    std::array<std::array<float, numFrames>, numChannels> audioBufferData;
    std::array<std::array<float, numFrames>, numChannels> expectedBufferData;
    std::array<std::array<float, numFrames>, numChannels> originalBufferData;

    // Initialize buffer spans
    AudioProcessorGraphF32::AudioBuffer audioBuffer {
        std::span<float, numFrames> (audioBufferData[0]),
        std::span<float, numFrames> (audioBufferData[1])
    };
    AudioProcessorGraphF32::AudioBuffer expectedBuffer {
        std::span<float, numFrames> (expectedBufferData[0]),
        std::span<float, numFrames> (expectedBufferData[1])
    };
    AudioProcessorGraphF32::AudioBuffer originalBuffer {
        std::span<float, numFrames> (originalBufferData[0]),
        std::span<float, numFrames> (originalBufferData[1])
    };

    // Fill buffer with white noise using raw pointers for compatibility
    std::array<float*, numChannels> rawPointers;
    for (size_t ch = 0; ch < numChannels; ++ch)
    {
        rawPointers[ch] = audioBufferData[ch].data();
    }
    generateWhiteNoise (rawPointers.data(), numChannels, numFrames, 1.0f);

    // Copy to expected buffer and apply gain, also make original copy
    for (size_t ch = 0; ch < numChannels; ++ch)
    {
        for (size_t i = 0; i < numFrames; ++i)
        {
            originalBuffer[ch][i] = audioBuffer[ch][i];
            expectedBuffer[ch][i] = audioBuffer[ch][i] * kGain;
        }
    }

    // Process the audio buffer
    graph.process_audio (audioBuffer);

    // Verify gain was applied correctly
    bool allSamplesCorrect = true;
    for (size_t ch = 0; ch < numChannels; ++ch)
    {
        for (size_t i = 0; i < numFrames; ++i)
        {
            if (std::abs (audioBuffer[ch][i] - expectedBuffer[ch][i]) >= 0.00001f)
            {
                allSamplesCorrect = false;
                break;
            }
        }
        if (!allSamplesCorrect)
            break;
    }

    // Debug output for first few samples if test fails
    if (!allSamplesCorrect)
    {
        std::cerr << "First few samples comparison:" << std::endl;
        for (size_t i = 0; i < std::min (static_cast<size_t> (10), numFrames); ++i)
        {
            std::cerr << "Sample " << i << ": Original=" << originalBuffer[0][i]
                      << ", Expected=" << expectedBuffer[0][i]
                      << ", Got=" << audioBuffer[0][i] << std::endl;
        }
    }

    assert (allSamplesCorrect && "All samples should be exactly half of the original values");
    std::cout << "Basic signal flow test passed!" << std::endl;
}

// ======== 5. MULTI-NODE SIGNAL FLOW TEST ========

void testMultiNodeSignalFlow()
{
    std::cout << "Testing multi-node signal flow..." << std::endl;

    constexpr size_t numChannels = 2;
    constexpr size_t numFrames = 512;

    // Create audio processor graph
    AudioProcessorGraphF32 graph;

    // Add two gain processor nodes in series
    auto gainNode1Id = graph.addProcessor<GainF32> (0.8f);
    auto gainNode2Id = graph.addProcessor<GainF32> (0.625f);

    // Connect the nodes
    graph.connect (gainNode1Id, gainNode2Id);
    graph.connect (gainNode2Id, graph.getOutputNode()->getId());

    // Prepare graph
    graph.prepare();

    // Create template-based audio buffers
    std::array<std::array<float, numFrames>, numChannels> audioBufferData;
    std::array<std::array<float, numFrames>, numChannels> expectedBufferData;

    // Initialize buffer spans
    AudioProcessorGraphF32::AudioBuffer audioBuffer {
        std::span<float, numFrames> (audioBufferData[0]),
        std::span<float, numFrames> (audioBufferData[1])
    };
    AudioProcessorGraphF32::AudioBuffer expectedBuffer {
        std::span<float, numFrames> (expectedBufferData[0]),
        std::span<float, numFrames> (expectedBufferData[1])
    };

    // Fill buffer with white noise using raw pointers for compatibility
    std::array<float*, numChannels> rawPointers;
    for (size_t ch = 0; ch < numChannels; ++ch)
    {
        rawPointers[ch] = audioBufferData[ch].data();
    }
    generateWhiteNoise (rawPointers.data(), numChannels, numFrames, 1.0f);

    // Copy to expected buffer and apply both gains (0.8 * 0.625 = 0.5)
    for (size_t ch = 0; ch < numChannels; ++ch)
    {
        for (size_t i = 0; i < numFrames; ++i)
        {
            expectedBuffer[ch][i] = audioBuffer[ch][i] * 0.8f * 0.625f;
        }
    }

    // Process the audio buffer
    graph.process_audio (audioBuffer);

    // Verify gain was applied correctly
    bool allSamplesCorrect = true;
    for (size_t ch = 0; ch < numChannels; ++ch)
    {
        for (size_t i = 0; i < numFrames; ++i)
        {
            if (std::abs (audioBuffer[ch][i] - expectedBuffer[ch][i]) >= 0.00001f)
            {
                allSamplesCorrect = false;
                break;
            }
        }
        if (!allSamplesCorrect)
            break;
    }

    assert (allSamplesCorrect && "Samples should have both gains applied");
    std::cout << "Multi-node signal flow test passed!" << std::endl;
    markUsed (allSamplesCorrect);
}

// ======== 6. TOPOLOGY UPDATES TESTS ========

void testTopologyUpdates()
{
    std::cout << "Testing topology updates..." << std::endl;

    AudioProcessorGraphF32 graph;

    // Create a more complex graph
    auto nodeA = graph.addProcessor<MockProcessorF32>();
    auto nodeB = graph.addProcessor<MockProcessorF32>();
    auto nodeC = graph.addProcessor<MockProcessorF32>();
    auto nodeD = graph.addProcessor<MockProcessorF32>();

    // Set up initial connections: A->B->D->Output, A->C->D
    graph.connect (nodeA, nodeB);
    graph.connect (nodeB, nodeD);
    graph.connect (nodeA, nodeC);
    graph.connect (nodeC, nodeD);
    graph.connect (nodeD, graph.getOutputNode()->getId());

    // Prepare the graph
    graph.prepare();

    // Create template-based audio buffer
    constexpr size_t numChannels = 2;
    constexpr size_t numFrames = 512;
    std::array<std::array<float, numFrames>, numChannels> bufferData;

    AudioProcessorGraphF32::AudioBuffer buffer {
        std::span<float, numFrames> (bufferData[0]),
        std::span<float, numFrames> (bufferData[1])
    };
    for (size_t ch = 0; ch < numChannels; ++ch)
    {
        // Fill with test data
        for (size_t i = 0; i < numFrames; ++i)
        {
            buffer[ch][i] = 1.0f;
        }
    }

    // Process initial graph
    graph.process_audio (buffer);

    // Verify each processor was called exactly once
    auto* procA = graph.getProcessor<MockProcessorF32> (nodeA);
    auto* procB = graph.getProcessor<MockProcessorF32> (nodeB);
    auto* procC = graph.getProcessor<MockProcessorF32> (nodeC);
    auto* procD = graph.getProcessor<MockProcessorF32> (nodeD);

    assert (procA->getProcessCallCount() == 1 && "ProcessorA should be called once");
    assert (procB->getProcessCallCount() == 1 && "ProcessorB should be called once");
    assert (procC->getProcessCallCount() == 1 && "ProcessorC should be called once");
    assert (procD->getProcessCallCount() == 1 && "ProcessorD should be called once");
    markUsed (procA, procB, procC, procD);

    // Change topology: Remove B and connect A directly to D
    graph.disconnect (nodeA, nodeB);
    graph.disconnect (nodeB, nodeD);
    graph.connect (nodeA, nodeD);

    // Process with new topology
    graph.process_audio (buffer);

    // Verify call counts after topology change
    assert (procA->getProcessCallCount() == 2 && "ProcessorA should be called again");
    assert (procB->getProcessCallCount() == 1 && "ProcessorB should not be called again");
    assert (procC->getProcessCallCount() == 2 && "ProcessorC should be called again");
    assert (procD->getProcessCallCount() == 2 && "ProcessorD should be called again");

    std::cout << "Topology updates tests passed!" << std::endl;
}

// ======== 7. CONCURRENCY SAFETY TESTS ========

void testConcurrencySafety()
{
    std::cout << "Testing concurrency safety..." << std::endl;

    AudioProcessorGraphF32 graph;

    // Create initial nodes
    auto nodeA = graph.addProcessor<MockProcessorF32>();
    auto nodeB = graph.addProcessor<MockProcessorF32>();

    // Connect to output
    graph.connect (nodeA, nodeB);
    graph.connect (nodeB, graph.getOutputNode()->getId());

    // Prepare graph
    graph.prepare();

    // Create buffer for processing
    constexpr size_t numChannels = 2;
    constexpr size_t numFrames = 512;
    std::array<std::array<float, numFrames>, numChannels> bufferData;

    AudioProcessorGraphF32::AudioBuffer buffer {
        std::span<float, numFrames> (bufferData[0]),
        std::span<float, numFrames> (bufferData[1])
    };
    for (size_t ch = 0; ch < numChannels; ++ch)
    {
        // Fill with test data
        for (size_t i = 0; i < numFrames; ++i)
        {
            buffer[ch][i] = 1.0f;
        }
    }

    // Flag to stop audio thread
    std::atomic<bool> shouldStop { false };

    // Audio processing thread
    std::thread audioThread ([&]() {
        while (!shouldStop)
        {
            graph.process_audio (buffer);
            // Simulate audio thread timing
            std::this_thread::sleep_for (std::chrono::milliseconds (5));
        }
    });

    // Main thread: make graph modifications while audio thread is running
    for (int i = 0; i < 20; i++)
    {
        // Add new node
        auto nodeC = graph.addProcessor<MockProcessorF32>();

        // Change connections
        graph.disconnect (nodeA, nodeB);
        graph.connect (nodeA, nodeC);
        graph.connect (nodeC, nodeB);

        // Wait a bit
        std::this_thread::sleep_for (std::chrono::milliseconds (10));

        // Restore original connections
        graph.disconnect (nodeA, nodeC);
        graph.disconnect (nodeC, nodeB);
        graph.connect (nodeA, nodeB);

        // Wait a bit
        std::this_thread::sleep_for (std::chrono::milliseconds (10));

        // Remove node C
        graph.removeProcessor (nodeC);
    }

    // Stop audio thread and wait for it
    shouldStop = true;
    audioThread.join();

    std::cout << "Concurrency safety tests passed!" << std::endl;
}

// ======== 8. CHAIN OF RESPONSIBILITY WORKFLOW TESTS ========

void testChainOfResponsibility()
{
    std::cout << "Testing chain of responsibility workflow..." << std::endl;

    AudioProcessorGraphF32 graph;

    // Create processors with specific behaviors
    auto nodeA = graph.addProcessor<BypassableProcessorF32>();
    auto nodeB = graph.addProcessor<MockProcessorF32>();
    auto nodeC = graph.addProcessor<BypassableProcessorF32>();

    // Set up processing behaviors
    auto* procA = graph.getProcessor<BypassableProcessorF32> (nodeA);
    auto* procB = graph.getProcessor<MockProcessorF32> (nodeB);
    auto* procC = graph.getProcessor<BypassableProcessorF32> (nodeC);
    markUsed (procA, procB, procC);

    // Set up a processing chain: A->B->C->Output
    graph.connect (nodeA, nodeB);
    graph.connect (nodeB, nodeC);
    graph.connect (nodeC, graph.getOutputNode()->getId());

    // Prepare the graph
    graph.prepare();

    // Create a test buffer
    constexpr size_t numChannels = 2;
    constexpr size_t numFrames = 512;
    std::array<std::array<float, numFrames>, numChannels> bufferData;

    AudioProcessorGraphF32::AudioBuffer buffer {
        std::span<float, numFrames> (bufferData[0]),
        std::span<float, numFrames> (bufferData[1])
    };
    for (size_t ch = 0; ch < numChannels; ++ch)
    {
        // Fill with known values
        for (size_t i = 0; i < numFrames; ++i)
        {
            buffer[ch][i] = 1.0f;
        }
    }

    // Test 1: All processors active
    procA->setBypassed (false);
    procC->setBypassed (false);

    graph.process_audio (buffer);

    assert (procA->getProcessCallCount() == 1 && "ProcessorA should be called");
    assert (procB->getProcessCallCount() == 1 && "ProcessorB should be called");
    assert (procC->getProcessCallCount() == 1 && "ProcessorC should be called");

    // Test 2: A bypassed
    procA->setBypassed (true);

    graph.process_audio (buffer);

    assert (procA->getProcessCallCount() == 1 && "Bypassed ProcessorA should not be called");
    assert (procB->getProcessCallCount() == 2 && "ProcessorB should be called again");
    assert (procC->getProcessCallCount() == 2 && "ProcessorC should be called again");

    // Test 3: A and C bypassed
    procC->setBypassed (true);

    graph.process_audio (buffer);

    assert (procA->getProcessCallCount() == 1 && "Bypassed ProcessorA should not be called");
    assert (procB->getProcessCallCount() == 3 && "ProcessorB should be called again");
    assert (procC->getProcessCallCount() == 2 && "Bypassed ProcessorC should not be called");

    std::cout << "Chain of responsibility workflow tests passed!" << std::endl;
}

// Resource tracker for cleanup tests
struct ResourceTracker
{
    static std::atomic<int> instanceCount;
    ResourceTracker() { instanceCount++; }
    ~ResourceTracker() { instanceCount--; }
};

std::atomic<int> ResourceTracker::instanceCount { 0 };

// Tracked processor for cleanup tests
template <typename SampleType = float,
    size_t BlockSize = 512,
    size_t SampleRate = 44100,
    size_t NumChannels = 2>
class TrackedProcessor : public MockProcessor<SampleType, BlockSize, SampleRate, NumChannels>
{
private:
    std::shared_ptr<ResourceTracker> tracker = std::make_shared<ResourceTracker>();
};

using TrackedProcessorF32 = TrackedProcessor<float, 512, 44100, 2>;

// ======== 9. CLEANUP AND TEARDOWN TESTS ========

void testCleanupAndTeardown()
{
    std::cout << "Testing cleanup and teardown..." << std::endl;

    // Test graph scope and cleanup
    {
        AudioProcessorGraphF32 graph;

        // Add processors that use resources
        for (int i = 0; i < 10; i++)
        {
            graph.addProcessor<TrackedProcessorF32>();
        }

        // Force some nodes to be connected
        auto node1 = graph.addProcessor<TrackedProcessorF32>();
        auto node2 = graph.addProcessor<TrackedProcessorF32>();
        graph.connect (node1, node2);
        graph.connect (node2, graph.getOutputNode()->getId());

        assert (ResourceTracker::instanceCount == 12 && "Should have 12 resource trackers");

        // Process some audio
        constexpr size_t numChannels = 2;
        constexpr size_t numFrames = 512;
        std::array<std::array<float, numFrames>, numChannels> bufferData;

        AudioProcessorGraphF32::AudioBuffer buffer {
            std::span<float, numFrames> (bufferData[0]),
            std::span<float, numFrames> (bufferData[1])
        };
        for (size_t ch = 0; ch < numChannels; ++ch)
        {
            for (size_t i = 0; i < numFrames; ++i)
            {
                buffer[ch][i] = 1.0f;
            }
        }

        graph.prepare();
        graph.process_audio (buffer);

        // Clear the graph explicitly
        graph.clear();

        // Give shared_ptr time to clean up
        std::this_thread::sleep_for (std::chrono::milliseconds (100));

        assert (ResourceTracker::instanceCount == 0 && "All resources should be freed after clear()");

        // Add more nodes after clearing
        graph.addProcessor<TrackedProcessorF32>();
        graph.addProcessor<TrackedProcessorF32>();

        assert (ResourceTracker::instanceCount == 2 && "Should have 2 new resource trackers");
    }

    // Give shared_ptr time to clean up after graph destruction
    std::this_thread::sleep_for (std::chrono::milliseconds (100));

    assert (ResourceTracker::instanceCount == 0 && "All resources should be freed after graph destruction");

    std::cout << "Cleanup and teardown tests passed!" << std::endl;
}

// ======== MAIN TEST RUNNER ========

int main()
{
    std::cout << "Running Audio Graph Tests" << std::endl;
    std::cout << "=========================" << std::endl;

    testNodeCreation();
    testConnectionManagement();
    testRoutingAndBufferHandling();
    testBasicSignalFlow();
    testMultiNodeSignalFlow();
    testTopologyUpdates();
    testConcurrencySafety();
    testChainOfResponsibility();
    testCleanupAndTeardown();

    std::cout << "=========================" << std::endl;
    std::cout << "All audio graph tests passed!" << std::endl;

    return 0;
}
