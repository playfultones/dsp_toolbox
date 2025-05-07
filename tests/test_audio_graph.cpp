#include "dynamics/gain.h"
#include "generators/noisegenerators.h"
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

// Mock processor for testing
class MockProcessor : public Processor
{
public:
    MockProcessor() = default;

    void prepare (double sampleRate, int maxFramesPerBlock) override
    {
        this->sampleRate = sampleRate;
        this->maxBlockSize = maxFramesPerBlock;
        prepared = true;
    }

    void process (BufferView& buffer) override
    {
        processCallCount++;
        lastProcessedChannels = buffer.getNumChannels();
        lastProcessedFrames = buffer.getNumFrames();

        // Optional processing behavior for testing
        if (processingBehavior)
            processingBehavior (buffer);
    }

    void reset() override
    {
        resetCallCount++;
        prepared = false;
    }

    bool wasPrepared() const { return prepared; }
    int getProcessCallCount() const { return processCallCount; }
    int getResetCallCount() const { return resetCallCount; }
    int getLastProcessedChannels() const { return lastProcessedChannels; }
    int getLastProcessedFrames() const { return lastProcessedFrames; }

    // For configuring test behavior
    void setProcessingBehavior (std::function<void (BufferView&)> behavior)
    {
        processingBehavior = std::move (behavior);
    }

private:
    bool prepared = false;
    int processCallCount = 0;
    int resetCallCount = 0;
    int lastProcessedChannels = 0;
    int lastProcessedFrames = 0;
    double sampleRate = 0.0;
    int maxBlockSize = 0;
    std::function<void (BufferView&)> processingBehavior;
};

// Mock processor that can be bypassed
class BypassableProcessor : public MockProcessor
{
public:
    BypassableProcessor() : bypassed (false) {}

    void process (BufferView& buffer) override
    {
        if (!bypassed)
        {
            MockProcessor::process (buffer);
        }
    }

    void setBypassed (bool shouldBypass) { bypassed = shouldBypass; }
    bool isBypassed() const { return bypassed; }

private:
    bool bypassed;
};

// ======== 1. NODE CREATION TESTS ========

void testNodeCreation()
{
    std::cout << "Testing node creation..." << std::endl;

    // Test valid node creation
    GraphBuilder builder;

    // Create node with unique processor
    auto proc1 = std::make_unique<MockProcessor>();
    auto node1 = builder.addNode (std::move (proc1));
    assert (node1 != nullptr && "Node creation should succeed with valid processor");
    assert (node1->getId() > 0 && "Node should have a valid ID");

    // Create node with template method
    auto node2 = builder.addNode<MockProcessor>();
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

    GraphBuilder builder;

    // Create nodes
    auto node1 = builder.addNode<MockProcessor>();
    auto node2 = builder.addNode<MockProcessor>();
    auto node3 = builder.addNode<MockProcessor>();

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

    const int numChannels = 2;
    const int numFrames = 512;

    AudioProcessorGraph graph;

    // Create processors that will modify the buffer in specific ways
    auto gainNode1Id = graph.addProcessor<Gain>();
    auto gainNode2Id = graph.addProcessor<Gain>();

    // Configure gain processors
    if (auto* gain1 = graph.getProcessor<Gain> (gainNode1Id))
        gain1->setGain (0.5f);

    if (auto* gain2 = graph.getProcessor<Gain> (gainNode2Id))
        gain2->setGain (2.0f);

    // Connect in series to create a chain
    graph.connect (gainNode1Id, gainNode2Id);
    graph.connect (gainNode2Id, graph.getOutputNode()->getId());

    // Prepare the graph
    graph.prepare (44100.0, numFrames);

    // Create a buffer and fill with known values
    AudioBuffer buffer (numChannels, numFrames);
    for (int ch = 0; ch < numChannels; ++ch)
    {
        auto* channelData = buffer.getChannelPointer (ch);
        for (int i = 0; i < numFrames; ++i)
        {
            channelData[i] = 1.0f; // Fill with ones
        }
    }

    // Process the buffer
    BufferView view;
    view.setData (buffer.getArrayOfChannels(), numChannels, numFrames);
    graph.process (view);

    // Verify the processing: 1.0 * 0.5 * 2.0 = 1.0
    for (int ch = 0; ch < numChannels; ++ch)
    {
        auto* channelData = buffer.getChannelPointer (ch);
        for (int i = 0; i < numFrames; ++i)
        {
            assert (std::abs (channelData[i] - 1.0f) < 0.00001f && "Buffer values should be 1.0 after processing");
        }
    }

    std::cout << "Routing and buffer handling tests passed!" << std::endl;
}

// ======== 4. BASIC SIGNAL FLOW TEST ========

void testBasicSignalFlow()
{
    std::cout << "Testing basic signal flow..." << std::endl;

    constexpr auto kGain = 0.5f;
    const int numChannels = 2;
    const double sampleRate = 44100.0;
    const int numFrames = static_cast<int> (sampleRate); // 1 second of audio

    // Create audio buffers
    AudioBuffer audioBuffer (numChannels, numFrames);
    AudioBuffer expectedBuffer (numChannels, numFrames);

    // Create audio processor graph
    AudioProcessorGraph graph;

    // Add a gain processor node and get its ID
    auto gainNodeId = graph.addProcessor<Gain>();

    // Get the gain processor and set its gain
    if (auto* gain = graph.getProcessor<Gain> (gainNodeId))
        gain->setGain (kGain);
    else
        throw std::runtime_error ("Failed to get Gain processor");

    // Connect gain to output
    graph.connect (gainNodeId, graph.getOutputNode()->getId());

    // Prepare graph
    graph.prepare (sampleRate, numFrames);

    // Fill buffer with white noise
    generateWhiteNoise (audioBuffer.getArrayOfChannels(), numChannels, numFrames, 1.0f);

    // Copy to expected buffer and apply gain
    for (int ch = 0; ch < numChannels; ++ch)
    {
        const auto* srcData = audioBuffer.getChannelPointer (ch);
        auto* dstData = expectedBuffer.getChannelPointer (ch);
        for (int i = 0; i < numFrames; ++i)
            dstData[i] = srcData[i] * kGain;
    }

    // Make a copy of the original buffer before processing
    AudioBuffer originalBuffer (numChannels, numFrames);
    for (int ch = 0; ch < numChannels; ++ch)
    {
        std::memcpy (originalBuffer.getChannelPointer (ch),
            audioBuffer.getChannelPointer (ch),
            numFrames * sizeof (float));
    }

    // Process using BufferView
    BufferView view;
    view.setData (audioBuffer.getArrayOfChannels(), numChannels, numFrames);
    graph.process (view);

    // Verify gain was applied correctly using the buffer validator
    bool allSamplesCorrect = compareAudioBuffers (audioBuffer, expectedBuffer);

    // Debug output for first few samples if test fails
    if (!allSamplesCorrect)
    {
        std::cerr << "First few samples comparison:" << std::endl;
        for (int i = 0; i < std::min (10, numFrames); ++i)
        {
            std::cerr << "Sample " << i << ": Original=" << originalBuffer.getChannelPointer (0)[i]
                      << ", Expected=" << expectedBuffer.getChannelPointer (0)[i]
                      << ", Got=" << audioBuffer.getChannelPointer (0)[i] << std::endl;
        }
    }

    assert (allSamplesCorrect && "All samples should be exactly half of the original values");
    std::cout << "Basic signal flow test passed for 1 second of audio!" << std::endl;
}

// ======== 5. MULTI-NODE SIGNAL FLOW TEST ========

void testMultiNodeSignalFlow()
{
    std::cout << "Testing multi-node signal flow..." << std::endl;

    const int numChannels = 2;
    const double sampleRate = 44100.0;
    const int numFrames = static_cast<int> (sampleRate); // 1 second of audio

    // Create audio buffers
    AudioBuffer audioBuffer (numChannels, numFrames);
    AudioBuffer expectedBuffer (numChannels, numFrames);

    // Create audio processor graph
    AudioProcessorGraph graph;

    // Add two gain processor nodes in series
    auto gainNode1Id = graph.addProcessor<Gain>();
    auto gainNode2Id = graph.addProcessor<Gain>();

    // Connect the nodes
    graph.connect (gainNode1Id, gainNode2Id);
    graph.connect (gainNode2Id, graph.getOutputNode()->getId());

    // Get the gain processors and set their gains
    if (auto* gain1 = graph.getProcessor<Gain> (gainNode1Id))
        gain1->setGain (0.8f);
    else
        throw std::runtime_error ("Failed to get first Gain processor");

    if (auto* gain2 = graph.getProcessor<Gain> (gainNode2Id))
        gain2->setGain (0.625f);
    else
        throw std::runtime_error ("Failed to get second Gain processor");

    // Prepare graph
    graph.prepare (sampleRate, numFrames);

    // Fill buffer with white noise
    generateWhiteNoise (audioBuffer.getArrayOfChannels(), numChannels, numFrames, 1.0f);

    // Copy to expected buffer and apply both gains (0.8 * 0.625 = 0.5)
    for (int ch = 0; ch < numChannels; ++ch)
    {
        const auto* srcData = audioBuffer.getChannelPointer (ch);
        auto* dstData = expectedBuffer.getChannelPointer (ch);
        for (int i = 0; i < numFrames; ++i)
            dstData[i] = srcData[i] * 0.8f * 0.625f;
    }

    // Process using BufferView
    BufferView view;
    view.setData (audioBuffer.getArrayOfChannels(), numChannels, numFrames);
    graph.process (view);

    // Verify gain was applied correctly
    bool allSamplesCorrect = compareAudioBuffers (audioBuffer, expectedBuffer);
    assert (allSamplesCorrect && "Samples should have both gains applied");
    std::cout << "Multi-node signal flow test passed!" << std::endl;
}

// ======== 6. TOPOLOGY UPDATES TESTS ========

void testTopologyUpdates()
{
    std::cout << "Testing topology updates..." << std::endl;

    AudioProcessorGraph graph;

    // Create a more complex graph
    auto nodeA = graph.addProcessor<MockProcessor>();
    auto nodeB = graph.addProcessor<MockProcessor>();
    auto nodeC = graph.addProcessor<MockProcessor>();
    auto nodeD = graph.addProcessor<MockProcessor>();

    // Set up initial connections: A->B->D->Output, A->C->D
    graph.connect (nodeA, nodeB);
    graph.connect (nodeB, nodeD);
    graph.connect (nodeA, nodeC);
    graph.connect (nodeC, nodeD);
    graph.connect (nodeD, graph.getOutputNode()->getId());

    // Prepare and check processing
    const int numChannels = 1;
    const int numFrames = 256;
    graph.prepare (44100.0, numFrames);

    AudioBuffer buffer (numChannels, numFrames);
    BufferView view;
    view.setData (buffer.getArrayOfChannels(), numChannels, numFrames);

    // Process initial graph
    graph.process (view);

    // Verify each processor was called exactly once
    auto* procA = graph.getProcessor<MockProcessor> (nodeA);
    auto* procB = graph.getProcessor<MockProcessor> (nodeB);
    auto* procC = graph.getProcessor<MockProcessor> (nodeC);
    auto* procD = graph.getProcessor<MockProcessor> (nodeD);

    assert (procA->getProcessCallCount() == 1 && "ProcessorA should be called once");
    assert (procB->getProcessCallCount() == 1 && "ProcessorB should be called once");
    assert (procC->getProcessCallCount() == 1 && "ProcessorC should be called once");
    assert (procD->getProcessCallCount() == 1 && "ProcessorD should be called once");

    // Change topology: Remove B and connect A directly to D
    graph.disconnect (nodeA, nodeB);
    graph.disconnect (nodeB, nodeD);
    graph.connect (nodeA, nodeD);

    // Process with new topology
    graph.process (view);

    // Verify call counts after topology change
    assert (procA->getProcessCallCount() == 2 && "ProcessorA should be called again");
    assert (procB->getProcessCallCount() == 1 && "ProcessorB should not be called again");
    assert (procC->getProcessCallCount() == 2 && "ProcessorC should be called again");
    assert (procD->getProcessCallCount() == 2 && "ProcessorD should be called again");

    std::cout << "Topology updates tests passed!" << std::endl;
}

// ======== 5. CONCURRENCY SAFETY TESTS ========

void testConcurrencySafety()
{
    std::cout << "Testing concurrency safety..." << std::endl;

    AudioProcessorGraph graph;

    // Create initial nodes
    auto nodeA = graph.addProcessor<MockProcessor>();
    auto nodeB = graph.addProcessor<MockProcessor>();

    // Connect to output
    graph.connect (nodeA, nodeB);
    graph.connect (nodeB, graph.getOutputNode()->getId());

    // Prepare graph
    const int numChannels = 2;
    const int numFrames = 256;
    graph.prepare (44100.0, numFrames);

    // Create buffer for processing
    AudioBuffer buffer (numChannels, numFrames);
    BufferView view;
    view.setData (buffer.getArrayOfChannels(), numChannels, numFrames);

    // Flag to stop audio thread
    std::atomic<bool> shouldStop { false };

    // Audio processing thread
    std::thread audioThread ([&]() {
        while (!shouldStop)
        {
            graph.process (view);
            // Simulate audio thread timing
            std::this_thread::sleep_for (std::chrono::milliseconds (5));
        }
    });

    // Main thread: make graph modifications while audio thread is running
    for (int i = 0; i < 20; i++)
    {
        // Add new node
        auto nodeC = graph.addProcessor<MockProcessor>();

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

// ======== 6. CHAIN OF RESPONSIBILITY WORKFLOW TESTS ========

void testChainOfResponsibility()
{
    std::cout << "Testing chain of responsibility workflow..." << std::endl;

    AudioProcessorGraph graph;

    // Create processors with specific behaviors
    auto nodeA = graph.addProcessor<BypassableProcessor>();
    auto nodeB = graph.addProcessor<MockProcessor>();
    auto nodeC = graph.addProcessor<BypassableProcessor>();

    // Set up processing behaviors
    auto* procA = graph.getProcessor<BypassableProcessor> (nodeA);
    auto* procB = graph.getProcessor<MockProcessor> (nodeB);
    auto* procC = graph.getProcessor<BypassableProcessor> (nodeC);

    // Set up a processing chain: A->B->C->Output
    graph.connect (nodeA, nodeB);
    graph.connect (nodeB, nodeC);
    graph.connect (nodeC, graph.getOutputNode()->getId());

    // Prepare the graph
    const int numChannels = 1;
    const int numFrames = 256;
    graph.prepare (44100.0, numFrames);

    // Create a test buffer
    AudioBuffer buffer (numChannels, numFrames);
    // Fill with known values
    for (int i = 0; i < numFrames; ++i)
    {
        buffer.getChannelPointer (0)[i] = 1.0f;
    }

    // Set up the buffer view
    BufferView view;
    view.setData (buffer.getArrayOfChannels(), numChannels, numFrames);

    // Test 1: All processors active
    procA->setBypassed (false);
    procC->setBypassed (false);

    graph.process (view);

    assert (procA->getProcessCallCount() == 1 && "ProcessorA should be called");
    assert (procB->getProcessCallCount() == 1 && "ProcessorB should be called");
    assert (procC->getProcessCallCount() == 1 && "ProcessorC should be called");

    // Test 2: A bypassed
    procA->setBypassed (true);

    graph.process (view);

    assert (procA->getProcessCallCount() == 1 && "Bypassed ProcessorA should not be called");
    assert (procB->getProcessCallCount() == 2 && "ProcessorB should be called again");
    assert (procC->getProcessCallCount() == 2 && "ProcessorC should be called again");

    // Test 3: A and C bypassed
    procC->setBypassed (true);

    graph.process (view);

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

// ======== 7. CLEANUP AND TEARDOWN TESTS ========

void testCleanupAndTeardown()
{
    std::cout << "Testing cleanup and teardown..." << std::endl;

    class TrackedProcessor : public MockProcessor
    {
    private:
        std::shared_ptr<ResourceTracker> tracker = std::make_shared<ResourceTracker>();
    };

    // Test graph scope and cleanup
    {
        AudioProcessorGraph graph;

        // Add processors that use resources
        for (int i = 0; i < 10; i++)
        {
            graph.addProcessor<TrackedProcessor>();
        }

        // Force some nodes to be connected
        auto node1 = graph.addProcessor<TrackedProcessor>();
        auto node2 = graph.addProcessor<TrackedProcessor>();
        graph.connect (node1, node2);
        graph.connect (node2, graph.getOutputNode()->getId());

        assert (ResourceTracker::instanceCount == 12 && "Should have 12 resource trackers");

        // Process some audio
        const int numChannels = 2;
        const int numFrames = 256;
        AudioBuffer buffer (numChannels, numFrames);
        BufferView view;
        view.setData (buffer.getArrayOfChannels(), numChannels, numFrames);

        graph.prepare (44100.0, numFrames);
        graph.process (view);

        // Clear the graph explicitly
        graph.clear();

        // Give shared_ptr time to clean up
        std::this_thread::sleep_for (std::chrono::milliseconds (100));

        assert (ResourceTracker::instanceCount == 0 && "All resources should be freed after clear()");

        // Add more nodes after clearing
        graph.addProcessor<TrackedProcessor>();
        graph.addProcessor<TrackedProcessor>();

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
