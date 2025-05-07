#pragma once
#include "audio_graph_node.h"
#include "graph_builder.h"
#include "output_processor.h"
#include "../processor.h"
#include <atomic>
#include <memory>
#include <mutex>
#include <unordered_map>

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief Main audio processing graph with lock-free updates
     * 
     * This class represents the audio processing graph and handles
     * thread-safe, lock-free updates to the processing chain.
     */
    class AudioGraph : public Processor
    {
    public:
        AudioGraph() = default;
        
        /**
         * @brief Process audio through the graph
         * @param buffer The audio buffer to process
         */
        void process(BufferView& buffer) override
        {
            // Get current render sequence - this is lock-free
            RenderSequence* sequence = currentSequence.load(std::memory_order_acquire);
            
            if (sequence && !sequence->empty())
            {
                // Process all nodes in the sequence
                for (const auto& node : *sequence)
                {
                    if (node && node->getProcessor())
                        node->getProcessor()->process(buffer);
                }
            }
            else
            {
                // Fall back to graph traversal if no sequence is available
                std::unordered_map<AudioGraphNode::Id, bool> processedNodes;
                builder.getOutputNode()->process(buffer, processedNodes);
            }
        }
        
        /**
         * @brief Prepare the graph for processing
         * @param sampleRate The sample rate
         * @param maxFramesPerBlock The maximum number of frames per block
         */
        void prepare(double sampleRate, int maxFramesPerBlock) override
        {
            // Store these values for future builder updates
            this->sampleRate = sampleRate;
            this->maxFramesPerBlock = maxFramesPerBlock;
            
            std::lock_guard<std::mutex> lock(builderMutex);
            builder.prepareAll(sampleRate, maxFramesPerBlock);
        }
        
        /**
         * @brief Reset all nodes in the graph
         */
        void reset() override
        {
            std::lock_guard<std::mutex> lock(builderMutex);
            builder.resetAll();
        }
        
        /**
         * @brief Get the graph builder for making modifications
         * @return Reference to the graph builder
         * @note Always acquire the builder through this method to ensure thread safety
         */
        GraphBuilder& getBuilder()
        {
            return builder;
        }
        
        /**
         * @brief Apply changes made to the builder to the audio processing graph
         */
        void applyChanges()
        {
            std::lock_guard<std::mutex> lock(builderMutex);
            
            // Prepare any new nodes that might have been added
            builder.prepareAll(sampleRate, maxFramesPerBlock);
            
            RenderSequence* currentActiveSequence = currentSequence.load(std::memory_order_acquire);
            
            // Determine which buffer to update (the one not currently in use)
            std::unique_ptr<RenderSequence>& inactiveBuffer = 
                (currentActiveSequence == sequenceA.get()) ? sequenceB : sequenceA;
            
            // Create a new sequence in the inactive buffer
            inactiveBuffer = std::make_unique<RenderSequence>(builder.createRenderSequence());
            
            // Atomically swap the current sequence pointer
            currentSequence.store(inactiveBuffer.get(), std::memory_order_release);
        }
        
        /**
         * @brief Explicitly clear all resources in the graph
         */
        void clear()
        {
            std::lock_guard<std::mutex> lock(builderMutex);
            
            // Clear the builder
            builder.clear();
            
            // Reset sequence pointers
            sequenceA.reset();
            sequenceB.reset();
            currentSequence.store(nullptr, std::memory_order_release);
        }
        
    private:
        using RenderSequence = std::vector<AudioGraphNode::Ptr>;
        
        GraphBuilder builder;
        std::atomic<RenderSequence*> currentSequence{nullptr};                
        static_assert(std::atomic<RenderSequence*>::is_always_lock_free,
            "std::atomic<RenderSequence*> must be lock-free for real-time audio processing");
        
        // Buffers for the render sequence.
        // These are used to avoid lock contention when applying changes
        // to the graph while processing.
        // The current sequence is always in use, while the other is being built.
        std::unique_ptr<RenderSequence> sequenceA;
        std::unique_ptr<RenderSequence> sequenceB;
        
        std::mutex builderMutex;  // Only used when modifying the graph structure
        
        double sampleRate = 44100.0;
        int maxFramesPerBlock = 512;
    };
}
