/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once
#include "../processor.h"
#include "audio_graph_node.h"
#include "graph_builder.h"
#include "output_processor.h"
#include <atomic>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <span>
#include <array>

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief High-performance template-based audio processing graph
     * 
     * This class represents the audio processing graph with compile-time optimization
     * and lock-free updates for maximum real-time performance.
     * 
     * @tparam SampleType The sample type (float, double)
     * @tparam BlockSize Fixed block size for processing
     * @tparam SampleRate Sample rate
     * @tparam NumChannels Number of audio channels
     */
    template<typename SampleType = float, 
             size_t BlockSize = 512, 
             size_t SampleRate = 44100,
             size_t NumChannels = 2>
    class AudioGraph : public ProcessorBase<AudioGraph<SampleType, BlockSize, SampleRate, NumChannels>,
                                            SampleType, BlockSize, SampleRate, NumChannels>
    {
    public:
        using Base = ProcessorBase<AudioGraph, SampleType, BlockSize, SampleRate, NumChannels>;
        using AudioBuffer = typename Base::AudioBuffer;
        using BuilderType = GraphBuilder<SampleType, BlockSize, SampleRate, NumChannels>;
        using NodeType = AudioGraphNode<SampleType, BlockSize, SampleRate, NumChannels>;
        using NodePtr = typename NodeType::Ptr;
        using NodeId = typename NodeType::Id;
        
        AudioGraph() = default;

        /**
         * @brief Process audio through the graph (hot path)
         * @param buffer The audio buffer to process
         */
        void process_audio_impl(AudioBuffer& buffer) noexcept
        {
            // Get current render sequence - this is lock-free
            RenderSequence* sequence = currentSequence_.load(std::memory_order_acquire);

            if (sequence && !sequence->empty())
            {
                // Process nodes in sequence order (optimized path)
                // Each node only processes itself, not the entire chain
                for (const auto& node : *sequence)
                {
                    if (node && node->getProcessor())
                    {
                        node->getProcessor()->process_audio_erased(&buffer);
                    }
                }
            }
            else
            {
                // Fall back to graph traversal if no sequence is available
                std::unordered_map<NodeId, bool> processedNodes;
                if (auto outputNode = builder_.getOutputNode())
                {
                    outputNode->processAudio(buffer, processedNodes);
                }
            }
        }

        /**
         * @brief Process control-rate updates (cold path)
         */
        void process_control_impl() noexcept
        {
            std::lock_guard<std::mutex> lock(builderMutex_);
            builder_.processControlAll();
        }

        /**
         * @brief Prepare the graph for processing
         */
        void prepare_impl() noexcept
        {
            std::lock_guard<std::mutex> lock(builderMutex_);
            builder_.prepareAll();
        }

        /**
         * @brief Reset all nodes in the graph
         */
        void reset_impl() noexcept
        {
            std::lock_guard<std::mutex> lock(builderMutex_);
            builder_.resetAll();
        }

        /**
         * @brief Get the graph builder for making modifications
         * @return Reference to the graph builder
         * @note Always acquire the builder through this method to ensure thread safety
         */
        BuilderType& getBuilder()
        {
            return builder_;
        }

        /**
         * @brief Apply changes made to the builder to the audio processing graph
         */
        void applyChanges()
        {
            std::lock_guard<std::mutex> lock(builderMutex_);

            // Prepare any new nodes that might have been added
            builder_.prepareAll();

            RenderSequence* currentActiveSequence = currentSequence_.load(std::memory_order_acquire);

            // Determine which buffer to update (the one not currently in use)
            std::unique_ptr<RenderSequence>& inactiveBuffer =
                (currentActiveSequence == sequenceA_.get()) ? sequenceB_ : sequenceA_;

            // Create a new sequence in the inactive buffer
            inactiveBuffer = std::make_unique<RenderSequence>(builder_.createRenderSequence());

            // Atomically swap the current sequence pointer
            currentSequence_.store(inactiveBuffer.get(), std::memory_order_release);
        }

        /**
         * @brief Explicitly clear all resources in the graph
         */
        void clear()
        {
            std::lock_guard<std::mutex> lock(builderMutex_);

            // Clear the builder
            builder_.clear();

            // Reset sequence pointers
            sequenceA_.reset();
            sequenceB_.reset();
            currentSequence_.store(nullptr, std::memory_order_release);
        }

        /**
         * @brief Get compile-time configuration
         */
        static constexpr size_t getBlockSize() noexcept { return BlockSize; }
        static constexpr size_t getSampleRate() noexcept { return SampleRate; }
        static constexpr size_t getNumChannels() noexcept { return NumChannels; }

    private:
        using RenderSequence = std::vector<NodePtr>;

        BuilderType builder_;
        std::atomic<RenderSequence*> currentSequence_{nullptr};
        static_assert(std::atomic<RenderSequence*>::is_always_lock_free,
            "std::atomic<RenderSequence*> must be lock-free for real-time audio processing");

        // Double-buffered render sequences for lock-free updates
        std::unique_ptr<RenderSequence> sequenceA_;
        std::unique_ptr<RenderSequence> sequenceB_;

        std::mutex builderMutex_; // Only used when modifying the graph structure
    };
    
    // Common type aliases
    using AudioGraphF32 = AudioGraph<float, 512, 44100, 2>;
    using AudioGraphF64 = AudioGraph<double, 512, 44100, 2>;
}
