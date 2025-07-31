/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once
#include "../processor.h"
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <concepts>
#include <span>
#include <array>

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief High-performance audio processing graph node
     * 
     * Template-based node that wraps processors for compile-time optimization.
     * All configuration is done at compile time for maximum performance.
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
    class AudioGraphNode
    {
    public:
        using Ptr = std::shared_ptr<AudioGraphNode>;
        using WeakPtr = std::weak_ptr<AudioGraphNode>;
        using Id = unsigned int;
        using sample_type = SampleType;
        
        // Compile-time constants
        static constexpr size_t block_size = BlockSize;
        static constexpr size_t sample_rate = SampleRate;
        static constexpr size_t num_channels = NumChannels;
        
        // Buffer types
        using ChannelBuffer = std::span<sample_type, block_size>;
        using AudioBuffer = std::array<ChannelBuffer, num_channels>;

    private:
        struct Connection
        {
            WeakPtr node;
            int outputChannel;
        };

    public:
        /**
         * @brief Construct a new AudioGraphNode with type-erased processor
         * @param processor The processor interface to wrap
         */
        AudioGraphNode(std::unique_ptr<ProcessorInterface> processor)
            : processor_(std::move(processor)), id_(nextId++)
        {
        }

        /**
         * @brief Get the unique ID of this node
         */
        Id getId() const { return id_; }

        /**
         * @brief Get the wrapped processor interface
         */
        ProcessorInterface* getProcessor() { return processor_.get(); }

        /**
         * @brief Get the wrapped processor as a specific wrapper type
         * @tparam ProcessorType The actual processor type
         * @return Pointer to the processor wrapper, or nullptr if the cast fails
         */
        template<Processor ProcessorType>
        ProcessorWrapper<ProcessorType>* getProcessorWrapper()
        {
            return dynamic_cast<ProcessorWrapper<ProcessorType>*>(processor_.get());
        }

        /**
         * @brief Add an input connection from another node
         * @param sourceNode The source node
         * @param outputChannel The output channel on the source node
         * @param inputChannel The input channel on this node
         * @return True if the connection was successful
         */
        bool addInputConnection(Ptr sourceNode, int outputChannel = 0, int inputChannel = 0)
        {
            if (!sourceNode)
                return false;

            inputs_[inputChannel].push_back({sourceNode, outputChannel});
            return true;
        }

        /**
         * @brief Remove an input connection
         * @param sourceNode The source node to disconnect
         * @param outputChannel The output channel on the source node
         * @param inputChannel The input channel on this node
         * @return True if a connection was removed
         */
        bool removeInputConnection(Ptr sourceNode, int outputChannel = 0, int inputChannel = 0)
        {
            if (!sourceNode)
                return false;

            auto& connections = inputs_[inputChannel];
            for (auto it = connections.begin(); it != connections.end(); ++it)
            {
                if (it->node.lock() == sourceNode && it->outputChannel == outputChannel)
                {
                    connections.erase(it);
                    return true;
                }
            }
            return false;
        }

        /**
         * @brief Clear all input connections
         */
        void clearInputConnections()
        {
            inputs_.clear();
        }

        /**
         * @brief Get all input connections for this node
         * @return Map of input channels to connections
         */
        const std::unordered_map<int, std::vector<Connection>>& getInputConnections() const
        {
            return inputs_;
        }

        /**
         * @brief Process audio through this node and its inputs (hot path)
         * @param buffer The audio buffer to process
         * @param processedNodes Set of nodes that have already been processed
         */
        void processAudio(AudioBuffer& buffer, std::unordered_map<Id, bool>& processedNodes) noexcept
        {
            // Prevent cycles by checking if this node has already been processed
            if (processedNodes[id_])
                return;

            // Process all inputs first
            for (auto& [channel, connections] : inputs_)
            {
                for (auto& connection : connections)
                {
                    if (auto sourceNode = connection.node.lock())
                    {
                        sourceNode->processAudio(buffer, processedNodes);
                    }
                }
            }

            // Process this node (hot path - no virtual calls in CRTP processors)
            processor_->process_audio_erased(&buffer);

            // Mark this node as processed
            processedNodes[id_] = true;
        }

        /**
         * @brief Process control-rate updates (cold path)
         */
        void processControl() noexcept
        {
            processor_->process_control();
        }

        /**
         * @brief Prepare the node for processing
         */
        void prepare() noexcept
        {
            processor_->prepare();
        }

        /**
         * @brief Reset the node
         */
        void reset() noexcept
        {
            processor_->reset();
        }

        /**
         * @brief Get processor configuration
         */
        size_t getBlockSize() const { return processor_->get_block_size(); }
        size_t getSampleRate() const { return processor_->get_sample_rate(); }
        size_t getNumChannels() const { return processor_->get_num_channels(); }

    private:
        std::unique_ptr<ProcessorInterface> processor_;
        Id id_;
        std::unordered_map<int, std::vector<Connection>> inputs_;

        // Static counter for auto-generating IDs
        static inline Id nextId = 1;
    };
    
    // Common type aliases
    using AudioGraphNodeF32 = AudioGraphNode<float, 512, 44100, 2>;
    using AudioGraphNodeF64 = AudioGraphNode<double, 512, 44100, 2>;
}
