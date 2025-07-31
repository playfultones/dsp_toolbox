/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once
#include "audio_graph.h"
#include <memory>

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief High-level template-based audio processor graph
     * 
     * This class provides a simplified interface to the AudioGraph with
     * compile-time optimization for maximum performance.
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
    class AudioProcessorGraph : public ProcessorBase<AudioProcessorGraph<SampleType, BlockSize, SampleRate, NumChannels>,
                                                     SampleType, BlockSize, SampleRate, NumChannels>
    {
    public:
        using Base = ProcessorBase<AudioProcessorGraph, SampleType, BlockSize, SampleRate, NumChannels>;
        using AudioBuffer = typename Base::AudioBuffer;
        using GraphType = AudioGraph<SampleType, BlockSize, SampleRate, NumChannels>;
        using NodeId = typename GraphType::NodeId;
        using NodePtr = typename GraphType::NodePtr;
        
        AudioProcessorGraph() = default;

        /**
         * @brief Process audio through the graph (hot path)
         * @param buffer The audio buffer to process
         */
        void process_audio_impl(AudioBuffer& buffer) noexcept
        {
            graph_.process_audio(buffer);
        }

        /**
         * @brief Process control-rate updates (cold path)
         */
        void process_control_impl() noexcept
        {
            graph_.process_control();
        }

        /**
         * @brief Prepare the graph for processing
         */
        void prepare_impl() noexcept
        {
            graph_.prepare();
        }

        /**
         * @brief Reset all nodes in the graph
         */
        void reset_impl() noexcept
        {
            graph_.reset();
        }

        /**
         * @brief Add a processor node to the graph
         * @tparam ProcessorType The CRTP processor type
         * @tparam Args Constructor argument types
         * @param args Constructor arguments for the processor
         * @return ID of the created node
         */
        template<Processor ProcessorType, typename... Args>
        NodeId addProcessor(Args&&... args)
        {
            auto& builder = graph_.getBuilder();
            auto node = builder.template addNode<ProcessorType>(std::forward<Args>(args)...);
            NodeId id = node->getId();
            graph_.applyChanges();
            return id;
        }

        /**
         * @brief Connect two processor nodes
         * @param sourceId ID of the source node
         * @param destinationId ID of the destination node
         * @param sourceChannel Output channel on the source node
         * @param destinationChannel Input channel on the destination node
         * @return True if the connection was successful
         */
        bool connect(NodeId sourceId, NodeId destinationId, int sourceChannel = 0, int destinationChannel = 0)
        {
            auto& builder = graph_.getBuilder();
            bool result = builder.connect(sourceId, destinationId, sourceChannel, destinationChannel);
            graph_.applyChanges();
            return result;
        }

        /**
         * @brief Disconnect two processor nodes
         * @param sourceId ID of the source node
         * @param destinationId ID of the destination node
         * @param sourceChannel Output channel on the source node
         * @param destinationChannel Input channel on the destination node
         * @return True if the disconnection was successful
         */
        bool disconnect(NodeId sourceId, NodeId destinationId, int sourceChannel = 0, int destinationChannel = 0)
        {
            auto& builder = graph_.getBuilder();
            bool result = builder.disconnect(sourceId, destinationId, sourceChannel, destinationChannel);
            graph_.applyChanges();
            return result;
        }

        /**
         * @brief Remove a processor node from the graph
         * @param id ID of the node to remove
         * @return True if the node was found and removed
         */
        bool removeProcessor(NodeId id)
        {
            auto& builder = graph_.getBuilder();
            bool result = builder.removeNode(id);
            graph_.applyChanges();
            return result;
        }

        /**
         * @brief Clear all nodes from the graph
         */
        void clear()
        {
            graph_.clear();
        }

        /**
         * @brief Get a processor by its ID
         * @tparam ProcessorType The processor type to get
         * @param id The ID of the node
         * @return Pointer to the processor, or nullptr if not found or wrong type
         */
        template<Processor ProcessorType>
        ProcessorType* getProcessor(NodeId id)
        {
            auto& builder = graph_.getBuilder();
            auto node = builder.getNode(id);
            if (!node)
                return nullptr;

            auto* wrapper = node->template getProcessorWrapper<ProcessorType>();
            return wrapper ? &wrapper->processor() : nullptr;
        }

        /**
         * @brief Get the output node of the graph
         * @return The output node pointer
         */
        NodePtr getOutputNode()
        {
            return graph_.getBuilder().getOutputNode();
        }

        /**
         * @brief Get compile-time configuration
         */
        static constexpr size_t getBlockSize() { return BlockSize; }
        static constexpr size_t getSampleRate() { return SampleRate; }
        static constexpr size_t getNumChannels() { return NumChannels; }

    private:
        GraphType graph_;
    };
    
    // Common type aliases
    using AudioProcessorGraphF32 = AudioProcessorGraph<float, 512, 44100, 2>;
    using AudioProcessorGraphF64 = AudioProcessorGraph<double, 512, 44100, 2>;
}
