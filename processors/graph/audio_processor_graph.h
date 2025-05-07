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
     * @brief High-level audio processor graph for easy DSP chain creation
     * 
     * This class provides a simplified interface to the AudioGraph for common use cases.
     */
    class AudioProcessorGraph : public Processor
    {
    public:
        AudioProcessorGraph() = default;

        /**
         * @brief Process audio through the graph
         * @param buffer The audio buffer to process
         */
        void process (BufferView& buffer) override
        {
            graph.process (buffer);
        }

        /**
         * @brief Prepare the graph for processing
         * @param sampleRate The sample rate
         * @param maxFramesPerBlock The maximum number of frames per block
         */
        void prepare (double sampleRate, int maxFramesPerBlock) override
        {
            graph.prepare (sampleRate, maxFramesPerBlock);
        }

        /**
         * @brief Reset all nodes in the graph
         */
        void reset() override
        {
            graph.reset();
        }

        /**
         * @brief Add a processor node to the graph
         * @param processor The processor to add
         * @return ID of the created node
         */
        AudioGraphNode::Id addProcessor (std::unique_ptr<Processor> processor)
        {
            auto& builder = graph.getBuilder();
            auto node = builder.addNode (std::move (processor));
            AudioGraphNode::Id id = node->getId();
            graph.applyChanges();
            return id;
        }

        /**
         * @brief Construct and add a processor node to the graph
         * @tparam T The processor type
         * @tparam Args Constructor argument types
         * @param args Constructor arguments for the processor
         * @return ID of the created node
         */
        template <typename T, typename... Args>
        AudioGraphNode::Id addProcessor (Args&&... args)
        {
            auto& builder = graph.getBuilder();
            auto node = builder.addNode<T> (std::forward<Args> (args)...);
            AudioGraphNode::Id id = node->getId();
            graph.applyChanges();
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
        bool connect (AudioGraphNode::Id sourceId, AudioGraphNode::Id destinationId, int sourceChannel = 0, int destinationChannel = 0)
        {
            auto& builder = graph.getBuilder();
            bool result = builder.connect (sourceId, destinationId, sourceChannel, destinationChannel);
            graph.applyChanges();
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
        bool disconnect (AudioGraphNode::Id sourceId, AudioGraphNode::Id destinationId, int sourceChannel = 0, int destinationChannel = 0)
        {
            auto& builder = graph.getBuilder();
            bool result = builder.disconnect (sourceId, destinationId, sourceChannel, destinationChannel);
            graph.applyChanges();
            return result;
        }

        /**
         * @brief Remove a processor node from the graph
         * @param id ID of the node to remove
         * @return True if the node was found and removed
         */
        bool removeProcessor (AudioGraphNode::Id id)
        {
            auto& builder = graph.getBuilder();
            bool result = builder.removeNode (id);
            graph.applyChanges();
            return result;
        }

        /**
         * @brief Clear all nodes from the graph
         */
        void clear()
        {
            graph.clear();
        }

        /**
         * @brief Get a processor by its ID
         * @tparam T The processor type to cast to
         * @param id The ID of the node
         * @return Pointer to the processor, or nullptr if not found or wrong type
         */
        template <typename T>
        T* getProcessor (AudioGraphNode::Id id)
        {
            auto& builder = graph.getBuilder();
            auto node = builder.getNode (id);
            if (!node)
                return nullptr;

            return dynamic_cast<T*> (node->getProcessor());
        }

        /**
         * @brief Get the output node of the graph
         * @return The output node pointer
         */
        AudioGraphNode::Ptr getOutputNode()
        {
            return graph.getBuilder().getOutputNode();
        }

    private:
        AudioGraph graph;
    };
}
