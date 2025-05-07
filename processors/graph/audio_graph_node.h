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

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief A node in the audio processing graph
     * 
     * Each node wraps a Processor and can be connected to other nodes 
     * via input/output channels. Each node has a unique ID.
     */
    class AudioGraphNode
    {
    public:
        using Ptr = std::shared_ptr<AudioGraphNode>;
        using WeakPtr = std::weak_ptr<AudioGraphNode>;
        using Id = unsigned int;

    private:
        struct Connection
        {
            WeakPtr node;
            int outputChannel;
        };

    public:
        /**
         * @brief Construct a new AudioGraphNode
         * @param processor The processor to wrap
         */
        AudioGraphNode (std::unique_ptr<Processor> processor)
            : processor (std::move (processor)), id (nextId++)
        {
        }

        /**
         * @brief Get the unique ID of this node
         */
        Id getId() const { return id; }

        /**
         * @brief Get the wrapped processor
         */
        Processor* getProcessor() { return processor.get(); }

        /**
         * @brief Get the wrapped processor as a specific type
         * @tparam T The type to cast the processor to
         * @return Pointer to the processor as type T, or nullptr if the cast fails
         */
        template <typename T>
        T* getProcessor()
        {
            return dynamic_cast<T*> (processor.get());
        }

        /**
         * @brief Add an input connection from another node
         * @param sourceNode The source node
         * @param outputChannel The output channel on the source node
         * @param inputChannel The input channel on this node
         * @return True if the connection was successful
         */
        bool addInputConnection (Ptr sourceNode, int outputChannel = 0, int inputChannel = 0)
        {
            if (!sourceNode)
                return false;

            inputs[inputChannel].push_back ({ sourceNode, outputChannel });
            return true;
        }

        /**
         * @brief Remove an input connection
         * @param sourceNode The source node to disconnect
         * @param outputChannel The output channel on the source node
         * @param inputChannel The input channel on this node
         * @return True if a connection was removed
         */
        bool removeInputConnection (Ptr sourceNode, int outputChannel = 0, int inputChannel = 0)
        {
            if (!sourceNode)
                return false;

            auto& connections = inputs[inputChannel];
            for (auto it = connections.begin(); it != connections.end(); ++it)
            {
                if (it->node.lock() == sourceNode && it->outputChannel == outputChannel)
                {
                    connections.erase (it);
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
            inputs.clear();
        }

        /**
         * @brief Get all input connections for this node
         * @return Map of input channels to connections
         */
        const std::unordered_map<int, std::vector<Connection>>& getInputConnections() const
        {
            return inputs;
        }

        /**
         * @brief Process audio through this node and its inputs
         * @param buffer The audio buffer to process
         * @param processedNodes Set of nodes that have already been processed
         */
        void process (BufferView& buffer, std::unordered_map<Id, bool>& processedNodes)
        {
            // Prevent cycles by checking if this node has already been processed
            if (processedNodes[id])
                return;

            // Process all inputs first
            for (auto& [channel, connections] : inputs)
            {
                for (auto& connection : connections)
                {
                    if (auto sourceNode = connection.node.lock())
                    {
                        sourceNode->process (buffer, processedNodes);
                    }
                }
            }

            // Process this node
            processor->process (buffer);

            // Mark this node as processed
            processedNodes[id] = true;
        }

        /**
         * @brief Prepare the node for processing
         * @param sampleRate The sample rate
         * @param maxFramesPerBlock The maximum number of frames per block
         */
        void prepare (double sampleRate, int maxFramesPerBlock)
        {
            processor->prepare (sampleRate, maxFramesPerBlock);
        }

        /**
         * @brief Reset the node
         */
        void reset()
        {
            processor->reset();
        }

    private:
        std::unique_ptr<Processor> processor;
        Id id;
        std::unordered_map<int, std::vector<Connection>> inputs;

        // Static counter for auto-generating IDs
        static inline Id nextId = 1;
    };
}
