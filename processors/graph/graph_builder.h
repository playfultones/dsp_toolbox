/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once
#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "audio_graph_node.h"
#include "output_processor.h"

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief Builds and manages an audio processing graph
     * 
     * This class handles the creation and connection of nodes in the audio graph,
     * as well as topological sorting to create a proper processing sequence.
     */
    class GraphBuilder
    {
    public:
        GraphBuilder()
        {
            createOutputNode();
        }

        /**
         * @brief Adds a node to the graph
         * @param processor The processor to wrap in a node
         * @return Shared pointer to the created node
         */
        AudioGraphNode::Ptr addNode (std::unique_ptr<Processor> processor)
        {
            auto node = std::make_shared<AudioGraphNode> (std::move (processor));
            AudioGraphNode::Id id = node->getId();
            nodes[id] = node;
            return node;
        }

        /**
         * @brief Template version of addNode that constructs the processor in-place
         * @tparam T The processor type
         * @tparam Args Constructor argument types
         * @param args Constructor arguments for the processor
         * @return Shared pointer to the created node
         */
        template <typename T, typename... Args>
        AudioGraphNode::Ptr addNode (Args&&... args)
        {
            auto processor = std::make_unique<T> (std::forward<Args> (args)...);
            return addNode (std::move (processor));
        }

        /**
         * @brief Removes a node from the graph
         * @param id The ID of the node to remove
         * @return True if the node was found and removed
         */
        bool removeNode (AudioGraphNode::Id id)
        {
            return nodes.erase (id) > 0;
        }

        /**
         * @brief Connects two nodes in the graph
         * @param sourceId The ID of the source node
         * @param targetId The ID of the target node
         * @param outputChannel The output channel on the source node
         * @param inputChannel The input channel on the target node
         * @return True if the connection was successful
         */
        bool connect (AudioGraphNode::Id sourceId, AudioGraphNode::Id targetId, int outputChannel = 0, int inputChannel = 0)
        {
            auto sourceIt = nodes.find (sourceId);
            auto targetIt = nodes.find (targetId);

            if (sourceIt == nodes.end() || targetIt == nodes.end())
                return false;

            return targetIt->second->addInputConnection (sourceIt->second, outputChannel, inputChannel);
        }

        /**
         * @brief Disconnects two nodes in the graph
         * @param sourceId The ID of the source node
         * @param targetId The ID of the target node
         * @param outputChannel The output channel on the source node
         * @param inputChannel The input channel on the target node
         * @return True if the disconnection was successful
         */
        bool disconnect (AudioGraphNode::Id sourceId, AudioGraphNode::Id targetId, int outputChannel = 0, int inputChannel = 0)
        {
            auto sourceIt = nodes.find (sourceId);
            auto targetIt = nodes.find (targetId);

            if (sourceIt == nodes.end() || targetIt == nodes.end())
                return false;

            return targetIt->second->removeInputConnection (sourceIt->second, outputChannel, inputChannel);
        }

        /**
         * @brief Gets a node by its ID
         * @param id The ID of the node to get
         * @return Shared pointer to the node, or nullptr if not found
         */
        AudioGraphNode::Ptr getNode (AudioGraphNode::Id id)
        {
            auto it = nodes.find (id);
            return (it != nodes.end()) ? it->second : nullptr;
        }

        /**
         * @brief Gets the output node of the graph
         * @return Shared pointer to the output node
         */
        AudioGraphNode::Ptr getOutputNode() const
        {
            return outputNode;
        }

        /**
         * @brief Prepares all nodes in the graph for processing
         * @param sampleRate The sample rate
         * @param maxFramesPerBlock The maximum number of frames per block
         */
        void prepareAll (double sampleRate, int maxFramesPerBlock)
        {
            for (auto& [id, node] : nodes)
            {
                node->prepare (sampleRate, maxFramesPerBlock);
            }
        }

        /**
         * @brief Resets all nodes in the graph
         */
        void resetAll()
        {
            for (auto& [id, node] : nodes)
            {
                node->reset();
            }
        }

        /**
         * @brief Clears all nodes from the graph
         */
        void clear()
        {
            nodes.clear();

            // Recreate the output node
            createOutputNode();
        }

        /**
         * @brief Performs topological sorting of the node graph
         * @return Vector of nodes in processing order
         */
        std::vector<AudioGraphNode::Ptr> createRenderSequence() const
        {
            std::vector<AudioGraphNode::Ptr> sequence;
            std::unordered_map<AudioGraphNode::Id, bool> visited;
            std::unordered_map<AudioGraphNode::Id, bool> inStack; // For cycle detection

            // DFS-based topological sort
            std::function<bool (AudioGraphNode::Ptr)> dfs =
                [&] (AudioGraphNode::Ptr node) -> bool {
                if (!node)
                    return true;

                AudioGraphNode::Id nodeId = node->getId();

                // Check for cycles
                if (inStack[nodeId])
                    return false; // Cycle detected

                // Skip if already visited
                if (visited[nodeId])
                    return true;

                inStack[nodeId] = true;

                // Visit all inputs first
                for (const auto& [channel, connections] : node->getInputConnections())
                {
                    for (const auto& connection : connections)
                    {
                        if (auto sourceNode = connection.node.lock())
                        {
                            if (!dfs (sourceNode))
                                return false; // Propagate cycle detection
                        }
                    }
                }

                // Add to sequence after dependencies
                sequence.push_back (node);
                visited[nodeId] = true;
                inStack[nodeId] = false;

                return true;
            };

            // Start with the output node
            auto output = getOutputNode();
            if (output)
            {
                if (!dfs (output))
                {
                    // Handle cycle detection - clear sequence and return empty
                    sequence.clear();
                }
            }

            // The sequence is already in the correct order for determining the output node
            // (output node at the end), but we don't need to reverse it since we use
            // the recursive processing approach
            return sequence;
        }

    private:
        void createOutputNode()
        {
            auto outputProcessor = std::make_unique<OutputProcessor>();
            outputNode = addNode (std::move (outputProcessor));
        }

        std::unordered_map<AudioGraphNode::Id, AudioGraphNode::Ptr> nodes;
        AudioGraphNode::Ptr outputNode;
    };
}
