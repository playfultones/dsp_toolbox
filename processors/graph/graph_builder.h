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
     * @brief High-performance template-based graph builder
     * 
     * This class handles the creation and connection of nodes in the audio graph,
     * with compile-time optimization for maximum performance.
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
    class GraphBuilder
    {
    public:
        using NodeType = AudioGraphNode<SampleType, BlockSize, SampleRate, NumChannels>;
        using NodePtr = typename NodeType::Ptr;
        using NodeId = typename NodeType::Id;
        
        GraphBuilder()
        {
            createOutputNode();
        }

        /**
         * @brief Adds a processor node to the graph using CRTP processor
         * @tparam ProcessorType The CRTP processor type
         * @tparam Args Constructor argument types
         * @param args Constructor arguments for the processor
         * @return Shared pointer to the created node
         */
        template<Processor ProcessorType, typename... Args>
        NodePtr addNode(Args&&... args)
        {
            auto wrapper = std::make_unique<ProcessorWrapper<ProcessorType>>(std::forward<Args>(args)...);
            auto node = std::make_shared<NodeType>(std::move(wrapper));
            NodeId id = node->getId();
            nodes_[id] = node;
            return node;
        }

        /**
         * @brief Removes a node from the graph
         * @param id The ID of the node to remove
         * @return True if the node was found and removed
         */
        bool removeNode(NodeId id) noexcept
        {
            return nodes_.erase(id) > 0;
        }

        /**
         * @brief Connects two nodes in the graph
         * @param sourceId The ID of the source node
         * @param targetId The ID of the target node
         * @param outputChannel The output channel on the source node
         * @param inputChannel The input channel on the target node
         * @return True if the connection was successful
         */
        bool connect(NodeId sourceId, NodeId targetId, int outputChannel = 0, int inputChannel = 0) noexcept
        {
            auto sourceIt = nodes_.find(sourceId);
            auto targetIt = nodes_.find(targetId);

            if (sourceIt == nodes_.end() || targetIt == nodes_.end())
                return false;

            return targetIt->second->addInputConnection(sourceIt->second, outputChannel, inputChannel);
        }

        /**
         * @brief Disconnects two nodes in the graph
         * @param sourceId The ID of the source node
         * @param targetId The ID of the target node
         * @param outputChannel The output channel on the source node
         * @param inputChannel The input channel on the target node
         * @return True if the disconnection was successful
         */
        bool disconnect(NodeId sourceId, NodeId targetId, int outputChannel = 0, int inputChannel = 0) noexcept
        {
            auto sourceIt = nodes_.find(sourceId);
            auto targetIt = nodes_.find(targetId);

            if (sourceIt == nodes_.end() || targetIt == nodes_.end())
                return false;

            return targetIt->second->removeInputConnection(sourceIt->second, outputChannel, inputChannel);
        }

        /**
         * @brief Gets a node by its ID
         * @param id The ID of the node to get
         * @return Shared pointer to the node, or nullptr if not found
         */
        NodePtr getNode(NodeId id) noexcept
        {
            auto it = nodes_.find(id);
            return (it != nodes_.end()) ? it->second : nullptr;
        }

        /**
         * @brief Gets the output node of the graph
         * @return Shared pointer to the output node
         */
        NodePtr getOutputNode() const noexcept
        {
            return outputNode_;
        }

        /**
         * @brief Prepares all nodes in the graph for processing
         */
        void prepareAll() noexcept
        {
            for (auto& [id, node] : nodes_)
            {
                node->prepare();
            }
        }

        /**
         * @brief Resets all nodes in the graph
         */
        void resetAll() noexcept
        {
            for (auto& [id, node] : nodes_)
            {
                node->reset();
            }
        }

        /**
         * @brief Process control updates for all nodes (cold path)
         */
        void processControlAll() noexcept
        {
            for (auto& [id, node] : nodes_)
            {
                node->processControl();
            }
        }

        /**
         * @brief Clears all nodes from the graph
         */
        void clear()
        {
            nodes_.clear();
            createOutputNode();
        }

        /**
         * @brief Performs topological sorting of the node graph
         * @return Vector of nodes in processing order
         */
        std::vector<NodePtr> createRenderSequence() const
        {
            std::vector<NodePtr> sequence;
            std::unordered_map<NodeId, bool> visited;
            std::unordered_map<NodeId, bool> inStack; // For cycle detection

            // DFS-based topological sort
            std::function<bool(NodePtr)> dfs = [&](NodePtr node) -> bool {
                if (!node)
                    return true;

                NodeId nodeId = node->getId();

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
                            if (!dfs(sourceNode))
                                return false; // Propagate cycle detection
                        }
                    }
                }

                // Add to sequence after dependencies
                sequence.push_back(node);
                visited[nodeId] = true;
                inStack[nodeId] = false;

                return true;
            };

            // Start with the output node
            auto output = getOutputNode();
            if (output)
            {
                if (!dfs(output))
                {
                    // Handle cycle detection - clear sequence and return empty
                    sequence.clear();
                }
            }

            return sequence;
        }

        /**
         * @brief Get compile-time configuration
         */
        static constexpr size_t getBlockSize() noexcept { return BlockSize; }
        static constexpr size_t getSampleRate() noexcept { return SampleRate; }
        static constexpr size_t getNumChannels() noexcept { return NumChannels; }

    private:
        void createOutputNode()
        {
            outputNode_ = addNode<OutputProcessor<SampleType, BlockSize, SampleRate, NumChannels>>();
        }

        std::unordered_map<NodeId, NodePtr> nodes_;
        NodePtr outputNode_;
    };
    
    // Common type aliases
    using GraphBuilderF32 = GraphBuilder<float, 512, 44100, 2>;
    using GraphBuilderF64 = GraphBuilder<double, 512, 44100, 2>;
}
