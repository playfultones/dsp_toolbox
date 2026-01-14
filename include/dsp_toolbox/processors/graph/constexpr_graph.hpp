/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/core/buffer_view.hpp"
#include "dsp_toolbox/core/constexpr_spec.hpp"
#include "dsp_toolbox/core/io_config.hpp"
#include "dsp_toolbox/processors/core/processor_base.hpp"
#include "dsp_toolbox/simd/simd.hpp"

#include <any>
#include <array>
#include <concepts>
#include <cstddef>
#include <tuple>
#include <type_traits>
#include <utility>

namespace PlayfulTones::DspToolbox::Graph
{

    /**
     * @brief Explicit connection between node channels.
     *
     * Describes a single data flow from one node's output channel to another
     * node's input channel. Supports both audio and CV routing.
     *
     * ## Channel Numbering
     * Channels are relative to each node's buffer slice:
     * - Audio channels: [0, audioChannels)
     * - CV channels: [audioChannels, totalChannels)
     *
     * ## Example
     * @code
     * // VCO has IOConfig<0, 1, 1, 0> → totalChannels=2
     * //   Channel 0: audio output
     * //   Channel 1: pitch CV input
     *
     * // VCA has IOConfig<1, 1, 1, 0> → totalChannels=2
     * //   Channel 0: audio in/out
     * //   Channel 1: amplitude CV input
     *
     * Connection{kVCO, 0, kVCA, 0}  // VCO audio → VCA audio
     * Connection{kEnv, 0, kVCA, 1}  // Env CV → VCA amplitude CV
     * @endcode
     */
    struct Connection
    {
        std::size_t sourceNode; ///< Source node ID
        std::size_t sourceChannel; ///< Channel within source node (0-based)
        std::size_t destNode; ///< Destination node ID
        std::size_t destChannel; ///< Channel within dest node (0-based)
    };

    /**
     * @brief Maps an external channel to an internal node channel.
     *
     * Used by GraphProcessor to define how external buffer channels map to
     * channels within the nested graph's nodes.
     *
     * ## Example
     * @code
     * // Map external CV input 0 to internal Clock node's trigger input
     * ChannelMapping{0, kClock, 0}
     *
     * // Map internal VCA output to external audio output 0
     * ChannelMapping{0, kVCA, 0}
     * @endcode
     */
    struct ChannelMapping
    {
        std::size_t externalChannel; ///< Channel in external (parent) buffer
        std::size_t internalNode; ///< Node index within the graph
        std::size_t internalChannel; ///< Channel within that node's slice
    };

    /**
     * @brief Concept for types that define a graph's external interface.
     *
     * An ExternalIO type specifies:
     * - IOConfig: The external channel configuration (how the graph appears to parent)
     * - Spec: The ConstexprSpec for the graph
     * - inputMappings: Array mapping external inputs to internal node channels
     * - outputMappings: Array mapping internal node channels to external outputs
     *
     * ## Example
     * @code
     * struct KickDrumIO {
     *     using IOConfig = IOConfig<0, 1, 1, 0>;  // 1 audio out, 1 CV in
     *     static constexpr auto Spec = DefaultSpec;
     *
     *     static constexpr std::array<ChannelMapping, 1> inputMappings = {{
     *         {0, 0, 0}   // External CV 0 → internal node 0 channel 0
     *     }};
     *
     *     static constexpr std::array<ChannelMapping, 1> outputMappings = {{
     *         {0, 3, 0}   // Internal node 3 channel 0 → external audio 0
     *     }};
     * };
     * @endcode
     */
    template <typename T>
    concept ExternalIOLike = IOConfigLike<typename T::IOConfig> && requires {
        { T::Spec } -> std::convertible_to<ConstexprSpec>;
        { T::inputMappings };
        { T::outputMappings };
    };

    /**
     * @brief Simple node wrapper with ID only.
     *
     * Used with explicit Connection arrays. The node ID is its index in the
     * nodes tuple (position-based).
     *
     * @tparam Processor The processor type
     */
    template <typename Processor>
    struct Node
    {
        using processor_type = Processor;
        using io_config = typename Processor::IOConfig;

        Processor processor;

        constexpr Node() = default;
        constexpr explicit Node (Processor p) : processor (std::move (p)) {}
    };

    namespace Detail
    {
        // Helper to compute channel offset for a node at Index
        // Uses constexpr lambda with fold expression for cleaner implementation
        template <typename NodesTuple, std::size_t Index>
        struct ChannelOffsetHelper
        {
            static constexpr std::size_t compute()
            {
                if constexpr (Index == 0)
                {
                    return 0;
                }
                else
                {
                    return computeImpl (std::make_index_sequence<Index> {});
                }
            }

        private:
            template <std::size_t... Is>
            static constexpr std::size_t computeImpl (std::index_sequence<Is...>)
            {
                return (std::tuple_element_t<Is, NodesTuple>::io_config::totalChannels + ... + 0);
            }
        };

        // Helper to compute total channels
        template <typename... Nodes>
        struct TotalChannelsImpl
        {
            static constexpr std::size_t value = (Nodes::io_config::totalChannels + ... + 0);
        };

        // Helper to compute max node channels (for temporary buffer)
        template <typename... Nodes>
        struct MaxNodeChannelsImpl;

        template <>
        struct MaxNodeChannelsImpl<>
        {
            static constexpr std::size_t value = 0;
        };

        template <typename First, typename... Rest>
        struct MaxNodeChannelsImpl<First, Rest...>
        {
            static constexpr std::size_t firstChannels = First::io_config::totalChannels;
            static constexpr std::size_t restMax = MaxNodeChannelsImpl<Rest...>::value;
            static constexpr std::size_t value = (firstChannels > restMax) ? firstChannels : restMax;
        };

        // Check if a State type has shared = false (needs separate L/R processors)
        template <typename StateT>
        constexpr bool state_needs_separate_channels()
        {
            if constexpr (requires { StateT::shared; })
            {
                return !StateT::shared; // true if shared = false
            }
            return false; // default: can share state
        }

        // Check if a processor needs separate state per channel
        template <typename Processor>
        constexpr bool processor_needs_separate_state()
        {
            if constexpr (requires { typename Processor::State; })
            {
                return state_needs_separate_channels<typename Processor::State>();
            }
            return false; // stateless processors can share
        }

        // Count connections targeting a specific node (for pre-indexed lookup)
        template <auto Connections, std::size_t DestNode>
        constexpr std::size_t countConnectionsToNode()
        {
            std::size_t count = 0;
            for (std::size_t i = 0; i < Connections.size(); ++i)
            {
                if (Connections[i].destNode == DestNode)
                    ++count;
            }
            return count;
        }

        // Build array of connection indices for a specific destination node
        template <auto Connections, std::size_t DestNode>
        constexpr auto getConnectionIndicesForNode()
        {
            constexpr std::size_t count = countConnectionsToNode<Connections, DestNode>();
            std::array<std::size_t, count> indices {};
            std::size_t idx = 0;
            for (std::size_t i = 0; i < Connections.size(); ++i)
            {
                if (Connections[i].destNode == DestNode)
                    indices[idx++] = i;
            }
            return indices;
        }

        // Generate lookup table: tuple of arrays, one per node
        template <auto Connections, std::size_t... NodeIndices>
        constexpr auto makeConnectionLookupImpl (std::index_sequence<NodeIndices...>)
        {
            return std::make_tuple (getConnectionIndicesForNode<Connections, NodeIndices>()...);
        }

        template <auto Connections, std::size_t NumNodes>
        constexpr auto makeConnectionLookup()
        {
            return makeConnectionLookupImpl<Connections> (std::make_index_sequence<NumNodes> {});
        }

    } // namespace Detail

    /**
     * @brief Zero-overhead constexpr graph for static topologies.
     *
     * Stores processors in a tuple and processes via fold expression,
     * enabling full inlining with no virtual dispatch overhead.
     *
     * ## Features
     * - Explicit connections for both audio and CV routing
     * - Node ID = tuple index (position-based)
     * - Zero overhead - connections wired at compile time
     * - Compile-time channel offset calculation
     *
     * ## Buffer Layout
     * The graph requires a buffer with TotalChannels channels, laid out as:
     * - Node 0: channels [0, node0.totalChannels)
     * - Node 1: channels [node0.totalChannels, node0.totalChannels + node1.totalChannels)
     * - etc.
     *
     * ## Example Usage
     * @code
     * enum Nodes { kClock, kEnv, kVCO, kVCA };
     *
     * constexpr std::array<Connection, 4> connections = {{
     *     {kClock, 0, kEnv, 0},    // Clock trigger → Env trigger
     *     {kEnv, 0, kVCO, 1},      // Env CV → VCO pitch
     *     {kVCO, 0, kVCA, 0},      // VCO audio → VCA audio
     *     {kEnv, 0, kVCA, 1},      // Env CV → VCA amplitude
     * }};
     *
     * auto graph = makeConstexprGraph<connections>(
     *     Clock{}, ADEnvelope{}, VCO{}, VCA{}
     * );
     * @endcode
     *
     * @tparam Connections Compile-time array of Connection structs
     * @tparam Nodes Node types (Node<Processor>)
     */
    template <auto Connections, typename... Nodes>
    class ConstexprGraph
    {
    public:
        /** @brief Number of nodes in the graph */
        static constexpr std::size_t NumNodes = sizeof...(Nodes);

        /** @brief Number of connections */
        static constexpr std::size_t NumConnections = Connections.size();

        /** @brief Total channels required in the buffer */
        static constexpr std::size_t TotalChannels = Detail::TotalChannelsImpl<Nodes...>::value;

        /** @brief Maximum channels for any single node */
        static constexpr std::size_t MaxNodeChannels = Detail::MaxNodeChannelsImpl<Nodes...>::value;

        /** @brief Pre-computed lookup: for each node, which connection indices target it */
        static constexpr auto ConnectionLookup = Detail::makeConnectionLookup<Connections, NumNodes>();

        constexpr ConstexprGraph() = default;

        constexpr explicit ConstexprGraph (Nodes... nodes)
            : nodes_ (std::move (nodes)...)
        {
        }

        template <std::size_t Index>
        [[nodiscard]] constexpr auto& getNode() noexcept
        {
            return std::get<Index> (nodes_);
        }

        template <std::size_t Index>
        [[nodiscard]] constexpr const auto& getNode() const noexcept
        {
            return std::get<Index> (nodes_);
        }

        template <std::size_t Index>
        [[nodiscard]] constexpr auto& getProcessor() noexcept
        {
            return std::get<Index> (nodes_).processor;
        }

        template <std::size_t Index>
        [[nodiscard]] constexpr const auto& getProcessor() const noexcept
        {
            return std::get<Index> (nodes_).processor;
        }

        template <std::size_t Index>
        [[nodiscard]] static constexpr std::size_t getChannelOffset() noexcept
        {
            return Detail::ChannelOffsetHelper<std::tuple<Nodes...>, Index>::compute();
        }

        template <typename SampleType>
        constexpr void process (BufferView<SampleType>& buffer, std::size_t sampleCount) noexcept
        {
            processImpl<SampleType> (buffer, sampleCount, std::index_sequence_for<Nodes...> {});
        }

        template <typename SampleType>
        constexpr void process (BufferView<SampleType>& buffer) noexcept
        {
            process (buffer, buffer.getNumSamples());
        }

        constexpr void reset() noexcept
        {
            std::apply (
                [] (auto&... nodes) {
                    (nodes.processor.reset(), ...);
                },
                nodes_);
        }

        /**
         * @brief Check if any node in the graph needs separate state per channel.
         *
         * Used by GraphProcessor to determine if stereo processing requires
         * independent L/R processor instances.
         */
        static constexpr bool anyNodeNeedsSeparateState() noexcept
        {
            return anyNodeNeedsSeparateStateImpl (std::index_sequence_for<Nodes...> {});
        }

    private:
        std::tuple<Nodes...> nodes_;

        template <std::size_t... Is>
        static constexpr bool anyNodeNeedsSeparateStateImpl (std::index_sequence<Is...>) noexcept
        {
            // Note: Node<P>::processor_type is the actual processor type
            return (Detail::processor_needs_separate_state<
                        typename std::tuple_element_t<Is, std::tuple<Nodes...>>::processor_type>()
                    || ...);
        }

        template <typename SampleType, std::size_t... Indices>
        constexpr void processImpl (BufferView<SampleType>& buffer, std::size_t sampleCount, std::index_sequence<Indices...>) noexcept
        {
            (processNode<SampleType, Indices> (buffer, sampleCount), ...);
        }

        template <typename SampleType, std::size_t NodeIndex>
        constexpr void processNode (BufferView<SampleType>& buffer, std::size_t sampleCount) noexcept
        {
            using NodeType = std::tuple_element_t<NodeIndex, std::tuple<Nodes...>>;
            using IOConfig = typename NodeType::io_config;

            constexpr std::size_t offset = getChannelOffset<NodeIndex>();
            constexpr std::size_t numChannels = IOConfig::totalChannels;

            // Wire connections to this node before processing
            wireConnectionsToNode<SampleType, NodeIndex> (buffer, sampleCount);

            // Create sub-view for this node
            std::array<SampleType*, numChannels> channelPtrs {};
            for (std::size_t ch = 0; ch < numChannels; ++ch)
            {
                channelPtrs[ch] = buffer.getWritePointer (offset + ch);
            }

            BufferView<SampleType> nodeBuffer (channelPtrs.data(), numChannels, sampleCount);

            // Process the node
            std::get<NodeIndex> (nodes_).processor.process (nodeBuffer, sampleCount);
        }

        template <typename SampleType, std::size_t DestNodeIndex>
        constexpr void wireConnectionsToNode (BufferView<SampleType>& buffer, std::size_t sampleCount) noexcept
        {
            constexpr auto& connIndices = std::get<DestNodeIndex> (ConnectionLookup);
            wirePreIndexedConnections<SampleType, DestNodeIndex> (
                buffer, sampleCount, std::make_index_sequence<connIndices.size()> {});
        }

        template <typename SampleType, std::size_t DestNodeIndex, std::size_t... LocalIndices>
        constexpr void wirePreIndexedConnections (
            BufferView<SampleType>& buffer,
            std::size_t sampleCount,
            std::index_sequence<LocalIndices...>) noexcept
        {
            constexpr auto& connIndices = std::get<DestNodeIndex> (ConnectionLookup);
            (wireConnection<SampleType, connIndices[LocalIndices]> (buffer, sampleCount), ...);
        }

        template <typename SampleType, std::size_t ConnIndex>
        constexpr void wireConnection (BufferView<SampleType>& buffer, std::size_t sampleCount) noexcept
        {
            constexpr auto conn = Connections[ConnIndex];
            constexpr std::size_t sourceOffset = getChannelOffset<conn.sourceNode>();
            constexpr std::size_t destOffset = getChannelOffset<conn.destNode>();

            SampleType* src = buffer.getWritePointer (sourceOffset + conn.sourceChannel);
            SampleType* dst = buffer.getWritePointer (destOffset + conn.destChannel);

            simd::copy (dst, src, sampleCount);
        }
    };

    /**
     * @brief Wrap a processor in a Node.
     *
     * @tparam Processor Processor type (deduced)
     * @param processor Processor instance
     * @return Node wrapping the processor
     */
    template <typename Processor>
    constexpr auto node (Processor processor)
    {
        return Node<Processor> { std::move (processor) };
    }

    /**
     * @brief Create a ConstexprGraph with explicit connections.
     *
     * Node IDs are their position in the parameter list (0, 1, 2, ...).
     *
     * @tparam Connections Compile-time array of Connection structs
     * @tparam Processors Processor types (deduced)
     * @param processors Processor instances
     * @return ConstexprGraph with the specified topology
     *
     * ## Example
     * @code
     * constexpr std::array<Connection, 2> connections = {{
     *     {0, 0, 1, 1},  // Node 0 ch 0 → Node 1 ch 1
     *     {1, 0, 2, 0},  // Node 1 ch 0 → Node 2 ch 0
     * }};
     *
     * auto graph = makeConstexprGraph<connections>(
     *     Clock{}, VCO{}, VCA{}
     * );
     * @endcode
     */
    template <auto Connections, typename... Processors>
    constexpr auto makeConstexprGraph (Processors... processors)
    {
        return ConstexprGraph<Connections, Node<Processors>...> { Node<Processors> { std::move (processors) }... };
    }

    /**
     * @brief Internal buffer state for GraphProcessor.
     *
     * Holds only the internal buffer for processing. The graph itself
     * is stored directly in GraphProcessor to avoid move/copy assignment issues.
     *
     * @tparam InternalChannels Total channels in the graph (Graph::TotalChannels)
     * @tparam MaxBlockSize Maximum samples per block
     * @tparam SampleType Sample type (float or double)
     */
    template <std::size_t InternalChannels, std::size_t MaxBlockSize, typename SampleType = float>
    struct GraphProcessorState
    {
        std::array<std::array<SampleType, MaxBlockSize>, InternalChannels> internalBuffer {};

        // Cached channel pointers (populated on first use)
        std::array<SampleType*, InternalChannels> channelPtrs {};
        bool ptrsCached { false };

        constexpr void resetTransient() noexcept
        {
            for (auto& channel : internalBuffer)
            {
                channel.fill (SampleType {});
            }
        }

        /**
         * @brief Migrate from type-erased state.
         *
         * GraphProcessorState contains only transient buffer data (scratch space
         * for processing). No meaningful state needs to be preserved across spec
         * changes, so we return a fresh default-initialized state.
         *
         * @param old Type-erased previous state (ignored for transient buffers)
         * @return Fresh GraphProcessorState
         */
        static GraphProcessorState migrateFromAny (const std::any& /*old*/) noexcept
        {
            return {}; // Transient buffer - start fresh
        }
    };

    /**
     * @brief GraphProcessorState wrapper that propagates the shared flag from graph nodes.
     *
     * This wrapper inherits from GraphProcessorState and adds the `shared` flag
     * based on whether any node in the graph needs separate state per channel.
     * Used by StereoExpander to determine if L/R need independent processors.
     *
     * @tparam Graph The ConstexprGraph type (needed to call anyNodeNeedsSeparateState)
     * @tparam MaxBlockSize Maximum samples per block
     * @tparam SampleType Sample type (float or double)
     */
    template <typename Graph, std::size_t MaxBlockSize, typename SampleType = float>
    struct GraphProcessorStateWithSharing : GraphProcessorState<Graph::TotalChannels, MaxBlockSize, SampleType>
    {
        using BaseState = GraphProcessorState<Graph::TotalChannels, MaxBlockSize, SampleType>;

        /// If any node needs separate state, the whole graph needs separate state
        static constexpr bool shared = !Graph::anyNodeNeedsSeparateState();

        constexpr GraphProcessorStateWithSharing() = default;

        static GraphProcessorStateWithSharing migrateFromAny (const std::any& old) noexcept
        {
            GraphProcessorStateWithSharing result;
            static_cast<BaseState&> (result) = BaseState::migrateFromAny (old);
            return result;
        }
    };

    /**
     * @brief Wraps a ConstexprGraph to be used as a processor node.
     *
     * GraphProcessor enables hierarchical graph composition by wrapping
     * a ConstexprGraph and exposing it with a defined external interface.
     *
     * ## How It Works
     * 1. External inputs are copied to the internal buffer at mapped locations
     * 2. The nested graph processes the internal buffer
     * 3. Internal outputs are copied back to the external buffer
     *
     * ## Example
     * @code
     * // Define a simple subgraph
     * constexpr std::array<Connection, 2> envVcaConns = {{
     *     {0, 0, 1, 0},  // Clock → Envelope trigger
     *     {1, 0, 2, 1},  // Envelope → VCA amplitude
     * }};
     *
     * // Define external interface
     * struct EnvVcaIO {
     *     using IOConfig = IOConfig<1, 1, 1, 0>;  // 1 audio, 1 CV in
     *     static constexpr auto Spec = DefaultSpec;
     *
     *     static constexpr std::array<ChannelMapping, 2> inputMappings = {{
     *         {0, 2, 0},   // External audio 0 → VCA audio in
     *         {1, 0, 0},   // External CV 0 → Clock trigger
     *     }};
     *
     *     static constexpr std::array<ChannelMapping, 1> outputMappings = {{
     *         {0, 2, 0},   // VCA audio out → external audio 0
     *     }};
     * };
     *
     * // Create the nested processor
     * auto subgraph = makeConstexprGraph<envVcaConns>(Clock{}, ADEnvelope{}, VCA{});
     * auto processor = makeGraphProcessor<EnvVcaIO>(std::move(subgraph));
     *
     * // Use in a parent graph
     * auto parentGraph = makeConstexprGraph<parentConns>(
     *     VCO{}, processor, Mixer{}
     * );
     * @endcode
     *
     * @tparam ExternalIO Type satisfying ExternalIOLike (defines IOConfig, Spec, mappings)
     * @tparam Graph The ConstexprGraph type being wrapped
     * @tparam InternalSampleType Sample type for internal buffer and processing (default: float)
     */
    template <ExternalIOLike ExternalIO, typename Graph, typename InternalSampleType = float>
    class GraphProcessor : public ProcessorBase<
                               GraphProcessor<ExternalIO, Graph, InternalSampleType>,
                               typename ExternalIO::IOConfig,
                               GraphProcessorStateWithSharing<Graph, ExternalIO::Spec.blockSize.value, InternalSampleType>,
                               ExternalIO::Spec>
    {
    public:
        using StateType = GraphProcessorStateWithSharing<Graph, ExternalIO::Spec.blockSize.value, InternalSampleType>;

        /** @brief Input channel mappings from ExternalIO */
        static constexpr auto InputMappings = ExternalIO::inputMappings;

        /** @brief Output channel mappings from ExternalIO */
        static constexpr auto OutputMappings = ExternalIO::outputMappings;

        constexpr GraphProcessor() = default;

        constexpr explicit GraphProcessor (Graph graph) noexcept
            : graph_ (std::move (graph))
        {
        }

        /**
         * @brief Process implementation for ProcessorBase.
         *
         * 1. Copy external inputs to internal buffer
         * 2. Process the nested graph
         * 3. Copy internal outputs to external buffer
         *
         * @tparam ExternalSampleType Sample type of the external buffer
         */
        template <typename ExternalSampleType>
        constexpr void processImpl (BufferView<ExternalSampleType>& buffer, StateType& state, std::size_t sampleCount) noexcept
        {
            // Cache channel pointers on first use (avoid rebuilding each call)
            if (!state.ptrsCached)
            {
                for (std::size_t ch = 0; ch < Graph::TotalChannels; ++ch)
                {
                    state.channelPtrs[ch] = state.internalBuffer[ch].data();
                }
                state.ptrsCached = true;
            }

            BufferView<InternalSampleType> internalBuffer (state.channelPtrs.data(), Graph::TotalChannels, sampleCount);

            // Copy inputs from external buffer to internal buffer
            copyInputs (buffer, internalBuffer, sampleCount);

            // Process the nested graph
            graph_.process (internalBuffer, sampleCount);

            // Copy outputs from internal buffer to external buffer
            copyOutputs (internalBuffer, buffer, sampleCount);
        }

        /**
         * @brief Reset processor state including nested graph.
         *
         * Shadows ProcessorBase::reset() to also reset the nested graph.
         */
        constexpr void reset() noexcept
        {
            // Reset internal buffer state
            this->state_.resetTransient();
            // Reset nested graph
            graph_.reset();
        }

        /**
         * @brief Access a processor within the nested graph.
         *
         * @tparam NodeIndex Index of the node in the nested graph
         * @return Reference to the processor
         */
        template <std::size_t NodeIndex>
        [[nodiscard]] constexpr auto& getNestedProcessor() noexcept
        {
            return graph_.template getProcessor<NodeIndex>();
        }

        template <std::size_t NodeIndex>
        [[nodiscard]] constexpr const auto& getNestedProcessor() const noexcept
        {
            return graph_.template getProcessor<NodeIndex>();
        }

        /**
         * @brief Access the nested graph directly.
         */
        [[nodiscard]] constexpr Graph& getNestedGraph() noexcept
        {
            return graph_;
        }

        [[nodiscard]] constexpr const Graph& getNestedGraph() const noexcept
        {
            return graph_;
        }

    private:
        Graph graph_ {};

        template <typename ExternalSampleType>
        constexpr void copyInputs (BufferView<ExternalSampleType>& external, BufferView<InternalSampleType>& internal, std::size_t sampleCount) noexcept
        {
            for (const auto& mapping : InputMappings)
            {
                const std::size_t destOffset = getInternalChannelOffset (mapping.internalNode) + mapping.internalChannel;

                const ExternalSampleType* src = external.getReadPointer (mapping.externalChannel);
                InternalSampleType* dst = internal.getWritePointer (destOffset);

                if constexpr (std::is_same_v<ExternalSampleType, InternalSampleType>)
                {
                    simd::copy (dst, src, sampleCount);
                }
                else
                {
                    for (std::size_t i = 0; i < sampleCount; ++i)
                    {
                        dst[i] = static_cast<InternalSampleType> (src[i]);
                    }
                }
            }
        }

        template <typename ExternalSampleType>
        constexpr void copyOutputs (BufferView<InternalSampleType>& internal, BufferView<ExternalSampleType>& external, std::size_t sampleCount) noexcept
        {
            for (const auto& mapping : OutputMappings)
            {
                const std::size_t srcOffset = getInternalChannelOffset (mapping.internalNode) + mapping.internalChannel;

                const InternalSampleType* src = internal.getReadPointer (srcOffset);
                ExternalSampleType* dst = external.getWritePointer (mapping.externalChannel);

                if constexpr (std::is_same_v<ExternalSampleType, InternalSampleType>)
                {
                    simd::copy (dst, src, sampleCount);
                }
                else
                {
                    for (std::size_t i = 0; i < sampleCount; ++i)
                    {
                        dst[i] = static_cast<ExternalSampleType> (src[i]);
                    }
                }
            }
        }

        /**
         * @brief Get channel offset for a node at runtime.
         *
         * This is needed because ChannelMapping stores node index as runtime value.
         */
        static constexpr std::size_t getInternalChannelOffset (std::size_t nodeIndex) noexcept
        {
            return getInternalChannelOffsetImpl (nodeIndex, std::make_index_sequence<Graph::NumNodes> {});
        }

        template <std::size_t... Is>
        static constexpr std::size_t getInternalChannelOffsetImpl (std::size_t nodeIndex, std::index_sequence<Is...>) noexcept
        {
            std::size_t offset = 0;
            ((Is < nodeIndex ? offset += Graph::template getChannelOffset<Is + 1>() - Graph::template getChannelOffset<Is>() : 0), ...);
            return offset;
        }
    };

    /**
     * @brief Create a GraphProcessor wrapping a ConstexprGraph.
     *
     * @tparam ExternalIO Type defining the external interface (IOConfig, Spec, mappings)
     * @tparam InternalSampleType Sample type for internal buffer (default: float)
     * @tparam Graph ConstexprGraph type (deduced)
     * @param graph The graph to wrap
     * @return GraphProcessor wrapping the graph
     *
     * ## Example
     * @code
     * auto subgraph = makeConstexprGraph<connections>(Clock{}, Env{}, VCA{});
     * auto processor = makeGraphProcessor<MyIO>(std::move(subgraph));
     *
     * // For double precision:
     * auto processorDouble = makeGraphProcessor<MyIO, double>(std::move(subgraph));
     * @endcode
     */
    template <typename ExternalIO, typename InternalSampleType = float, typename Graph>
    constexpr auto makeGraphProcessor (Graph graph)
    {
        return GraphProcessor<ExternalIO, Graph, InternalSampleType> { std::move (graph) };
    }

} // namespace PlayfulTones::DspToolbox::Graph
