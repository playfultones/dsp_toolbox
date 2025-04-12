#pragma once
#include "processor_node.h"
#include <atomic>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <type_traits>

namespace PlayfulTones::DspToolBox
{
    namespace detail
    {
        static_assert (std::atomic<ProcessorNode::Ptr>::is_always_lock_free,
            "std::atomic<ProcessorNode::Ptr> must be lock-free for real-time audio processing");
    }

    class ProcessorChain : public Processor
    {
    public:
        ProcessorChain() = default;

        void prepare (double sampleRate, int maxFramesPerBlock) override
        {
            if (auto* node = outputNode.load (std::memory_order_acquire))
                node->prepare (sampleRate, maxFramesPerBlock);
        }

        void process (float** buffer, int numChannels, int numFrames) override
        {
            if (auto* node = outputNode.load (std::memory_order_acquire))
                node->process (buffer, numChannels, numFrames);
        }

        void reset() override
        {
            if (auto* node = outputNode.load (std::memory_order_acquire))
                node->reset();
        }

        void setOutputNode (ProcessorNode::Ptr node)
        {
            std::lock_guard<std::mutex> lock (modificationMutex);
            outputNode.store (node, std::memory_order_release);
        }

        ProcessorNode::Ptr getOutputNode()
        {
            return outputNode.load (std::memory_order_acquire);
        }

        template <typename T, typename... Args>
        ProcessorNode::Ptr createNode (Args&&... args)
        {
            std::lock_guard<std::mutex> lock (modificationMutex);
            auto processor = std::make_unique<T> (std::forward<Args> (args)...);
            auto node = std::make_unique<ProcessorNode> (std::move (processor));
            auto nodePtr = node.get();
            nodes.push_back (std::move (node));
            return nodePtr;
        }

        void clear()
        {
            std::lock_guard<std::mutex> lock (modificationMutex);
            outputNode.store (nullptr, std::memory_order_release);
            nodes.clear();
        }

    private:
        std::atomic<ProcessorNode::Ptr> outputNode { nullptr };
        std::vector<std::unique_ptr<ProcessorNode>> nodes;
        std::mutex modificationMutex;
    };
}