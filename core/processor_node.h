#pragma once
#include "processor.h"
#include <algorithm>
#include <atomic>
#include <memory>
#include <mutex>
#include <type_traits>
#include <vector>

namespace PlayfulTones::DspToolBox
{
    class ProcessorNode : public Processor
    {
    public:
        explicit ProcessorNode (std::unique_ptr<Processor> processor)
            : processor (std::move (processor))
        {
            currentInputs.store (&inputs[0]);
        }

        using Ptr = ProcessorNode*;
        using VectorPtr = std::vector<ProcessorNode::Ptr>*;

        void prepare (double sampleRate, int maxFramesPerBlock) override
        {
            processor->prepare (sampleRate, maxFramesPerBlock);
            auto* current = currentInputs.load();
            for (auto& input : *current)
            {
                if (input)
                    input->prepare (sampleRate, maxFramesPerBlock);
            }
        }

        void process (BufferView& buffer) override
        {
            auto* current = currentInputs.load (std::memory_order_acquire);

            for (auto& input : *current)
            {
                if (input)
                    input->process (buffer);
            }

            processor->process (buffer);
        }

        void reset() override
        {
            processor->reset();
            auto* current = currentInputs.load();
            for (auto& input : *current)
            {
                if (input)
                    input->reset();
            }
        }

        void addInput (ProcessorNode::Ptr input)
        {
            if (!input)
                return;

            std::lock_guard<std::mutex> lock (modificationMutex);

            auto* inactive = getInactiveBuffer();
            *inactive = *currentInputs.load();

            // Modify inactive buffer
            inactive->push_back (input);

            // Atomically switch to the new configuration
            currentInputs.store (inactive, std::memory_order_release);
        }

        void removeInput (ProcessorNode::Ptr input)
        {
            if (!input)
                return;

            std::lock_guard<std::mutex> lock (modificationMutex);

            auto* inactive = getInactiveBuffer();
            *inactive = *currentInputs.load();

            inactive->erase (
                std::remove (inactive->begin(), inactive->end(), input),
                inactive->end());

            currentInputs.store (inactive, std::memory_order_release);
        }

        void clearInputs()
        {
            std::lock_guard<std::mutex> lock (modificationMutex);

            auto* inactive = getInactiveBuffer();

            inactive->clear();

            currentInputs.store (inactive, std::memory_order_release);
        }

        Processor* getProcessor() { return processor.get(); }

        template <typename T>
        T* getProcessor()
        {
            if (auto* p = dynamic_cast<T*> (processor.get()))
                return p;
            return nullptr;
        }

    private:
        ProcessorNode::VectorPtr getInactiveBuffer()
        {
            auto* current = currentInputs.load();
            return (current == &inputs[0]) ? &inputs[1] : &inputs[0];
        }

        std::unique_ptr<Processor> processor;

        // Double-buffered connections
        std::vector<ProcessorNode::Ptr> inputs[2];
        std::atomic<ProcessorNode::VectorPtr> currentInputs;

        std::mutex modificationMutex;

        static_assert (std::atomic<ProcessorNode::VectorPtr>::is_always_lock_free,
            "std::atomic<vector<ProcessorNode::Ptr>*> must be lock-free for real-time audio processing");
    };
}