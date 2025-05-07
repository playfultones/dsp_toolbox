#pragma once
#include "../processor.h"

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief Special processor that serves as the output node in an audio graph
     * 
     * This processor simply passes audio through without modification.
     * It serves as a dedicated terminal node in the audio processing graph.
     */
    class OutputProcessor : public Processor
    {
    public:
        OutputProcessor() = default;
        
        void prepare(double sampleRate, int /* maxFramesPerBlock */) override
        {
            // No processing needed, just store the sample rate
            setSampleRate(sampleRate);
        }
        
        void process(BufferView& /* buffer */) override
        {
            // This is the terminal node, it just passes audio through
            // No processing needed
        }
        
        void reset() override
        {
            // Nothing to reset
        }
    };
}
