#pragma once
#include "../core/buffer_view.h"

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief Abstract base class for all DSP processors.
     *
     * This class defines the interface that all DSP processors must implement,
     * providing the standard methods for preparation, processing and reset.
     */
    class Processor
    {
    public:
        virtual ~Processor() = default;

        /**
         * @brief Prepare the processor for playback.
         * @param sampleRate The sample rate at which the processor will operate.
         * @param maxFramesPerBlock The maximum number of frames that will be processed at once.
         */
        virtual void prepare (double sampleRate, int maxFramesPerBlock) = 0;

        /**
         * @brief Process audio data using BufferView.
         * @param buffer The buffer view containing the audio data to process.
         */
        virtual void process (BufferView& buffer) = 0;

        /**
         * @brief Legacy process function for backward compatibility.
         * @param buffer Array of audio channel data.
         * @param numChannels Number of channels in the buffer.
         * @param numFrames Number of frames to process.
         */
        void process (float** buffer, int numChannels, int numFrames)
        {
            BufferView view;
            view.setData (buffer, numChannels, numFrames);
            process (view);
        }

        /**
         * @brief Reset the processor's internal state.
         */
        virtual void reset()
        {
            // Default implementation does nothing.
            // Derived classes can override this method to reset their internal state.
        }

        /**
         * @brief Get the sample rate of the processor.
         * @return The sample rate at which the processor is operating.
         */
        virtual double getSampleRate() const
        {
            return sr;
        }

        /**
         * @brief Set the sample rate of the processor.
         * @param sampleRate The new sample rate to set.
         */
        virtual void setSampleRate (double sampleRate)
        {
            sr = sampleRate;
        }

    private:
        double sr = 44100.0; // Default sample rate
    };
} // namespace PlayfulTones::DspToolBox