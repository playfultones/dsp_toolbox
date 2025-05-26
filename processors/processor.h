/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

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
         * @brief This method is used to prepare the processor for playback.
         * It sets the sample rate and calls the virtual prepare method to allow derived classes
         * to implement their own preparation logic.
         * @param sampleRate The sample rate at which the processor will operate.
         * @param maxFramesPerBlock The maximum number of frames that will be processed at once.
         */
        void prepareProcessor (double sampleRate, int maxFramesPerBlock)
        {
            setSampleRate (sampleRate);
            // Call the virtual prepare method to allow derived classes to implement their own preparation logic.
            prepare (sampleRate, maxFramesPerBlock);
        }

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
        double getSampleRate() const
        {
            return sr;
        }

        /**
         * @brief Set the sample rate of the processor.
         * @param sampleRate The new sample rate to set.
         */
        void setSampleRate (double sampleRate)
        {
            sr = sampleRate;
        }

    private:
        double sr = 44100.0; // Default sample rate
    };
} // namespace PlayfulTones::DspToolBox