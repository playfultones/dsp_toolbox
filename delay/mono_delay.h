/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once

#include "../core/ring_buffer.h"
#include "../processors/processor.h"
#include <cmath>

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief A mono delay processor.
     * 
     * It supports delay times up to 2 seconds, with controls for feedback level
     * and dry/wet mix.
     */
    class Delay : public Processor
    {
    public:
        static constexpr auto MaxDelayTimeMs = 2000.0f; // Maximum delay time in milliseconds
        /**
         * @brief Construct a new Delay processor
         * 
         * @param initialDelayMs Initial delay time in milliseconds
         * @param initialFeedback Initial feedback amount (0.0 to 1.0)
         * @param initialMix Initial dry/wet mix (0.0 = dry, 1.0 = wet)
         */
        Delay (float initialDelayMs = 500.0f, float initialFeedback = 0.5f, float initialMix = 0.5f)
            : delayMs (initialDelayMs), feedback (initialFeedback), mix (initialMix), delayBuffer (1) // Start with minimum size, will be resized in prepare()
        {
            // Clamp initial values to valid ranges
            delayMs = std::max (0.0f, std::min (delayMs.load(), MaxDelayTimeMs));
            feedback = std::max (0.0f, std::min (feedback.load(), 1.0f));
            mix = std::max (0.0f, std::min (mix.load(), 1.0f));
        }

        /**
         * @brief Prepare the delay processor for playback
         * 
         * @param sampleRate The sample rate at which the processor will operate
         * @param maxFramesPerBlock The maximum number of frames that will be processed at once
         */
        void prepare (double sampleRate, int /* maxFramesPerBlock */) override
        {
            // Calculate the maximum number of samples needed for 2 seconds of delay
            const auto maxDelaySamples = static_cast<size_t> (std::ceil (2.0 * sampleRate)) + 1;

            // Resize the ring buffer to accommodate the maximum delay time
            delayBuffer.resize (maxDelaySamples);

            // Clear the buffer
            delayBuffer.clear();

            // Calculate the delay in samples based on current delay time in ms
            delaySamples = static_cast<int> ((delayMs / 1000.0f) * sampleRate);
        }

        /**
         * @brief Process audio data
         * 
         * @param buffer The buffer view containing the audio data to process
         */
        void process (BufferView& buffer) override
        {
            const int numFrames = buffer.getNumFrames();

            // Process only the specified channel
            float* channelData = buffer.getChannelPointer (channelIndex);

            for (int i = 0; i < numFrames; ++i)
            {
                // Get the current sample
                const float inputSample = channelData[i];

                // Read from the delay buffer
                float delaySample = 0.0f;
                delayBuffer.peek (delaySample);

                // Calculate the output sample with dry/wet mix
                const float outputSample = (1.0f - mix) * inputSample + mix * delaySample;

                // Write back to the delay buffer with feedback
                delayBuffer.pop (delaySample); // Remove the sample we just read
                delayBuffer.push (inputSample + feedback * delaySample);

                // Write the output sample to the buffer
                channelData[i] = outputSample;
            }
        }

        /**
         * @brief Reset the delay processor's internal state
         */
        void reset() override
        {
            delayBuffer.clear();
        }

        /**
         * @brief Set the delay time in milliseconds
         * 
         * @param delayTimeMs The new delay time in milliseconds (0 to 2000)
         */
        void setDelayTime (float delayTimeMs)
        {
            // Clamp to valid range
            delayTimeMs = std::max (0.0f, std::min (delayTimeMs, MaxDelayTimeMs));

            delayMs = delayTimeMs;

            // Update delay samples if we have a valid sample rate
            if (getSampleRate() > 0.0)
            {
                delaySamples = static_cast<int> ((delayMs / 1000.0f) * getSampleRate());
            }
        }

        /**
         * @brief Set the feedback amount
         * 
         * @param feedbackAmount The new feedback amount (0.0 to 1.0)
         */
        void setFeedback (float feedbackAmount)
        {
            // Clamp to valid range
            feedback = std::max (0.0f, std::min (feedbackAmount, 1.0f));
        }

        /**
         * @brief Set the dry/wet mix
         * 
         * @param mixAmount The new mix amount (0.0 = dry, 1.0 = wet)
         */
        void setMix (float mixAmount)
        {
            // Clamp to valid range
            mix = std::max (0.0f, std::min (mixAmount, 1.0f));
        }

        /**
         * @brief Set the current channel index
         * 
         * @param index The new channel index (0 or greater)
         */
        void setChannelIndex (int index)
        {
            if (index < 0)
                index = 0;

            channelIndex = index;
        }

        /**
         * @brief Get the current channel index
         * 
         * @return int The current channel index
         */
        int getChannelIndex() const
        {
            return channelIndex;
        }

        /**
         * @brief Get the current delay time in milliseconds
         * 
         * @return float The current delay time in milliseconds
         */
        float getDelayTime() const
        {
            return delayMs;
        }

        /**
         * @brief Get the current feedback amount
         * 
         * @return float The current feedback amount (0.0 to 1.0)
         */
        float getFeedback() const
        {
            return feedback;
        }

        /**
         * @brief Get the current dry/wet mix
         * 
         * @return float The current mix amount (0.0 = dry, 1.0 = wet)
         */
        float getMix() const
        {
            return mix;
        }

    private:
        std::atomic<float> delayMs; // Delay time in milliseconds
        std::atomic<float> feedback; // Feedback amount (0.0 to 1.0)
        std::atomic<float> mix; // Dry/wet mix (0.0 = dry, 1.0 = wet)
        std::atomic<int> delaySamples; // Delay time in samples
        std::atomic<int> channelIndex { 0 }; // Current channel index

        RingBuffer<float> delayBuffer; // The delay line buffer
    };
} // namespace PlayfulTones::DspToolBox
