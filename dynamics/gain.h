/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once
#include "../processors/processor.h"
#include <algorithm>
#include <cmath>

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief A simple gain processor with ramping capabilities.
     *
     * This class implements a simple gain processor that can apply a gain factor to audio samples.
     * It also supports ramping the gain over a specified duration.
     */
    class Gain : public Processor
    {
    public:
        Gain() : gain_ (1.0f), targetGain_ (1.0f), rampLengthSeconds_ (0.0f), rampLengthSamples_ (0), currentRampSample_ (0) {}

        void prepare (double /* sampleRate */, int /* maxFramesPerBlock */) override
        {
            reset();
        }

        void reset() override
        {
            gain_ = targetGain_;
            currentRampSample_ = 0;
        }

        void process (BufferView& buffer) override
        {
            const auto numFrames = buffer.getNumFrames();
            const auto numChannels = buffer.getNumChannels();

            if (rampLengthSamples_ > 0 && currentRampSample_ < rampLengthSamples_)
            {
                // Process with gain ramping
                float gainIncrement = (targetGain_ - gain_) / rampLengthSamples_;
                for (int i = 0; i < numFrames; ++i)
                {
                    if (currentRampSample_ < rampLengthSamples_)
                    {
                        gain_ += gainIncrement;
                        currentRampSample_++;
                    }

                    for (int ch = 0; ch < numChannels; ++ch)
                    {
                        buffer.getChannelPointer (ch)[i] *= gain_;
                    }
                }
            }
            else
            {
                // Process without ramping
                for (int ch = 0; ch < numChannels; ++ch)
                {
                    auto* channelData = buffer.getChannelPointer (ch);
                    for (int i = 0; i < numFrames; ++i)
                    {
                        channelData[i] *= gain_;
                    }
                }
            }
        }

        void setGain (float newGain)
        {
            targetGain_ = newGain;
            if (rampLengthSamples_ == 0)
            {
                gain_ = newGain;
            }
        }

        void setGainRampLength (float seconds)
        {
            rampLengthSeconds_ = seconds;
            rampLengthSamples_ = static_cast<int> (seconds * getSampleRate());
            currentRampSample_ = 0;
        }

        float getCurrentGain() const { return gain_; }
        float getTargetGain() const { return targetGain_; }
        float getRampLengthSeconds() const { return rampLengthSeconds_; }

    private:
        float gain_;
        float targetGain_;
        float rampLengthSeconds_;
        int rampLengthSamples_;
        int currentRampSample_;
    };
} // namespace PlayfulTones::DspToolBox