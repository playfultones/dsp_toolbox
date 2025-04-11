#pragma once
#include <algorithm>
#include <cmath>

namespace PlayfulTones::DspToolBox
{
    static constexpr auto kNoReduction = 1.0f;
    static constexpr auto kSmallValue = 1e-6f;
    /*
        * Apply a simple limiter to the audio buffer.
        * This function applies a hard limiter to the audio samples in the buffer.
        * It reduces the amplitude of samples that exceed a specified threshold.
        *
        * @param buffer The audio buffer containing the samples to be limited.
        * @param numChannels The number of channels in the audio buffer.
        * @param numFrames The number of frames in the audio buffer.
        * @param threshold The threshold above which samples will be limited. Default is 1.0f.
        * @param ceiling The maximum allowed amplitude after limiting. Default is 1.0f.
    */
    inline void applyLimiter (float** buffer, int numChannels, int numFrames, float threshold = 1.f, float ceiling = 1.f)
    {
        for (int ch = 0; ch < numChannels; ch++)
        {
            float* channelBuffer = buffer[ch];

            for (int i = 0; i < numFrames; i++)
            {
                float sample = channelBuffer[i];
                float absValue = std::abs (sample);
                float gainReduction = std::min (kNoReduction, ceiling / (absValue + kSmallValue));
                channelBuffer[i] = sample * gainReduction;
            }
        }
    }
} // namespace PlayfulTones::DspToolBox