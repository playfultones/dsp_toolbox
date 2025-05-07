/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once
#include <algorithm>
#include <cmath>

namespace PlayfulTones::DspToolBox
{
    static constexpr auto kNoReduction = 1.0f;
    static constexpr auto kSmallValue = 1e-6f;
    /*
        * Apply a simple limiter to the audio buffer.
        * This function applies a hard limiter to the audio samples in the buffer
        * ensuring that no sample exceeds the specified ceiling value.
        *
        * @param buffer The audio buffer containing the samples to be limited.
        * @param numChannels The number of channels in the audio buffer.
        * @param numFrames The number of frames in the audio buffer.
        * @param ceiling The maximum allowed amplitude after limiting. Default is 1.0f.
    */
    inline void applyLimiter (float** buffer, int numChannels, int numFrames, float ceiling = 1.f)
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