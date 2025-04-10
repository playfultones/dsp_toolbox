#pragma once

namespace PlayfulTones::DspToolBox
{
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
    static inline void applyLimiter (float** buffer, int numChannels, int numFrames, float threshold = 1.f, float ceiling = 1.f)
    {
        for (int i = 0; i < numFrames; i++)
        {
            for (int ch = 0; ch < numChannels; ch++)
            {
                float sample = buffer[ch][i];
                float absValue = std::abs (sample);

                if (absValue > threshold)
                {
                    float gainReduction = ceiling / absValue;
                    buffer[ch][i] = sample * gainReduction;
                }
            }
        }
    }
} // namespace PlayfulTones::DspToolBox