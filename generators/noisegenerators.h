#pragma once
#include <random>

namespace PlayfulTones::DspToolBox
{
    /*
        * Generate white noise and fill the audio buffer with it.
        * This function generates white noise samples and fills the provided audio buffer.
        * The generated noise is uniform across all channels.
        *
        * @param buffer The audio buffer to be filled with white noise samples.
        * @param numChannels The number of channels in the audio buffer.
        * @param numFrames The number of frames in the audio buffer.
        * @param gain The gain factor to apply to the generated noise. Default is 1.0f.
    */
    inline void generateWhiteNoise (float** buffer, int numChannels, int numFrames, float gain = 1.f)
    {
        static std::random_device rd;
        static std::mt19937 gen (rd());
        static std::uniform_real_distribution<float> dist (-1.0f, 1.0f);

        for (int i = 0; i < numFrames; i++)
        {
            float sample = dist (gen) * gain;
            for (int ch = 0; ch < numChannels; ch++)
            {
                buffer[ch][i] = sample;
            }
        }
    }
} // namespace PlayfulTones::DspToolBox