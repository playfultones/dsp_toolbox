/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once
#include <cmath>

namespace PlayfulTones::DspToolBox
{
    /**
     * Counts the number of zero crossings in a multi-channel audio buffer.
     * 
     * @param buffer Pointer to an array of audio buffers, one for each channel.
     * @param numChannels Number of audio channels.
     * @param numFrames Number of frames (samples) in each channel.
     * @param hysteresis Optional hysteresis value to filter out noise.
     * @return The number of zero crossings detected across all channels.
     */
    inline int countZeroCrossings (float** buffer, int numChannels, int numFrames, float hysteresis = 0.0f)
    {
        int zeroCrossings = 0;

        for (int ch = 0; ch < numChannels; ch++)
        {
            float prevSample = buffer[ch][0];

            for (int i = 1; i < numFrames; i++)
            {
                float currentSample = buffer[ch][i];

                // Detect zero crossing by sign change
                if (std::signbit (prevSample) != std::signbit (currentSample))
                {
                    // Linear interpolation to get a more precise crossing point
                    float t = -prevSample / (currentSample - prevSample);

                    // Validate the crossing
                    if (hysteresis == 0.0f)
                    {
                        // For basic tests, just count sign changes
                        zeroCrossings++;
                    }
                    else if (t >= 0.0f && t <= 1.0f)
                    {
                        // For real signals, ensure we have a valid crossing
                        float slope = currentSample - prevSample;
                        if (std::abs (slope) > hysteresis) // Check if we're moving fast enough through zero
                        {
                            zeroCrossings++;
                        }
                    }
                }

                prevSample = currentSample;
            }
        }

        return zeroCrossings;
    }
} // namespace PlayfulTones::DspToolBox