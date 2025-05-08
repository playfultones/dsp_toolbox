/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once
#include "../core/constants.h"
#include <cmath>

namespace PlayfulTones::DspToolBox
{
    inline void generateSineWave (float** buffer, int numChannels, int numFrames, float frequency, float sampleRate, float& phase)
    {
        const auto phaseDelta = (float) (Constants::twoPi * frequency / sampleRate);
        for (int i = 0; i < numFrames; i++)
        {
            const auto sample = std::sin (phase);
            phase = std::fmod (phase + phaseDelta, Constants::twoPi);

            for (int ch = 0; ch < numChannels; ch++)
            {
                buffer[ch][i] = sample;
            }
        }
    }
} // namespace PlayfulTones::DspToolBox