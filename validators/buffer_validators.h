/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once
#include "../core/audio_buffer.h"
#include <cmath>
#include <iostream>
#include <vector>

namespace PlayfulTones::DspToolBox
{
    /**
     * Validates that two audio buffers have valid dimensions.
     * 
     * @param bufferA First buffer to validate
     * @param bufferB Second buffer to validate 
     * @return true if buffer dimensions are valid and match, false otherwise
     */
    inline bool validateBufferDimensions (const AudioBuffer& bufferA, const AudioBuffer& bufferB)
    {
        return bufferA.getNumChannels() > 0
               && bufferA.getNumFrames() > 0
               && bufferA.getNumChannels() == bufferB.getNumChannels()
               && bufferA.getNumFrames() == bufferB.getNumFrames();
    }

    /**
     * Compare two audio buffers for equality within a specified threshold.
     * 
     * @param bufferA First buffer to compare
     * @param bufferB Second buffer to compare
     * @param threshold Maximum allowed difference between samples (default: 0.000001f)
     * @param printMismatches Whether to print details about mismatches (default: true)
     * @return true if buffers match within threshold, false otherwise
     */
    inline bool compareAudioBuffers (const AudioBuffer& bufferA,
        const AudioBuffer& bufferB,
        float threshold = 0.000001f,
        bool printMismatches = true)
    {
        // First validate buffer dimensions
        if (!validateBufferDimensions (bufferA, bufferB))
            return false;

        const auto numChannels = bufferA.getNumChannels();
        const auto numFrames = bufferA.getNumFrames();

        for (int ch = 0; ch < numChannels; ++ch)
        {
            const auto* channelA = bufferA.getChannelPointer (ch);
            const auto* channelB = bufferB.getChannelPointer (ch);

            for (int i = 0; i < numFrames; ++i)
            {
                if (std::abs (channelA[i] - channelB[i]) > threshold)
                {
                    if (printMismatches)
                    {
                        std::cout << "Mismatch at channel " << ch << ", frame " << i
                                  << ". Expected: " << channelB[i]
                                  << ", Got: " << channelA[i] << std::endl;
                    }
                    return false;
                }
            }
        }
        return true;
    }

    // Keep the raw pointer versions for backward compatibility
    inline bool validateBufferDimensions (float* const* bufferA,
        float* const* bufferB,
        int numChannels,
        int numFrames)
    {
        // Check for null pointers and valid dimensions
        if (!bufferA || !bufferB || numChannels <= 0 || numFrames <= 0)
            return false;

        // Validate channel pointers
        for (int ch = 0; ch < numChannels; ++ch)
        {
            if (!bufferA[ch] || !bufferB[ch])
                return false;
        }

        return true;
    }

    inline bool compareAudioBuffers (float* const* bufferA,
        float* const* bufferB,
        int numChannels,
        int numFrames,
        float threshold = 0.000001f,
        bool printMismatches = true)
    {
        // First validate buffer pointers
        if (!validateBufferDimensions (bufferA, bufferB, numChannels, numFrames))
            return false;

        for (int ch = 0; ch < numChannels; ++ch)
        {
            for (int i = 0; i < numFrames; ++i)
            {
                if (std::abs (bufferA[ch][i] - bufferB[ch][i]) > threshold)
                {
                    if (printMismatches)
                    {
                        std::cout << "Mismatch at channel " << ch << ", frame " << i
                                  << ". Expected: " << bufferB[ch][i]
                                  << ", Got: " << bufferA[ch][i] << std::endl;
                    }
                    return false;
                }
            }
        }
        return true;
    }
} // namespace PlayfulTones::DspToolBox
