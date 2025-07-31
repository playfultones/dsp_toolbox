/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once
#include "../core/audio_buffer.h"
#include <cmath>
#include <iostream>
#include <type_traits>

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief Validates that two audio buffers have valid and matching dimensions.
     * 
     * @tparam AudioBufferA Must satisfy AudioBufferType concept
     * @tparam AudioBufferB Must satisfy AudioBufferType concept
     * @param bufferA First buffer to validate
     * @param bufferB Second buffer to validate 
     * @return true if buffer dimensions are valid and match, false otherwise
     */
    template <AudioBufferType AudioBufferA, AudioBufferType AudioBufferB>
    constexpr bool validateBufferDimensions(const AudioBufferA& bufferA, const AudioBufferB& bufferB) noexcept
    {
        // Check that both buffers have channels and that dimensions match
        return bufferA.size() > 0 && 
               bufferA.size() == bufferB.size() &&
               (bufferA.size() == 0 || bufferA[0].size() > 0) &&
               (bufferA.size() == 0 || bufferA[0].size() == bufferB[0].size());
    }

    /**
     * @brief Compare two audio buffers for equality within a specified threshold.
     * 
     * Uses compile-time optimization and follows real-time safety principles.
     * 
     * @tparam AudioBufferA Must satisfy AudioBufferType concept
     * @tparam AudioBufferB Must satisfy AudioBufferType concept
     * @tparam SampleType The sample type for threshold comparison
     * @param bufferA First buffer to compare
     * @param bufferB Second buffer to compare
     * @param threshold Maximum allowed difference between samples
     * @param printMismatches Whether to print details about mismatches (default: true)
     * @return true if buffers match within threshold, false otherwise
     */
    template <AudioBufferType AudioBufferA, AudioBufferType AudioBufferB, typename SampleType = float>
    [[nodiscard]] bool compareAudioBuffers(const AudioBufferA& bufferA,
                                          const AudioBufferB& bufferB,
                                          SampleType threshold = SampleType{0.000001},
                                          bool printMismatches = true)
    {
        // First validate buffer dimensions
        if (!validateBufferDimensions(bufferA, bufferB))
        {
            if (printMismatches)
            {
                std::cout << "Buffer dimension mismatch: BufferA channels=" << bufferA.size()
                          << ", BufferB channels=" << bufferB.size() << std::endl;
            }
            return false;
        }

        const size_t numChannels = bufferA.size();
        
        for (size_t ch = 0; ch < numChannels; ++ch)
        {
            const auto& channelA = bufferA[ch];
            const auto& channelB = bufferB[ch];
            
            const size_t numFrames = channelA.size();

            for (size_t i = 0; i < numFrames; ++i)
            {
                const auto diff = std::abs(static_cast<SampleType>(channelA[i]) - static_cast<SampleType>(channelB[i]));
                
                if (diff > threshold)
                {
                    if (printMismatches)
                    {
                        std::cout << "Mismatch at channel " << ch << ", frame " << i
                                  << ". Expected: " << channelB[i]
                                  << ", Got: " << channelA[i] 
                                  << ", Diff: " << diff << std::endl;
                    }
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * @brief Fast buffer comparison optimized for exact matches.
     * 
     * Uses std::equal for potentially better optimization than manual loops.
     * 
     * @tparam AudioBufferA Must satisfy AudioBufferType concept
     * @tparam AudioBufferB Must satisfy AudioBufferType concept
     * @param bufferA First buffer to compare
     * @param bufferB Second buffer to compare
     * @return true if buffers are exactly equal, false otherwise
     */
    template <AudioBufferType AudioBufferA, AudioBufferType AudioBufferB>
    [[nodiscard]] constexpr bool compareAudioBuffersExact(const AudioBufferA& bufferA,
                                                          const AudioBufferB& bufferB) noexcept
    {
        if (!validateBufferDimensions(bufferA, bufferB))
            return false;

        const size_t numChannels = bufferA.size();
        
        for (size_t ch = 0; ch < numChannels; ++ch)
        {
            if (!std::equal(bufferA[ch].begin(), bufferA[ch].end(), bufferB[ch].begin()))
                return false;
        }
        
        return true;
    }

    /**
     * @brief Validate a single buffer has valid dimensions.
     * 
     * @tparam AudioBuffer Must satisfy AudioBufferType concept
     * @param buffer Buffer to validate
     * @return true if buffer has valid dimensions, false otherwise
     */
    template <AudioBufferType AudioBuffer>
    constexpr bool validateSingleBuffer(const AudioBuffer& buffer) noexcept
    {
        return buffer.size() > 0 && (buffer.size() == 0 || buffer[0].size() > 0);
    }

    /**
     * @brief Check if buffer contains any NaN or infinite values.
     * 
     * Critical for debugging audio processing chains.
     * 
     * @tparam AudioBuffer Must satisfy AudioBufferType concept
     * @param buffer Buffer to check
     * @param printDetails Whether to print details about invalid values
     * @return true if buffer contains valid finite values, false otherwise
     */
    template <AudioBufferType AudioBuffer>
    [[nodiscard]] bool validateFiniteValues(const AudioBuffer& buffer, bool printDetails = true)
    {
        using SampleType = typename AudioBuffer::value_type::value_type;
        
        if constexpr (std::is_floating_point_v<SampleType>)
        {
            const size_t numChannels = buffer.size();
            
            for (size_t ch = 0; ch < numChannels; ++ch)
            {
                const auto& channel = buffer[ch];
                const size_t numFrames = channel.size();
                
                for (size_t i = 0; i < numFrames; ++i)
                {
                    const SampleType sample = channel[i];
                    
                    if (!std::isfinite(sample))
                    {
                        if (printDetails)
                        {
                            std::cout << "Invalid value at channel " << ch << ", frame " << i
                                      << ". Value: " << sample;
                            if (std::isnan(sample))
                                std::cout << " (NaN)";
                            else if (std::isinf(sample))
                                std::cout << " (Infinite)";
                            std::cout << std::endl;
                        }
                        return false;
                    }
                }
            }
        }
        
        return true;
    }

} // namespace PlayfulTones::DspToolBox
