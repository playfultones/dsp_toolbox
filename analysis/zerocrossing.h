/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once
#include <cmath>
#include <span>
#include <type_traits>

namespace PlayfulTones::DspToolBox
{
    /**
     * Specifies which type of zero crossings to count
     */
    enum class ZeroCrossingDirection {
        All, // Count both positive and negative going crossings
        Positive, // Count only positive going crossings (from negative to positive)
        Negative // Count only negative going crossings (from positive to negative)
    };

    /**
     * Concept to validate audio buffer types
     */
    template <typename T>
    concept AudioBufferType = requires (T buffer) {
        buffer.size();
        buffer[0];
        typename T::value_type;
    };

    /**
     * Zero crossing analysis tools - stateless design following DSP toolbox patterns
     */
    class ZeroCrossingAnalyzer
    {
    public:
        /**
         * Counts zero crossings in a single channel buffer
         * 
         * @param buffer Single channel audio buffer
         * @param direction Which zero crossings to count
         * @param hysteresis Threshold to filter out noise
         * @return Number of zero crossings detected
         */
        template <typename SampleType, size_t BlockSize>
        [[nodiscard]] static constexpr size_t countZeroCrossings (
            std::span<const SampleType, BlockSize> buffer,
            ZeroCrossingDirection direction = ZeroCrossingDirection::All,
            SampleType hysteresis = SampleType { 0 }) noexcept
        {
            static_assert (std::is_floating_point_v<SampleType>, "SampleType must be floating point");

            if constexpr (BlockSize <= 1)
            {
                return 0;
            }

            size_t zeroCrossings = 0;
            SampleType prevSample = buffer[0];

            for (size_t i = 1; i < BlockSize; ++i)
            {
                const SampleType currentSample = buffer[i];

                // Detect zero crossing by sign change
                if (std::signbit (prevSample) != std::signbit (currentSample))
                {
                    const bool isPositiveGoing = prevSample < SampleType { 0 } && currentSample > SampleType { 0 };

                    // Check if we should count this crossing based on direction
                    const bool shouldCount =
                        direction == ZeroCrossingDirection::All || (direction == ZeroCrossingDirection::Positive && isPositiveGoing) || (direction == ZeroCrossingDirection::Negative && !isPositiveGoing);

                    if (shouldCount)
                    {
                        if (hysteresis == SampleType { 0 })
                        {
                            ++zeroCrossings;
                        }
                        else
                        {
                            // Check slope magnitude against hysteresis
                            const SampleType slope = std::abs (currentSample - prevSample);
                            if (slope > hysteresis)
                            {
                                ++zeroCrossings;
                            }
                        }
                    }
                }

                prevSample = currentSample;
            }

            return zeroCrossings;
        }

        /**
         * Counts zero crossings in a multi-channel audio buffer
         * 
         * @param buffer Multi-channel audio buffer
         * @param direction Which zero crossings to count
         * @param hysteresis Threshold to filter out noise
         * @return Total number of zero crossings across all channels
         */
        template <AudioBufferType AudioBuffer, typename SampleType = typename AudioBuffer::value_type::value_type>
        [[nodiscard]] static constexpr size_t countZeroCrossings (
            const AudioBuffer& buffer,
            ZeroCrossingDirection direction = ZeroCrossingDirection::All,
            SampleType hysteresis = SampleType { 0 }) noexcept
        {
            static_assert (std::is_floating_point_v<SampleType>, "SampleType must be floating point");

            size_t totalCrossings = 0;

            for (const auto& channel : buffer)
            {
                totalCrossings += countZeroCrossings (channel, direction, hysteresis);
            }

            return totalCrossings;
        }

        /**
         * Calculates zero-crossing rate for a single channel
         * 
         * @param buffer Single channel audio buffer
         * @param direction Which zero crossings to count
         * @param hysteresis Threshold to filter out noise
         * @return Zero-crossing rate (crossings per sample)
         */
        template <typename SampleType, size_t BlockSize>
        [[nodiscard]] static constexpr SampleType calculateZeroCrossingRate (
            std::span<const SampleType, BlockSize> buffer,
            ZeroCrossingDirection direction = ZeroCrossingDirection::All,
            SampleType hysteresis = SampleType { 0 }) noexcept
        {
            static_assert (std::is_floating_point_v<SampleType>, "SampleType must be floating point");

            if constexpr (BlockSize == 0)
            {
                return SampleType { 0 };
            }

            const size_t crossings = countZeroCrossings (buffer, direction, hysteresis);
            return static_cast<SampleType> (crossings) / static_cast<SampleType> (BlockSize);
        }

        /**
         * Calculates zero-crossing rate for a multi-channel buffer
         * 
         * @param buffer Multi-channel audio buffer
         * @param direction Which zero crossings to count
         * @param hysteresis Threshold to filter out noise
         * @return Average zero-crossing rate across all channels
         */
        template <AudioBufferType AudioBuffer, typename SampleType = typename AudioBuffer::value_type::value_type>
        [[nodiscard]] static constexpr SampleType calculateZeroCrossingRate (
            const AudioBuffer& buffer,
            ZeroCrossingDirection direction = ZeroCrossingDirection::All,
            SampleType hysteresis = SampleType { 0 }) noexcept
        {
            static_assert (std::is_floating_point_v<SampleType>, "SampleType must be floating point");

            if (buffer.empty())
            {
                return SampleType { 0 };
            }

            const size_t totalCrossings = countZeroCrossings (buffer, direction, hysteresis);
            const size_t totalSamples = buffer.size() * buffer[0].size();

            return static_cast<SampleType> (totalCrossings) / static_cast<SampleType> (totalSamples);
        }

        /**
         * Estimates fundamental frequency from zero-crossing rate
         * 
         * @param zcr Zero-crossing rate
         * @param sampleRate Sample rate in Hz
         * @return Estimated frequency in Hz
         */
        template <typename SampleType>
        [[nodiscard]] static constexpr SampleType estimateFrequencyFromZCR (
            SampleType zcr,
            SampleType sampleRate) noexcept
        {
            static_assert (std::is_floating_point_v<SampleType>, "SampleType must be floating point");
            return (zcr * sampleRate) / SampleType { 2 };
        }
    };

} // namespace PlayfulTones::DspToolBox
