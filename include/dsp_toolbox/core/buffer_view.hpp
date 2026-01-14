/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/core/concepts.hpp"
#include "dsp_toolbox/core/crv.hpp"
#include "dsp_toolbox/simd/simd.hpp"

#include <concepts>
#include <cstddef>
#include <type_traits>

namespace PlayfulTones::DspToolbox
{

    /**
     * @brief Non-owning view over audio buffer data.
     *
     * Provides channel access for audio processing without owning the data.
     * Compatible with JUCE AudioBuffer::getArrayOfWritePointers().
     *
     * Supports:
     * - Read/write pointer access per channel
     * - Sub-view slicing for sample-accurate processing
     * - Utility operations (clear, gain, copy, add)
     *
     * @tparam SampleType Numeric type for audio samples (default: float)
     */
    template <Numeric SampleType = float>
    class BufferView
    {
        SampleType** data_ {};
        std::size_t numChannels_ {};
        std::size_t numSamples_ {};
        std::size_t startOffset_ {};

    public:
        constexpr BufferView() = default;

        /**
         * @brief Construct a BufferView from channel pointers.
         *
         * @param channelData Array of pointers to channel data
         * @param numChannels Number of channels
         * @param numSamples Number of samples per channel
         */
        constexpr BufferView (SampleType** channelData,
            std::size_t numChannels,
            std::size_t numSamples) noexcept
            : data_ (channelData), numChannels_ (numChannels), numSamples_ (numSamples), startOffset_ (0)
        {
        }

        /**
         * @brief Construct a sub-view from an existing BufferView.
         *
         * Creates a view into a portion of the buffer for sample-accurate processing.
         *
         * @param other Source BufferView
         * @param startSample Starting sample offset
         * @param numSamples Number of samples in the sub-view
         */
        constexpr BufferView (const BufferView& other,
            std::size_t startSample,
            std::size_t numSamples) noexcept
            : data_ (other.data_), numChannels_ (other.numChannels_), numSamples_ (numSamples), startOffset_ (other.startOffset_ + startSample)
        {
        }

        /**
         * @brief Get writable pointer to channel data.
         *
         * @param channel Channel index
         * @return Pointer to the start of channel data (offset for sub-views)
         */
        [[nodiscard]] constexpr SampleType* getWritePointer (std::size_t channel) noexcept
        {
            return data_[channel] + startOffset_;
        }

        /**
         * @brief Get read-only pointer to channel data.
         *
         * @param channel Channel index
         * @return Const pointer to the start of channel data (offset for sub-views)
         */
        [[nodiscard]] constexpr const SampleType* getReadPointer (std::size_t channel) const noexcept
        {
            return data_[channel] + startOffset_;
        }

        /**
         * @brief Get number of channels in the buffer.
         */
        [[nodiscard]] constexpr std::size_t getNumChannels() const noexcept
        {
            return numChannels_;
        }

        /**
         * @brief Get number of samples per channel.
         */
        [[nodiscard]] constexpr std::size_t getNumSamples() const noexcept
        {
            return numSamples_;
        }

        /**
         * @brief Clear all samples to zero.
         *
         * Uses SIMD acceleration when SampleType is float or double.
         */
        void clear() noexcept
        {
            for (std::size_t ch = 0; ch < numChannels_; ++ch)
            {
                SampleType* ptr = getWritePointer (ch);
                if constexpr (std::floating_point<SampleType>)
                {
                    simd::clear (ptr, numSamples_);
                }
                else
                {
                    for (std::size_t i = 0; i < numSamples_; ++i)
                    {
                        ptr[i] = SampleType {};
                    }
                }
            }
        }

        /**
         * @brief Multiply all samples by a factor.
         *
         * Uses SIMD acceleration when SampleType is float or double.
         *
         * @param factor Multiplication factor (supports CV for compile-time optimization)
         */
        template <IsCRV<SampleType> FactorType>
        void multiply (FactorType factor) noexcept
        {
            if constexpr (isConstantOne<FactorType>)
            {
                return;
            }
            else if constexpr (isConstantZero<FactorType>)
            {
                clear();
            }
            else
            {
                for (std::size_t ch = 0; ch < numChannels_; ++ch)
                {
                    SampleType* ptr = getWritePointer (ch);
                    if constexpr (std::floating_point<SampleType>)
                    {
                        simd::multiply (ptr, numSamples_, factor.get());
                    }
                    else
                    {
                        for (std::size_t i = 0; i < numSamples_; ++i)
                        {
                            ptr[i] *= factor.get();
                        }
                    }
                }
            }
        }

        /**
         * @brief Multiply all samples by a factor.
         *
         * Convenience overload accepting raw SampleType.
         *
         * @param factor Multiplication factor
         */
        void multiply (SampleType factor) noexcept
        {
            multiply (RV<SampleType> { factor });
        }

        /**
         * @brief Copy samples from another buffer.
         *
         * Copies min(this->numSamples, source.numSamples) samples
         * for min(this->numChannels, source.numChannels) channels.
         * Uses SIMD acceleration when SampleType is float or double.
         *
         * @param source Source buffer to copy from
         */
        void copyFrom (const BufferView& source) noexcept
        {
            const std::size_t channelsToCopy =
                numChannels_ < source.numChannels_ ? numChannels_ : source.numChannels_;
            const std::size_t samplesToCopy =
                numSamples_ < source.numSamples_ ? numSamples_ : source.numSamples_;

            for (std::size_t ch = 0; ch < channelsToCopy; ++ch)
            {
                SampleType* dst = getWritePointer (ch);
                const SampleType* src = source.getReadPointer (ch);
                if constexpr (std::floating_point<SampleType>)
                {
                    simd::copy (dst, src, samplesToCopy);
                }
                else
                {
                    for (std::size_t i = 0; i < samplesToCopy; ++i)
                    {
                        dst[i] = src[i];
                    }
                }
            }
        }

        /**
         * @brief Add samples from another buffer.
         *
         * Adds min(this->numSamples, source.numSamples) samples
         * for min(this->numChannels, source.numChannels) channels.
         * Uses SIMD acceleration when SampleType is float or double.
         *
         * @param source Source buffer to add from
         */
        void addFrom (const BufferView& source) noexcept
        {
            const std::size_t channelsToAdd =
                numChannels_ < source.numChannels_ ? numChannels_ : source.numChannels_;
            const std::size_t samplesToAdd =
                numSamples_ < source.numSamples_ ? numSamples_ : source.numSamples_;

            for (std::size_t ch = 0; ch < channelsToAdd; ++ch)
            {
                SampleType* dst = getWritePointer (ch);
                const SampleType* src = source.getReadPointer (ch);
                if constexpr (std::floating_point<SampleType>)
                {
                    simd::add (dst, src, samplesToAdd);
                }
                else
                {
                    for (std::size_t i = 0; i < samplesToAdd; ++i)
                    {
                        dst[i] += src[i];
                    }
                }
            }
        }
    };

} // namespace PlayfulTones::DspToolbox
