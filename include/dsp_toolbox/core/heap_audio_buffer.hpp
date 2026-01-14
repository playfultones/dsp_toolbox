/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

// HeapAudioBuffer is only available on desktop platforms where dynamic
// allocation is acceptable. Define DSP_TOOLBOX_NO_HEAP_BUFFER to disable.
#ifndef DSP_TOOLBOX_NO_HEAP_BUFFER

    #include "dsp_toolbox/core/buffer_view.hpp"
    #include "dsp_toolbox/core/concepts.hpp"

    #include <algorithm>
    #include <cstddef>
    #include <memory>
    #include <vector>

namespace PlayfulTones::DspToolbox
{

    /**
     * @brief Runtime-sized audio buffer with heap storage.
     *
     * Owning buffer for desktop applications where runtime sizing is needed.
     * Uses heap allocation - not suitable for embedded/constrained systems.
     *
     * @tparam SampleType Numeric type for audio samples (default: float)
     */
    template <Numeric SampleType = float>
    class HeapAudioBuffer
    {
        std::size_t numChannels_ {};
        std::size_t numSamples_ {};
        std::vector<SampleType> interleavedData_ {};
        std::vector<SampleType*> channelPointers_ {};

        void initializePointers() noexcept
        {
            channelPointers_.resize (numChannels_);
            for (std::size_t ch = 0; ch < numChannels_; ++ch)
            {
                channelPointers_[ch] = interleavedData_.data() + (ch * numSamples_);
            }
        }

    public:
        HeapAudioBuffer() = default;

        /**
         * @brief Construct buffer with specified dimensions.
         *
         * @param numChannels Number of audio channels
         * @param numSamples Number of samples per channel
         */
        HeapAudioBuffer (std::size_t numChannels, std::size_t numSamples)
            : numChannels_ (numChannels), numSamples_ (numSamples), interleavedData_ (numChannels * numSamples, SampleType {})
        {
            initializePointers();
        }

        /**
         * @brief Construct buffer with specified dimensions and initial value.
         *
         * @param numChannels Number of audio channels
         * @param numSamples Number of samples per channel
         * @param initialValue Value to fill all samples with
         */
        HeapAudioBuffer (std::size_t numChannels, std::size_t numSamples, SampleType initialValue)
            : numChannels_ (numChannels), numSamples_ (numSamples), interleavedData_ (numChannels * numSamples, initialValue)
        {
            initializePointers();
        }

        // Copy operations
        HeapAudioBuffer (const HeapAudioBuffer& other)
            : numChannels_ (other.numChannels_), numSamples_ (other.numSamples_), interleavedData_ (other.interleavedData_)
        {
            initializePointers();
        }

        HeapAudioBuffer& operator= (const HeapAudioBuffer& other)
        {
            if (this != &other)
            {
                numChannels_ = other.numChannels_;
                numSamples_ = other.numSamples_;
                interleavedData_ = other.interleavedData_;
                initializePointers();
            }
            return *this;
        }

        // Move operations
        HeapAudioBuffer (HeapAudioBuffer&& other) noexcept
            : numChannels_ (other.numChannels_), numSamples_ (other.numSamples_), interleavedData_ (std::move (other.interleavedData_))
        {
            initializePointers();
            other.numChannels_ = 0;
            other.numSamples_ = 0;
            other.channelPointers_.clear();
        }

        HeapAudioBuffer& operator= (HeapAudioBuffer&& other) noexcept
        {
            if (this != &other)
            {
                numChannels_ = other.numChannels_;
                numSamples_ = other.numSamples_;
                interleavedData_ = std::move (other.interleavedData_);
                initializePointers();
                other.numChannels_ = 0;
                other.numSamples_ = 0;
                other.channelPointers_.clear();
            }
            return *this;
        }

        ~HeapAudioBuffer() = default;

        /**
         * @brief Get number of channels.
         */
        [[nodiscard]] std::size_t getNumChannels() const noexcept
        {
            return numChannels_;
        }

        /**
         * @brief Get number of samples per channel.
         */
        [[nodiscard]] std::size_t getNumSamples() const noexcept
        {
            return numSamples_;
        }

        /**
         * @brief Check if buffer is empty (zero channels or samples).
         */
        [[nodiscard]] bool isEmpty() const noexcept
        {
            return numChannels_ == 0 || numSamples_ == 0;
        }

        /**
         * @brief Get read-only pointer to channel data.
         *
         * @param channel Channel index
         * @return Const pointer to channel sample data
         */
        [[nodiscard]] const SampleType* getReadPointer (std::size_t channel) const noexcept
        {
            return interleavedData_.data() + (channel * numSamples_);
        }

        /**
         * @brief Get writable pointer to channel data.
         *
         * @param channel Channel index
         * @return Pointer to channel sample data
         */
        [[nodiscard]] SampleType* getWritePointer (std::size_t channel) noexcept
        {
            return interleavedData_.data() + (channel * numSamples_);
        }

        /**
         * @brief Get a non-owning BufferView over this buffer.
         *
         * @return BufferView providing access to all channels and samples
         */
        [[nodiscard]] BufferView<SampleType> getView() noexcept
        {
            if (isEmpty())
            {
                return BufferView<SampleType> {};
            }
            return BufferView<SampleType> (channelPointers_.data(), numChannels_, numSamples_);
        }

        /**
         * @brief Get a non-owning const BufferView over this buffer.
         *
         * @return Const BufferView providing read access to all channels and samples
         */
        [[nodiscard]] const BufferView<SampleType> getView() const noexcept
        {
            if (isEmpty())
            {
                return BufferView<SampleType> {};
            }
            return BufferView<SampleType> (const_cast<SampleType**> (channelPointers_.data()), numChannels_, numSamples_);
        }

        /**
         * @brief Access sample at specific channel and index.
         *
         * @param channel Channel index
         * @param sample Sample index
         * @return Reference to the sample
         */
        [[nodiscard]] SampleType& operator() (std::size_t channel, std::size_t sample) noexcept
        {
            return interleavedData_[channel * numSamples_ + sample];
        }

        /**
         * @brief Access sample at specific channel and index (const).
         *
         * @param channel Channel index
         * @param sample Sample index
         * @return Const reference to the sample
         */
        [[nodiscard]] const SampleType& operator() (std::size_t channel, std::size_t sample) const noexcept
        {
            return interleavedData_[channel * numSamples_ + sample];
        }

        /**
         * @brief Clear all samples to zero.
         */
        void clear() noexcept
        {
            std::fill (interleavedData_.begin(), interleavedData_.end(), SampleType {});
        }

        /**
         * @brief Fill all samples with a value.
         *
         * @param value Value to fill with
         */
        void fill (SampleType value) noexcept
        {
            std::fill (interleavedData_.begin(), interleavedData_.end(), value);
        }

        /**
         * @brief Resize the buffer.
         *
         * Contents are cleared after resize.
         *
         * @param numChannels New number of channels
         * @param numSamples New number of samples per channel
         *
         * @note May allocate memory. Not real-time safe.
         */
        void resize (std::size_t numChannels, std::size_t numSamples)
        {
            numChannels_ = numChannels;
            numSamples_ = numSamples;
            interleavedData_.assign (numChannels * numSamples, SampleType {});
            initializePointers();
        }
    };

} // namespace PlayfulTones::DspToolbox

#endif // DSP_TOOLBOX_NO_HEAP_BUFFER
