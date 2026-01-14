/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/core/buffer_view.hpp"
#include "dsp_toolbox/core/concepts.hpp"

#include <array>
#include <cstddef>

namespace PlayfulTones::DspToolbox
{

    /**
     * @brief Compile-time sized audio buffer with static storage.
     *
     * Owning buffer for embedded/constrained systems where allocation-free
     * operation is required. All storage is determined at compile time.
     *
     * @tparam Channels Number of audio channels
     * @tparam Samples Number of samples per channel
     * @tparam SampleType Numeric type for audio samples (default: float)
     */
    template <std::size_t Channels, std::size_t Samples, Numeric SampleType = float>
    class StaticAudioBuffer
    {
        static_assert (Channels > 0, "StaticAudioBuffer must have at least one channel");
        static_assert (Samples > 0, "StaticAudioBuffer must have at least one sample");

        std::array<std::array<SampleType, Samples>, Channels> data_ {};
        mutable std::array<SampleType*, Channels> channelPointers_ {};
        mutable bool pointersInitialized_ { false };

        constexpr void initializePointers() const noexcept
        {
            if (!pointersInitialized_)
            {
                for (std::size_t ch = 0; ch < Channels; ++ch)
                {
                    channelPointers_[ch] = const_cast<SampleType*> (data_[ch].data());
                }
                pointersInitialized_ = true;
            }
        }

    public:
        constexpr StaticAudioBuffer() = default;

        /**
         * @brief Construct with initial value for all samples.
         *
         * @param initialValue Value to fill all samples with
         */
        constexpr explicit StaticAudioBuffer (SampleType initialValue) noexcept
        {
            fill (initialValue);
        }

        /**
         * @brief Get number of channels.
         */
        [[nodiscard]] static constexpr std::size_t getNumChannels() noexcept
        {
            return Channels;
        }

        /**
         * @brief Get number of samples per channel.
         */
        [[nodiscard]] static constexpr std::size_t getNumSamples() noexcept
        {
            return Samples;
        }

        /**
         * @brief Get read-only pointer to channel data.
         *
         * @param channel Channel index
         * @return Const pointer to channel sample data
         */
        [[nodiscard]] constexpr const SampleType* getReadPointer (std::size_t channel) const noexcept
        {
            return data_[channel].data();
        }

        /**
         * @brief Get writable pointer to channel data.
         *
         * @param channel Channel index
         * @return Pointer to channel sample data
         */
        [[nodiscard]] constexpr SampleType* getWritePointer (std::size_t channel) noexcept
        {
            return data_[channel].data();
        }

        /**
         * @brief Get a non-owning BufferView over this buffer.
         *
         * @return BufferView providing access to all channels and samples
         */
        [[nodiscard]] BufferView<SampleType> getView() noexcept
        {
            initializePointers();
            return BufferView<SampleType> (channelPointers_.data(), Channels, Samples);
        }

        /**
         * @brief Get a non-owning const BufferView over this buffer.
         *
         * @return Const BufferView providing read access to all channels and samples
         */
        [[nodiscard]] const BufferView<SampleType> getView() const noexcept
        {
            initializePointers();
            return BufferView<SampleType> (channelPointers_.data(), Channels, Samples);
        }

        /**
         * @brief Access sample at specific channel and index.
         *
         * @param channel Channel index
         * @param sample Sample index
         * @return Reference to the sample
         */
        [[nodiscard]] constexpr SampleType& operator() (std::size_t channel, std::size_t sample) noexcept
        {
            return data_[channel][sample];
        }

        /**
         * @brief Access sample at specific channel and index (const).
         *
         * @param channel Channel index
         * @param sample Sample index
         * @return Const reference to the sample
         */
        [[nodiscard]] constexpr const SampleType& operator() (std::size_t channel, std::size_t sample) const noexcept
        {
            return data_[channel][sample];
        }

        /**
         * @brief Clear all samples to zero.
         */
        constexpr void clear() noexcept
        {
            fill (SampleType {});
        }

        /**
         * @brief Fill all samples with a value.
         *
         * @param value Value to fill with
         */
        constexpr void fill (SampleType value) noexcept
        {
            for (std::size_t ch = 0; ch < Channels; ++ch)
            {
                for (std::size_t i = 0; i < Samples; ++i)
                {
                    data_[ch][i] = value;
                }
            }
        }

        /**
         * @brief Get direct access to underlying channel array.
         *
         * @param channel Channel index
         * @return Reference to the channel's sample array
         */
        [[nodiscard]] constexpr std::array<SampleType, Samples>& getChannelArray (std::size_t channel) noexcept
        {
            return data_[channel];
        }

        /**
         * @brief Get direct access to underlying channel array (const).
         *
         * @param channel Channel index
         * @return Const reference to the channel's sample array
         */
        [[nodiscard]] constexpr const std::array<SampleType, Samples>& getChannelArray (std::size_t channel) const noexcept
        {
            return data_[channel];
        }
    };

} // namespace PlayfulTones::DspToolbox
