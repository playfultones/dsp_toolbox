#pragma once
#include <algorithm>
#include <stdexcept>
#include <vector>

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief A class for handling multi-channel audio buffer data.
     * 
     * This class provides a safe and efficient way to store and manipulate
     * multi-channel audio data. It handles memory management automatically
     * and provides bounds checking in debug builds.
     */
    class AudioBuffer
    {
    public:
        /**
         * @brief Construct an empty audio buffer
         */
        AudioBuffer() = default;

        /**
         * @brief Construct an audio buffer with specified dimensions
         * @param numChannels The number of audio channels
         * @param numFrames The number of frames per channel
         */
        AudioBuffer (int numChannels, int numFrames)
        {
            resize (numChannels, numFrames);
        }

        /**
         * @brief Resize the audio buffer
         * @param newNumChannels New number of channels
         * @param newNumFrames New number of frames per channel
         * @throws std::invalid_argument if dimensions are invalid
         */
        void resize (int newNumChannels, int newNumFrames)
        {
            if (newNumChannels <= 0 || newNumFrames <= 0)
                throw std::invalid_argument ("Buffer dimensions must be positive");

            data.resize (newNumChannels);
            channelPointers.resize (newNumChannels);

            for (int ch = 0; ch < newNumChannels; ++ch)
            {
                data[ch].resize (newNumFrames, 0.0f);
                channelPointers[ch] = data[ch].data();
            }
        }

        /**
         * @brief Clear the buffer contents (set all samples to 0)
         */
        void clear()
        {
            for (auto& channel : data)
                std::fill (channel.begin(), channel.end(), 0.0f);
        }

        /**
         * @brief Get the number of channels
         * @return Number of channels
         */
        int getNumChannels() const { return static_cast<int> (data.size()); }

        /**
         * @brief Get the number of frames per channel
         * @return Number of frames per channel
         */
        int getNumFrames() const { return data.empty() ? 0 : static_cast<int> (data[0].size()); }

        /**
         * @brief Get write access to sample data for a channel
         * @param channel Channel index
         * @return Pointer to the channel's sample data
         * @throws std::out_of_range if channel index is invalid
         */
        float* getChannelPointer (int channel)
        {
            if (channel < 0 || channel >= getNumChannels())
                throw std::out_of_range ("Channel index out of range");
            return data[channel].data();
        }

        /**
         * @brief Get read-only access to sample data for a channel
         * @param channel Channel index
         * @return Const pointer to the channel's sample data
         * @throws std::out_of_range if channel index is invalid
         */
        const float* getChannelPointer (int channel) const
        {
            if (channel < 0 || channel >= getNumChannels())
                throw std::out_of_range ("Channel index out of range");
            return data[channel].data();
        }

        /**
         * @brief Get raw pointer array for use with processing functions
         * @return Pointer to array of channel pointers
         */
        float** getArrayOfChannels()
        {
            return channelPointers.data();
        }

        /**
         * @brief Get const raw pointer array for use with processing functions
         * @return Const pointer to array of channel pointers
         */
        float* const* getArrayOfChannels() const
        {
            return channelPointers.data();
        }

    private:
        std::vector<std::vector<float>> data;
        std::vector<float*> channelPointers; // Cache of data pointers for efficient access
    };
} // namespace PlayfulTones::DspToolBox