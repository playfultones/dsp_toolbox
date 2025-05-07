/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once
#include <stdexcept>
#include <vector>

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief A non-owning view into multi-channel audio buffer data.
     * 
     * This class provides a way to work with audio buffer data that is owned
     * by another entity. It provides similar interface to AudioBuffer but doesn't
     * manage memory itself.
     */
    struct BufferView
    {
    public:
        /**
         * @brief Set the data for the buffer view
         * @param buffer Array of pointers to channel data
         * @param numChannels Number of audio channels
         * @param numFrames Number of frames per channel
         * @throws std::invalid_argument if dimensions are invalid or buffer is null
         */
        void setData (float** buffer, int numChannels, int numFrames)
        {
            if (buffer == nullptr)
                throw std::invalid_argument ("Buffer pointer cannot be null");
            if (numChannels <= 0 || numFrames <= 0)
                throw std::invalid_argument ("Buffer dimensions must be positive");

            channelPointers.resize (numChannels);
            for (int ch = 0; ch < numChannels; ++ch)
            {
                if (buffer[ch] == nullptr)
                    throw std::invalid_argument ("Channel pointer cannot be null");
                channelPointers[ch] = buffer[ch];
            }

            this->numChannels = numChannels;
            this->numFrames = numFrames;
        }

        /**
         * @brief Get the number of channels
         * @return Number of channels
         */
        int getNumChannels() const { return numChannels; }

        /**
         * @brief Get the number of frames per channel
         * @return Number of frames per channel
         */
        int getNumFrames() const { return numFrames; }

        /**
         * @brief Get write access to sample data for a channel
         * @param channel Channel index
         * @return Pointer to the channel's sample data
         * @throws std::out_of_range if channel index is invalid
         */
        float* getChannelPointer (int channel)
        {
            if (channel < 0 || channel >= numChannels)
                throw std::out_of_range ("Channel index out of range");
            return channelPointers[channel];
        }

        /**
         * @brief Get read-only access to sample data for a channel
         * @param channel Channel index
         * @return Const pointer to the channel's sample data
         * @throws std::out_of_range if channel index is invalid
         */
        const float* getChannelPointer (int channel) const
        {
            if (channel < 0 || channel >= numChannels)
                throw std::out_of_range ("Channel index out of range");
            return channelPointers[channel];
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
        std::vector<float*> channelPointers;
        int numChannels = 0;
        int numFrames = 0;
    };
} // namespace PlayfulTones::DspToolBox