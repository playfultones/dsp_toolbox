/*******************************************************************                                                                                                                
* Copyright         : 2025 Playful Tones                                                                                                                                            
* Author            : Bence Kovács                                                                                                                                                  
* License           : GNU General Public License v3.0                                                                                                                               
*******************************************************************/

#pragma once
#include <algorithm>
#include <array>
#include <concepts>
#include <cstring>
#include <span>
#include <utility>

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
    #include <immintrin.h>
#endif

namespace PlayfulTones::DspToolBox
{
    /**                                                                                                                                                                             
     * @brief High-performance audio buffer utilities for compile-time sized buffers                                                                                                
     *                                                                                                                                                                              
     * This provides utilities for working with the compile-time sized audio buffers                                                                                                
     * used in the ProcessorBase class. All operations are cache-efficient and                                                                                                      
     * designed for real-time audio processing.                                                                                                                                     
     */

    /**                                                                                                                                                                             
     * @brief Compile-time audio buffer storage with proper alignment                                                                                                               
     * @tparam SampleType The sample type (float, double)                                                                                                                           
     * @tparam BlockSize Fixed block size (compile-time constant)                                                                                                                   
     * @tparam NumChannels Number of audio channels (compile-time constant)                                                                                                         
     */
    template <typename SampleType, size_t BlockSize, size_t NumChannels>
    class alignas (32) AudioBufferStorage
    {
    public:
        using sample_type = SampleType;
        using ChannelBuffer = std::span<sample_type, BlockSize>;
        using AudioBuffer = std::array<ChannelBuffer, NumChannels>;

        static constexpr size_t block_size = BlockSize;
        static constexpr size_t num_channels = NumChannels;

        /**                                                                                                                                                                         
         * @brief Constructor - initializes all samples to zero                                                                                                                     
         */
        constexpr AudioBufferStorage() noexcept : data_ {}, spans_ { createSpans() }
        {
        }

    private:
        // Helper function to initialize spans using index sequence
        template<size_t... Is>
        constexpr AudioBuffer createSpansImpl(std::index_sequence<Is...>) noexcept
        {
            return AudioBuffer{ChannelBuffer{data_[Is].data(), BlockSize}...};
        }
        
        constexpr AudioBuffer createSpans() noexcept
        {
            return createSpansImpl(std::make_index_sequence<NumChannels>{});
        }

    public:

        /**                                                                                                                                                                         
         * @brief Get the audio buffer for processing                                                                                                                               
         * @return AudioBuffer reference ready for processing                                                                                                                       
         */
        constexpr AudioBuffer& getBuffer() noexcept
        {
            return spans_;
        }

        /**                                                                                                                                                                         
         * @brief Get the audio buffer for processing (const version)                                                                                                               
         * @return Const AudioBuffer reference                                                                                                                                      
         */
        constexpr const AudioBuffer& getBuffer() const noexcept
        {
            return spans_;
        }

        /**                                                                                                                                                                         
         * @brief Clear all channels to zero                                                                                                                                        
         */
        void clear() noexcept
        {
            for (auto& channel : data_)
            {
                std::fill (channel.begin(), channel.end(), sample_type { 0 });
            }
        }

        /**                                                                                                                                                                         
         * @brief Get access to a specific channel's data                                                                                                                           
         * @param channel Channel index                                                                                                                                             
         * @return Span to the channel's data                                                                                                                                       
         */
        constexpr ChannelBuffer getChannel (size_t channel) noexcept
        {
            return spans_[channel];
        }

        /**                                                                                                                                                                         
         * @brief Get const access to a specific channel's data                                                                                                                     
         * @param channel Channel index                                                                                                                                             
         * @return Const span to the channel's data                                                                                                                                 
         */
        constexpr ChannelBuffer getChannel (size_t channel) const noexcept
        {
            return spans_[channel];
        }

    private:
        // Aligned storage for maximum cache efficiency
        alignas (32) std::array<std::array<sample_type, BlockSize>, NumChannels> data_;
        AudioBuffer spans_;
    };

    /**                                                                                                                                                                             
     * @brief High-performance buffer operations                                                                                                                                    
     */
    namespace BufferOps
    {
        /**                                                                                                                                                                         
         * @brief Clear all channels in an audio buffer                                                                                                                             
         * @tparam AudioBuffer The audio buffer type                                                                                                                                
         * @param buffer The buffer to clear                                                                                                                                        
         */
        template <typename AudioBuffer>
        void clear (AudioBuffer& buffer) noexcept
        {
            for (auto& channel : buffer)
            {
                std::fill (channel.begin(), channel.end(), typename AudioBuffer::value_type::value_type { 0 });
            }
        }

        /**                                                                                                                                                                         
         * @brief Copy data from one buffer to another                                                                                                                              
         * @tparam AudioBuffer The audio buffer type                                                                                                                                
         * @param source Source buffer                                                                                                                                              
         * @param destination Destination buffer                                                                                                                                    
         */
        template <typename AudioBuffer>
        void copy (const AudioBuffer& source, AudioBuffer& destination) noexcept
        {
            for (size_t ch = 0; ch < source.size(); ++ch)
            {
                std::copy (source[ch].begin(), source[ch].end(), destination[ch].begin());
            }
        }

        /**                                                                                                                                                                         
         * @brief Add one buffer to another (mix)                                                                                                                                   
         * @tparam AudioBuffer The audio buffer type                                                                                                                                
         * @param source Source buffer to add                                                                                                                                       
         * @param destination Destination buffer (modified)                                                                                                                         
         */
        template <typename AudioBuffer>
        void add (const AudioBuffer& source, AudioBuffer& destination) noexcept
        {
            for (size_t ch = 0; ch < source.size(); ++ch)
            {
                for (size_t i = 0; i < source[ch].size(); ++i)
                {
                    destination[ch][i] += source[ch][i];
                }
            }
        }

        /**                                                                                                                                                                         
         * @brief Multiply buffer by a gain factor                                                                                                                                  
         * @tparam AudioBuffer The audio buffer type                                                                                                                                
         * @tparam GainType The gain type                                                                                                                                           
         * @param buffer Buffer to modify                                                                                                                                           
         * @param gain Gain factor to apply                                                                                                                                         
         */
        template <typename AudioBuffer, typename GainType>
        void multiply (AudioBuffer& buffer, GainType gain) noexcept
        {
            for (auto& channel : buffer)
            {
                for (auto& sample : channel)
                {
                    sample *= gain;
                }
            }
        }

        /**                                                                                                                                                                         
         * @brief SIMD-optimized buffer clearing (when available)                                                                                                                   
         * @tparam AudioBuffer The audio buffer type                                                                                                                                
         * @param buffer Buffer to clear                                                                                                                                            
         */
        template <typename AudioBuffer>
        void clear_simd (AudioBuffer& buffer) noexcept
        {
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
            if constexpr (std::is_same_v<typename AudioBuffer::value_type::value_type, float>)
            {
                const __m256 zero = _mm256_setzero_ps();

                for (auto& channel : buffer)
                {
                    float* data = channel.data();
                    const size_t simd_size = channel.size() & ~7; // Round down to multiple of 8

                    // Use unaligned stores for safety - compiler will optimize if aligned
                    for (size_t i = 0; i < simd_size; i += 8)
                    {
                        _mm256_storeu_ps (&data[i], zero);
                    }

                    // Handle remaining samples
                    for (size_t i = simd_size; i < channel.size(); ++i)
                    {
                        data[i] = 0.0f;
                    }
                }
                return;
            }
#endif

            // Fallback to standard clear
            clear (buffer);
        }
    }

    /**                                                                                                                                                                             
     * @brief Concept to validate audio buffer types                                                                                                                                
     */
    template <typename T>
    concept AudioBufferType = requires (T buffer) {
        typename T::value_type; // Should be a channel buffer type
        typename T::value_type::value_type; // Should have a sample type
        { buffer.size() } -> std::convertible_to<size_t>;
        { buffer[0] } -> std::convertible_to<typename T::value_type>;
        requires std::contiguous_iterator<decltype (buffer[0].begin())>;
        requires std::ranges::sized_range<typename T::value_type>;
    };

} // namespace PlayfulTones::DspToolBox
