/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once
#include <span>
#include <array>
#include <concepts>
#include <type_traits>

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
    #include <immintrin.h>
#endif

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief High-performance CRTP-based processor base class
     * 
     * This class uses CRTP (Curiously Recurring Template Pattern) to avoid virtual function
     * overhead in hot audio processing paths. All configuration is done at compile-time
     * for maximum cache efficiency.
     * 
     * @tparam Derived The derived processor class
     * @tparam SampleType The sample type (float, double)
     * @tparam BlockSize Fixed block size for processing (compile-time constant)
     * @tparam SampleRate Sample rate (compile-time constant)
     * @tparam NumChannels Number of audio channels
     */
    template<typename Derived, 
             typename SampleType = float, 
             size_t BlockSize = 512, 
             size_t SampleRate = 44100,
             size_t NumChannels = 2>
    class ProcessorBase
    {
    public:
        // Compile-time constants for cache optimization
        static constexpr size_t block_size = BlockSize;
        static constexpr size_t sample_rate = SampleRate;
        static constexpr size_t num_channels = NumChannels;
        using sample_type = SampleType;
        
        // Type aliases for buffer management
        using ChannelBuffer = std::span<sample_type, block_size>;
        using AudioBuffer = std::array<ChannelBuffer, num_channels>;
        
        /**
         * @brief Audio-rate processing (hot path) - no virtual functions
         * @param buffer Multi-channel audio buffer with compile-time known dimensions
         */
        void process_audio(AudioBuffer& buffer) noexcept
        {
            // Handle denormals by flushing to zero if needed
            flush_denormals();
            static_cast<Derived*>(this)->process_audio_impl(buffer);
        }
        
        /**
         * @brief Control-rate processing (cold path) for parameter updates
         * Called less frequently than audio processing
         */
        void process_control() noexcept
        {
            static_cast<Derived*>(this)->process_control_impl();
        }
        
        /**
         * @brief Prepare the processor - called once before processing starts
         */
        void prepare() noexcept
        {
            static_cast<Derived*>(this)->prepare_impl();
        }
        
        /**
         * @brief Reset processor state
         */
        void reset() noexcept
        {
            static_cast<Derived*>(this)->reset_impl();
        }
        
        /**
         * @brief Get compile-time sample rate
         */
        static constexpr size_t get_sample_rate() noexcept
        {
            return sample_rate;
        }
        
        /**
         * @brief Get compile-time block size
         */
        static constexpr size_t get_block_size() noexcept
        {
            return block_size;
        }
        
        /**
         * @brief Get compile-time channel count
         */
        static constexpr size_t get_num_channels() noexcept
        {
            return num_channels;
        }

    protected:
        // Default implementations that derived classes can override
        void process_audio_impl(AudioBuffer& /* buffer */) noexcept
        {
            // Default: pass through
        }
        
        void process_control_impl() noexcept
        {
            // Default: no control processing
        }
        
        void prepare_impl() noexcept
        {
            // Default: no preparation needed
        }
        
        void reset_impl() noexcept
        {
            // Default: no reset needed
        }

    private:
        /**
         * @brief Handle denormal numbers by setting flush-to-zero
         */
        static void flush_denormals() noexcept
        {
            #if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
                // Set flush-to-zero and denormals-are-zero on x86/x64
                _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
                _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
            #endif
        }
    };
    
    /**
     * @brief Concept to check if a type is a valid processor
     */
    template<typename T>
    concept Processor = requires(T processor) {
        typename T::sample_type;
        { T::block_size } -> std::convertible_to<size_t>;
        { T::sample_rate } -> std::convertible_to<size_t>;
        { T::num_channels } -> std::convertible_to<size_t>;
        { processor.process_audio(std::declval<typename T::AudioBuffer&>()) } -> std::same_as<void>;
        { processor.process_control() } -> std::same_as<void>;
        { processor.prepare() } -> std::same_as<void>;
        { processor.reset() } -> std::same_as<void>;
    };
    
    /**
     * @brief Type-erased processor interface for graph nodes
     */
    class ProcessorInterface
    {
    public:
        virtual ~ProcessorInterface() = default;
        virtual void process_audio_erased(void* buffer) noexcept = 0;
        virtual void process_control() noexcept = 0;
        virtual void prepare() noexcept = 0;
        virtual void reset() noexcept = 0;
        virtual size_t get_block_size() const noexcept = 0;
        virtual size_t get_sample_rate() const noexcept = 0;
        virtual size_t get_num_channels() const noexcept = 0;
    };
    
    /**
     * @brief Wrapper to type-erase processors for use in heterogeneous graphs
     */
    template<Processor P>
    class ProcessorWrapper : public ProcessorInterface
    {
    public:
        template<typename... Args>
        ProcessorWrapper(Args&&... args) : processor_(std::forward<Args>(args)...) {}
        
        void process_audio_erased(void* buffer) noexcept override
        {
            auto* typed_buffer = static_cast<typename P::AudioBuffer*>(buffer);
            processor_.process_audio(*typed_buffer);
        }
        
        void process_control() noexcept override
        {
            processor_.process_control();
        }
        
        void prepare() noexcept override
        {
            processor_.prepare();
        }
        
        void reset() noexcept override
        {
            processor_.reset();
        }
        
        size_t get_block_size() const noexcept override
        {
            return P::block_size;
        }
        
        size_t get_sample_rate() const noexcept override
        {
            return P::sample_rate;
        }
        
        size_t get_num_channels() const noexcept override
        {
            return P::num_channels;
        }
        
        P& processor() { return processor_; }
        const P& processor() const { return processor_; }
        
    private:
        P processor_;
    };
} // namespace PlayfulTones::DspToolBox
