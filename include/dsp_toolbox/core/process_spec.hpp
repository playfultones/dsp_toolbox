/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/core/sample_rate.hpp"
#include "dsp_toolbox/core/samples.hpp"

#include <cstdint>

namespace PlayfulTones::DspToolbox
{

    /**
     * @brief Runtime configuration for processor preparation.
     *
     * ProcessSpec is passed to processors during the prepare() phase to configure
     * them for the current audio context. This is the runtime variant used in
     * production code and JUCE integration.
     *
     * Uses strong types for type safety:
     * - SampleRate<double> for sample rate
     * - Samples<std::uint32_t> for block size
     *
     * @see ConstexprSpec for compile-time configuration used in constexpr testing
     */
    struct ProcessSpec
    {
        /** @brief Sample rate in samples per second (e.g., 44100.0, 48000.0, 96000.0) */
        SampleRate<double> sampleRate { 48000.0 };

        /** @brief Maximum number of samples per processing block */
        Samples<std::uint32_t> maxBlockSize { 512 };

        /** @brief Number of audio channels (1 = mono, 2 = stereo, etc.) */
        std::uint32_t numChannels { 2 };

        //----------------------------------------------------------------------
        // Platform hints - processors may use these to optimize behavior
        //----------------------------------------------------------------------

        /**
         * @brief Whether dynamic memory allocation is allowed.
         *
         * Set to false on constrained platforms (watchOS, embedded) where
         * real-time safety requires static allocation only.
         */
        bool allowDynamicAllocation { true };

        /**
         * @brief Whether to prefer SIMD optimizations.
         *
         * May be set to false on battery-constrained devices where power
         * efficiency is more important than raw performance.
         */
        bool preferSIMD { true };

        //----------------------------------------------------------------------
        // Comparison operators
        //----------------------------------------------------------------------

        constexpr bool operator== (const ProcessSpec& other) const noexcept = default;
    };

} // namespace PlayfulTones::DspToolbox
