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
     * @brief Storage for runtime spec values when using RuntimeSpec.
     *
     * When a processor is instantiated with RuntimeSpec, this struct stores
     * the actual sample rate and block size values provided at runtime via
     * prepareRuntime(). Uses [[no_unique_address]] in ProcessorBase to have
     * zero overhead when using static specs (replaced by std::monostate).
     */
    struct RuntimeSpecStorage
    {
        SampleRate<double> sampleRate { 48000.0 };
        Samples<std::uint32_t> blockSize { 512 };
    };

    /**
     * @brief Compile-time configuration for processor preparation.
     *
     * ConstexprSpec is the compile-time variant of ProcessSpec, designed for
     * constexpr testing and static optimization. All values are constexpr and
     * can be used in static_assert expressions and compile-time DSP evaluation.
     *
     * Uses strong types for type safety:
     * - SampleRate<double> for sample rate
     * - Samples<std::uint32_t> for block size
     *
     * Usage:
     * @code
     * constexpr ConstexprSpec spec {
     *     .sampleRate = SampleRate { 48000.0 },
     *     .blockSize = Samples<std::uint32_t> { 256 }
     * };
     * MyFilter filter;
     * filter.prepare(spec);  // constexpr preparation
     * @endcode
     *
     * @see ProcessSpec for runtime configuration used in production
     */
    struct ConstexprSpec
    {
        /** @brief Sample rate in samples per second */
        SampleRate<double> sampleRate { 48000.0 };

        /** @brief Number of samples per processing block */
        Samples<std::uint32_t> blockSize { 512 };

        /** @brief Number of audio channels (1 = mono, 2 = stereo, etc.) */
        std::uint32_t numChannels { 2 };

        //----------------------------------------------------------------------
        // Comparison operators
        //----------------------------------------------------------------------

        constexpr bool operator== (const ConstexprSpec& other) const noexcept = default;

        //----------------------------------------------------------------------
        // Conversion to ProcessSpec
        //----------------------------------------------------------------------

        /**
         * @brief Convert ConstexprSpec to ProcessSpec for runtime use.
         *
         * Enables using the same spec values in both compile-time and runtime
         * contexts. Platform hints default to allowing dynamic allocation and
         * preferring SIMD.
         *
         * @param allowDynamic Whether to allow dynamic allocation (default: true)
         * @param useSIMD Whether to prefer SIMD optimizations (default: true)
         * @return ProcessSpec with equivalent values
         */
        [[nodiscard]] constexpr auto toProcessSpec (bool allowDynamic = true, bool useSIMD = true) const noexcept;
    };

    //--------------------------------------------------------------------------
    // RuntimeSpec support for JUCE and other runtime-configured frameworks
    //--------------------------------------------------------------------------

    /**
     * @brief Detect if a ConstexprSpec is the runtime sentinel value.
     *
     * RuntimeSpec uses sampleRate = 0.0 as a sentinel to indicate that
     * the sample rate should be retrieved from runtime storage rather
     * than compile-time constants.
     *
     * @tparam Spec The ConstexprSpec to check
     */
    template <ConstexprSpec Spec>
    inline constexpr bool kIsRuntimeSpec = (Spec.sampleRate.value == 0.0);

    /**
     * @brief Sentinel value indicating runtime-configured processor.
     *
     * When a processor template is instantiated with RuntimeSpec, it stores
     * sample rate and block size at runtime rather than using compile-time
     * constants. ProcessorBase detects this via kIsRuntimeSpec<Spec> and
     * stores values in a RuntimeSpecStorage member.
     *
     * Usage with ProcessorWrapper:
     * @code
     * // HybridSpecs tries static specs first, falls back to RuntimeSpec
     * using HybridSpecs = SpecSet<Spec44100_1024, Spec48000_1024, RuntimeSpec>;
     * ProcessorWrapper<MyFilter, HybridSpecs> wrapper;
     * wrapper.prepare({88200.0, 512});  // Uses RuntimeSpec variant
     * @endcode
     */
    inline constexpr ConstexprSpec RuntimeSpec {
        .sampleRate = SampleRate { 0.0 }, // Sentinel: means "stored at runtime"
        .blockSize = Samples<std::uint32_t> { 0 }
    };

} // namespace PlayfulTones::DspToolbox

// Late include for toProcessSpec implementation (avoids forward declaration)
#include "dsp_toolbox/core/process_spec.hpp" // NOLINT:include-order

namespace PlayfulTones::DspToolbox
{

    constexpr auto ConstexprSpec::toProcessSpec (bool allowDynamic, bool useSIMD) const noexcept
    {
        return ProcessSpec {
            .sampleRate = sampleRate,
            .maxBlockSize = blockSize,
            .numChannels = numChannels,
            .allowDynamicAllocation = allowDynamic,
            .preferSIMD = useSIMD
        };
    }

} // namespace PlayfulTones::DspToolbox
