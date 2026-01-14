/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include <algorithm>
#include <concepts>
#include <cstddef>

namespace PlayfulTones::DspToolbox
{

    /**
     * @brief Default I/O configuration for simple mono processors.
     *
     * Use this configuration for processors that have:
     * - 1 audio input channel
     * - 1 audio output channel
     * - No CV (control voltage) inputs or outputs
     *
     * ## Example Usage
     * @code
     * class SimpleGain : public ProcessorBase<SimpleGain, DefaultIOConfig> {
     *     // Mono in/out, no CV
     * };
     * @endcode
     */
    struct DefaultIOConfig
    {
        /** @brief Number of audio input channels */
        static constexpr std::size_t inAudio = 1;

        /** @brief Number of audio output channels */
        static constexpr std::size_t outAudio = 1;

        /** @brief Number of CV (control voltage) input channels */
        static constexpr std::size_t cvIn = 0;

        /** @brief Number of CV (control voltage) output channels */
        static constexpr std::size_t cvOut = 0;

        /** @brief Maximum of input and output audio channels */
        static constexpr std::size_t audioChannels = 1;

        /** @brief Maximum of input and output CV channels */
        static constexpr std::size_t cvChannels = 0;

        /** @brief Total channels in unified buffer (audio + CV) */
        static constexpr std::size_t totalChannels = 1;
    };

    /**
     * @brief Compile-time channel configuration for processors.
     *
     * IOConfig defines the I/O requirements of a processor at compile time,
     * enabling static verification and the unified audio + CV buffer model.
     *
     * ## Template Parameters
     * @tparam InAudio Number of audio input channels (0+)
     * @tparam OutAudio Number of audio output channels (0+)
     * @tparam CVIn Number of CV input channels for modulation (0+)
     * @tparam CVOut Number of CV output channels (default: 0)
     *
     * ## Unified Buffer Layout
     * The buffer is organized as:
     * - Channels [0, audioChannels): Audio I/O (in-place processing)
     * - Channels [audioChannels, totalChannels): CV I/O
     *
     * ## Example Usage
     * @code
     * // Stereo processor with no CV
     * using StereoConfig = IOConfig<2, 2, 0, 0>;
     *
     * // Mono filter with 2 CV inputs (cutoff, resonance)
     * using FilterConfig = IOConfig<1, 1, 2, 0>;
     *
     * // LFO: no audio in, 1 CV out
     * using LFOConfig = IOConfig<0, 0, 0, 1>;
     *
     * class SVFilter : public ProcessorBase<SVFilter, IOConfig<1, 1, 2, 0>> {
     *     // Mono audio, 2 CV inputs (cutoff CV at channel 1, resonance CV at channel 2)
     * };
     * @endcode
     */
    template <std::size_t InAudio, std::size_t OutAudio, std::size_t CVIn, std::size_t CVOut = 0>
    struct IOConfig
    {
        /** @brief Number of audio input channels */
        static constexpr std::size_t inAudio = InAudio;

        /** @brief Number of audio output channels */
        static constexpr std::size_t outAudio = OutAudio;

        /** @brief Number of CV (control voltage) input channels */
        static constexpr std::size_t cvIn = CVIn;

        /** @brief Number of CV (control voltage) output channels */
        static constexpr std::size_t cvOut = CVOut;

        /** @brief Maximum of input and output audio channels (for in-place processing) */
        static constexpr std::size_t audioChannels = std::max (InAudio, OutAudio);

        /** @brief Maximum of input and output CV channels */
        static constexpr std::size_t cvChannels = std::max (CVIn, CVOut);

        /** @brief Total channels in unified buffer (audio + CV) */
        static constexpr std::size_t totalChannels = audioChannels + cvChannels;
    };

    /**
     * @brief Concept for valid I/O configurations.
     *
     * A type satisfies IOConfigLike if it provides all the static constexpr
     * members required for channel configuration, and the derived values
     * are consistent with the primary values.
     *
     * ## Required Members
     * | Member | Type | Description |
     * |--------|------|-------------|
     * | `inAudio` | `std::size_t` | Number of audio input channels |
     * | `outAudio` | `std::size_t` | Number of audio output channels |
     * | `cvIn` | `std::size_t` | Number of CV input channels |
     * | `cvOut` | `std::size_t` | Number of CV output channels |
     * | `audioChannels` | `std::size_t` | max(inAudio, outAudio) |
     * | `cvChannels` | `std::size_t` | max(cvIn, cvOut) |
     * | `totalChannels` | `std::size_t` | audioChannels + cvChannels |
     *
     * ## Consistency Requirements
     * - `audioChannels` must equal `max(inAudio, outAudio)`
     * - `cvChannels` must equal `max(cvIn, cvOut)`
     * - `totalChannels` must equal `audioChannels + cvChannels`
     *
     * ## Example Implementation
     * @code
     * struct MyCustomIOConfig {
     *     static constexpr std::size_t inAudio = 2;
     *     static constexpr std::size_t outAudio = 2;
     *     static constexpr std::size_t cvIn = 1;
     *     static constexpr std::size_t cvOut = 0;
     *     static constexpr std::size_t audioChannels = 2;
     *     static constexpr std::size_t cvChannels = 1;
     *     static constexpr std::size_t totalChannels = 3;
     * };
     * static_assert(IOConfigLike<MyCustomIOConfig>);
     * @endcode
     */
    template <typename C>
    concept IOConfigLike = requires {
        { C::inAudio } -> std::convertible_to<std::size_t>;
        { C::outAudio } -> std::convertible_to<std::size_t>;
        { C::cvIn } -> std::convertible_to<std::size_t>;
        { C::cvOut } -> std::convertible_to<std::size_t>;
        { C::audioChannels } -> std::convertible_to<std::size_t>;
        { C::cvChannels } -> std::convertible_to<std::size_t>;
        { C::totalChannels } -> std::convertible_to<std::size_t>;
    } && (C::audioChannels == std::max (C::inAudio, C::outAudio)) && (C::cvChannels == std::max (C::cvIn, C::cvOut)) && (C::totalChannels == C::audioChannels + C::cvChannels);

    //--------------------------------------------------------------------------
    // Static assertions to verify IOConfig and DefaultIOConfig satisfy IOConfigLike
    //--------------------------------------------------------------------------

    static_assert (IOConfigLike<DefaultIOConfig>, "DefaultIOConfig must satisfy IOConfigLike");
    static_assert (IOConfigLike<IOConfig<1, 1, 0, 0>>, "IOConfig<1,1,0,0> must satisfy IOConfigLike");
    static_assert (IOConfigLike<IOConfig<2, 2, 0, 0>>, "IOConfig<2,2,0,0> must satisfy IOConfigLike");
    static_assert (IOConfigLike<IOConfig<1, 1, 2, 0>>, "IOConfig<1,1,2,0> must satisfy IOConfigLike");
    static_assert (IOConfigLike<IOConfig<0, 0, 0, 1>>, "IOConfig<0,0,0,1> must satisfy IOConfigLike");

    //--------------------------------------------------------------------------
    // IOConfig classification concepts
    //--------------------------------------------------------------------------

    /**
     * @brief Concept for mono effect IOConfigs (1 in, 1 out).
     */
    template <typename IO>
    concept MonoEffectIO = IOConfigLike<IO> && IO::inAudio == 1 && IO::outAudio == 1;

    /**
     * @brief Concept for stereo effect IOConfigs (2 in, 2 out).
     */
    template <typename IO>
    concept StereoEffectIO = IOConfigLike<IO> && IO::inAudio == 2 && IO::outAudio == 2;

    /**
     * @brief Concept for mono-to-stereo IOConfigs (1 in, 2 out).
     */
    template <typename IO>
    concept MonoToStereoIO = IOConfigLike<IO> && IO::inAudio == 1 && IO::outAudio == 2;

    /**
     * @brief Concept for IOConfigs compatible with StereoExpander.
     *
     * Valid configurations:
     * - 1 in, 2 out (mono-to-stereo)
     * - 2 in, 2 out (stereo effect)
     */
    template <typename IO>
    concept StereoExpanderCompatibleIO = IOConfigLike<IO> && (IO::inAudio >= 1 && IO::inAudio <= 2) && (IO::outAudio == 2);

    //--------------------------------------------------------------------------
    // Static assertions for IOConfig classification concepts
    //--------------------------------------------------------------------------

    static_assert (MonoEffectIO<IOConfig<1, 1, 0, 0>>, "IOConfig<1,1,0,0> must satisfy MonoEffectIO");
    static_assert (StereoEffectIO<IOConfig<2, 2, 0, 0>>, "IOConfig<2,2,0,0> must satisfy StereoEffectIO");
    static_assert (MonoToStereoIO<IOConfig<1, 2, 0, 0>>, "IOConfig<1,2,0,0> must satisfy MonoToStereoIO");
    static_assert (StereoExpanderCompatibleIO<IOConfig<1, 2, 0, 0>>, "IOConfig<1,2,0,0> must satisfy StereoExpanderCompatibleIO");
    static_assert (StereoExpanderCompatibleIO<IOConfig<2, 2, 0, 0>>, "IOConfig<2,2,0,0> must satisfy StereoExpanderCompatibleIO");

    //--------------------------------------------------------------------------
    // Common configuration aliases
    //--------------------------------------------------------------------------

    /** @brief Mono processor configuration (1 in, 1 out, no CV) */
    using MonoConfig = IOConfig<1, 1, 0, 0>;

    /** @brief Stereo processor configuration (2 in, 2 out, no CV) */
    using StereoConfig = IOConfig<2, 2, 0, 0>;

    //--------------------------------------------------------------------------
    // Drone configurations (0 audio in, N audio out)
    //--------------------------------------------------------------------------

    /** @brief Mono drone configuration (0 in, 1 out, no CV) */
    using MonoDroneConfig = IOConfig<0, 1, 0, 0>;

    /** @brief Stereo drone configuration (0 in, 2 out, no CV) */
    using StereoDroneConfig = IOConfig<0, 2, 0, 0>;

    //--------------------------------------------------------------------------
    // Effect configurations (N audio in, N audio out, matching)
    //--------------------------------------------------------------------------

    /** @brief Mono effect configuration (1 in, 1 out, no CV) */
    using MonoEffectConfig = IOConfig<1, 1, 0, 0>;

    /** @brief Stereo effect configuration (2 in, 2 out, no CV) */
    using StereoEffectConfig = IOConfig<2, 2, 0, 0>;

    /** @brief Mono-to-stereo effect configuration (1 in, 2 out, no CV) */
    using MonoToStereoEffectConfig = IOConfig<1, 2, 0, 0>;

} // namespace PlayfulTones::DspToolbox
