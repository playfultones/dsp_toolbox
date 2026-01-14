/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "constexpr_graph.hpp"

#include <concepts>
#include <cstddef>

namespace PlayfulTones::DspToolbox::Graph
{

    //----------------------------------------------------------------------
    // DroneExternalIOLike - Self-clocked generators (0 audio in, N audio out)
    //----------------------------------------------------------------------

    /**
     * @brief Concept for drone-style ExternalIO configurations.
     *
     * Drones are self-clocked generators with:
     * - 0 audio inputs (generates sound autonomously)
     * - 1 or 2 audio outputs (mono or stereo)
     * - Any number of CV inputs (for external modulation)
     * - No CV outputs (internal modulation only)
     *
     * ## Valid Configurations
     * | inAudio | outAudio | cvIn | cvOut | Valid |
     * |---------|----------|------|-------|-------|
     * | 0       | 1        | 0+   | 0     | Yes   |
     * | 0       | 2        | 0+   | 0     | Yes   |
     * | 1+      | any      | any  | any   | No    |
     * | 0       | 3+       | any  | any   | No    |
     *
     * ## Example Implementation
     * @code
     * struct KickDrumIO {
     *     using IOConfig = IOConfig<0, 1, 0, 0>;  // Mono drone
     *     static constexpr auto Spec = MySpec;
     *     static constexpr std::array<ChannelMapping, 0> inputMappings = {};
     *     static constexpr std::array<ChannelMapping, 1> outputMappings = {...};
     * };
     * static_assert(DroneExternalIOLike<KickDrumIO>);
     * @endcode
     */
    template <typename T>
    concept DroneExternalIOLike = ExternalIOLike<T>
                                  && (T::IOConfig::inAudio == 0)
                                  && (T::IOConfig::outAudio >= 1 && T::IOConfig::outAudio <= 2)
                                  && (T::IOConfig::cvOut == 0);

    //----------------------------------------------------------------------
    // EffectExternalIOLike - Audio processors (N in, N out, matching)
    //----------------------------------------------------------------------

    /**
     * @brief Concept for effect-style ExternalIO configurations.
     *
     * Effects process audio with:
     * - 1 or 2 audio inputs
     * - 1 or 2 audio outputs (must be >= inputs, no channel reduction)
     * - Any number of CV inputs (for parameter modulation)
     * - No CV outputs
     *
     * ## Valid Configurations
     * | inAudio | outAudio | cvIn | cvOut | Valid | Use Case |
     * |---------|----------|------|-------|-------|----------|
     * | 1       | 1        | 0+   | 0     | Yes   | Mono effect |
     * | 2       | 2        | 0+   | 0     | Yes   | Stereo effect |
     * | 1       | 2        | 0+   | 0     | Yes   | Mono-to-stereo (widener, reverb) |
     * | 2       | 1        | any  | any   | No    | Channel reduction not allowed |
     * | 0       | any      | any  | any   | No    | Must have audio input |
     *
     * ## Example Implementation
     * @code
     * struct FilterIO {
     *     using IOConfig = IOConfig<1, 1, 2, 0>;  // Mono with cutoff + resonance CV
     *     static constexpr auto Spec = MySpec;
     *     static constexpr std::array<ChannelMapping, 3> inputMappings = {...};
     *     static constexpr std::array<ChannelMapping, 1> outputMappings = {...};
     * };
     * static_assert(EffectExternalIOLike<FilterIO>);
     * @endcode
     */
    template <typename T>
    concept EffectExternalIOLike = ExternalIOLike<T>
                                   && (T::IOConfig::inAudio >= 1 && T::IOConfig::inAudio <= 2)
                                   && (T::IOConfig::outAudio >= T::IOConfig::inAudio && T::IOConfig::outAudio <= 2)
                                   && (T::IOConfig::cvOut == 0);

    //----------------------------------------------------------------------
    // Future concepts (placeholder documentation)
    //----------------------------------------------------------------------

    /**
     * @brief Concept for synth-style ExternalIO configurations (future).
     *
     * Synths are like drones but with MIDI input:
     * - 0 audio inputs
     * - 1 or 2 audio outputs
     * - MIDI input (when MIDI support is added)
     *
     * TODO: Add when MIDI support is implemented.
     */
    // template <typename T>
    // concept SynthExternalIOLike = DroneExternalIOLike<T>
    //     && (T::IOConfig::midiIn == 1);

    /**
     * @brief Concept for MIDI effect configurations (future).
     *
     * MIDI effects transform MIDI without audio:
     * - 0 audio in/out
     * - MIDI in/out
     *
     * TODO: Add when MIDI support is implemented.
     */
    // template <typename T>
    // concept MidiEffectExternalIOLike = ...;

    //----------------------------------------------------------------------
    // Constrained factory functions for specialized GraphProcessors
    //----------------------------------------------------------------------

    /**
     * @brief Create a GraphProcessor for drone-style graphs.
     *
     * Provides compile-time validation that ExternalIO satisfies DroneExternalIOLike.
     *
     * @tparam ExternalIO Type defining the external interface (must satisfy DroneExternalIOLike)
     * @tparam InternalSampleType Sample type for internal buffer (default: float)
     * @tparam Graph ConstexprGraph type (deduced)
     * @param graph The graph to wrap
     * @return GraphProcessor wrapping the graph
     *
     * ## Example
     * @code
     * auto kickGraph = makeConstexprGraph<connections>(Clock{}, Env{}, VCO{}, VCA{});
     * auto kickProcessor = makeDroneProcessor<KickDrumIO>(std::move(kickGraph));
     * @endcode
     */
    template <DroneExternalIOLike ExternalIO, typename InternalSampleType = float, typename Graph>
    constexpr auto makeDroneProcessor (Graph graph)
    {
        return GraphProcessor<ExternalIO, Graph, InternalSampleType> { std::move (graph) };
    }

    /**
     * @brief Create a GraphProcessor for effect-style graphs.
     *
     * Provides compile-time validation that ExternalIO satisfies EffectExternalIOLike.
     *
     * @tparam ExternalIO Type defining the external interface (must satisfy EffectExternalIOLike)
     * @tparam InternalSampleType Sample type for internal buffer (default: float)
     * @tparam Graph ConstexprGraph type (deduced)
     * @param graph The graph to wrap
     * @return GraphProcessor wrapping the graph
     *
     * ## Example
     * @code
     * auto filterGraph = makeConstexprGraph<connections>(Biquad{});
     * auto filterProcessor = makeEffectProcessor<FilterIO>(std::move(filterGraph));
     * @endcode
     */
    template <EffectExternalIOLike ExternalIO, typename InternalSampleType = float, typename Graph>
    constexpr auto makeEffectProcessor (Graph graph)
    {
        return GraphProcessor<ExternalIO, Graph, InternalSampleType> { std::move (graph) };
    }

} // namespace PlayfulTones::DspToolbox::Graph
