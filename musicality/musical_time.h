/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#pragma once

#include <cstdint>
#include <ratio>

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief Constants and utilities for musical time and rhythm.
     * 
     * This namespace contains types and functions for working with musical time,
     * including note durations, tempo, and conversion between musical time and samples/ticks.
     */
    namespace MusicalTime
    {
        // Standard PPQN (Pulses Per Quarter Note) for MIDI
        constexpr uint32_t PPQN = 960;

        /**
         * @brief Represents standard note durations as ratios relative to a whole note
         */
        using ThirtySecondNote = std::ratio<1, 32>;
        using SixteenthNote = std::ratio<1, 16>;
        using EighthNote = std::ratio<1, 8>;
        using QuarterNote = std::ratio<1, 4>;
        using HalfNote = std::ratio<1, 2>;
        using WholeNote = std::ratio<1, 1>;

        /**
         * @brief Represents dotted note durations (1.5x the regular duration)
         */
        using DottedThirtySecondNote = std::ratio<3, 64>;
        using DottedSixteenthNote = std::ratio<3, 32>;
        using DottedEighthNote = std::ratio<3, 16>;
        using DottedQuarterNote = std::ratio<3, 8>;
        using DottedHalfNote = std::ratio<3, 4>;
        using DottedWholeNote = std::ratio<3, 2>;

        /**
         * @brief Converts BPM (beats per minute) to microseconds per beat
         * @param bpm Tempo in beats per minute
         * @return Microseconds per beat
         */
        constexpr uint32_t bpmToMicroseconds (double bpm) noexcept
        {
            return static_cast<uint32_t> (60'000'000.0 / bpm);
        }

        /**
         * @brief Converts musical time to ticks/samples based on tempo and sample rate
         * @tparam NoteDuration The note duration ratio type
         * @param bpm Tempo in beats per minute
         * @param sampleRate The sample rate in Hz
         * @return Number of samples for the specified duration at the given tempo
         */
        template <typename NoteDuration>
        constexpr uint64_t durationToSamples (double bpm, double sampleRate) noexcept
        {
            constexpr double quarterNoteRatio = static_cast<double> (NoteDuration::num) / static_cast<double> (NoteDuration::den) * 4.0;
            const double secondsPerBeat = 60.0 / bpm;
            return static_cast<uint64_t> (sampleRate * secondsPerBeat * quarterNoteRatio);
        }

        /**
         * @brief Converts musical time to MIDI ticks
         * @tparam NoteDuration The note duration ratio type
         * @return Number of MIDI ticks for the specified duration
         */
        template <typename NoteDuration>
        constexpr uint64_t durationToTicks() noexcept
        {
            constexpr double quarterNoteRatio = static_cast<double> (NoteDuration::num) / static_cast<double> (NoteDuration::den) * 4.0;
            return static_cast<uint64_t> (PPQN * quarterNoteRatio);
        }

        /**
         * @brief Class representing a musical interval in ticks or samples
         */
        class Duration
        {
        public:
            /**
             * @brief Creates a Duration from a note length at a specific tempo
             * @tparam NoteDuration The note duration ratio type
             * @param bpm Tempo in beats per minute
             * @param sampleRate The sample rate in Hz
             * @return Duration object representing the interval in samples
             */
            template <typename NoteDuration>
            static constexpr Duration fromMusicalTime (double bpm, double sampleRate) noexcept
            {
                return Duration (durationToSamples<NoteDuration> (bpm, sampleRate));
            }

            /**
             * @brief Creates a Duration from MIDI ticks
             * @tparam NoteDuration The note duration ratio type
             * @return Duration object representing the interval in ticks
             */
            template <typename NoteDuration>
            static constexpr Duration fromTicks() noexcept
            {
                return Duration (durationToTicks<NoteDuration>());
            }

            constexpr explicit Duration (uint64_t length) noexcept : m_length (length) {}
            [[nodiscard]] constexpr uint64_t length() const noexcept { return m_length; }

        private:
            uint64_t m_length;
        };
    }
}
