/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/
#pragma once

#include "../../midi/midi_sequence.h"
#include "../musical_time.h"
#include <memory>
#include <random>
#include <vector>

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief Implements a simple random melody generation algorithm.
     * 
     * This class generates melodies based on a hierarchical grammar where:
     * - A pattern is either a single cell or a cell followed by another pattern
     * - A cell is either 2 or 3 beats long
     * - A two-beat cell is either two eighths or one quarter
     * - A three-beat cell is either three eighths, or an eighth + quarter in either order
     * - Each note can be either a note or a rest
     */
    class SimpleMelodyGenerator
    {
    public:
        SimpleMelodyGenerator()
            : m_rng (std::random_device {}())
        {
        }

        /**
         * @brief Generates a melody pattern following the recursive grammar rule:
         *        A pattern is either a single cell or a cell followed by another pattern
         * @return Generated MIDI sequence
         */
        MidiSequence generatePattern()
        {
            MidiSequence sequence;

            if (m_scale.empty())
                return sequence; // No scale set, return empty sequence

            generatePatternRecursive (sequence, 0);
            return sequence;
        }

        /**
         * @brief Sets the maximum depth for recursive pattern generation to prevent stack overflow
         * @param maxDepth The maximum recursion depth (default is 8)
         */
        void setMaxRecursionDepth (size_t maxDepth) noexcept
        {
            m_maxDepth = maxDepth;
        }

        /**
         * @brief Sets the root note for the melody
         * @param rootNote MIDI note number to use as the root note
         */
        void setRootNote (uint8_t rootNote) noexcept
        {
            m_rootNote = rootNote;
        }

        /**
         * @brief Sets the scale to use for melody generation
         * @param scale Vector of scale degrees (offsets from root note)
         */
        void setScale (std::vector<int8_t> scale)
        {
            m_scale = std::move (scale);
        }

        /**
         * @brief Sets the MIDI channel for generated notes
         * @param velocity MIDI channel (0-15)
         * @note MIDI channels are 0-indexed, so channel 0 corresponds to MIDI channel 1
         */
        void setChannel (uint8_t channel) noexcept
        {
            m_channel = channel;
        }

        /**
         * @brief Sets the velocity range for generated notes
         * @param min Minimum velocity (0-127)
         * @param max Maximum velocity (0-127)
         * @note The generator will randomly choose a velocity between min and max
         */
        void setVelocityRange (uint8_t min, uint8_t max) noexcept
        {
            m_velocityMin = min;
            m_velocityMax = max;
        }

    private:
        mutable std::mt19937 m_rng;
        uint8_t m_channel = 0; // Default to MIDI channel 1
        uint8_t m_velocityMin = 64; // Default minimum velocity
        uint8_t m_velocityMax = 127; // Default maximum velocity
        uint8_t m_rootNote = 60; // Default to middle C
        std::vector<int8_t> m_scale;
        size_t m_maxDepth = 8; // Default max recursion depth

        /**
         * @brief Recursively generates a pattern
         * 
         * @param sequence MIDI sequence to add notes to
         * @param startTime Start time for this pattern
         * @return End time of the generated pattern
         */
        uint64_t generatePatternRecursive (MidiSequence& sequence, uint64_t startTime)
        {
            // Generate the first cell
            uint64_t currentTime = generateCell (sequence, startTime);

            // Continue with another pattern if we haven't reached max depth
            // and with 50% probability
            if (m_maxDepth > 1 && std::uniform_int_distribution<> { 0, 1 }(m_rng) == 1)
            {
                --m_maxDepth;
                currentTime = generatePatternRecursive (sequence, currentTime);
                ++m_maxDepth;
            }

            return currentTime;
        }

        /**
         * @brief Generates a single cell in the pattern
         * 
         * @param sequence MIDI sequence to add notes to
         * @param startTime Start time for this cell
         * @return End time of the generated cell
         */
        uint64_t generateCell (MidiSequence& sequence, uint64_t startTime)
        {
            // Randomly choose between 2-beat and 3-beat cell
            bool isTwoBeats = std::uniform_int_distribution<> { 0, 1 }(m_rng) == 0;

            if (isTwoBeats)
            {
                return generateTwoBeatsCell (sequence, startTime);
            }
            else
            {
                return generateThreeBeatsCell (sequence, startTime);
            }
        }

        /**
         * @brief Generates a two-beat cell (either two eighths or one quarter)
         * @param sequence MIDI sequence to add notes to
         * @param startTime Start time for this cell
         * @return End time of the generated cell
         */
        uint64_t generateTwoBeatsCell (MidiSequence& sequence, uint64_t startTime)
        {
            using namespace MusicalTime;

            // Decide between two eighths or one quarter
            bool twoEighths = std::uniform_int_distribution<> { 0, 1 }(m_rng) == 0;

            if (twoEighths)
            {
                // Generate two eighth notes
                uint64_t firstNoteLength = durationToTicks<EighthNote>();
                uint64_t secondNoteLength = firstNoteLength;

                addNoteOrRest (sequence, startTime, firstNoteLength);
                addNoteOrRest (sequence, startTime + firstNoteLength, secondNoteLength);

                return startTime + firstNoteLength + secondNoteLength;
            }
            else
            {
                // Generate one quarter note
                uint64_t noteLength = durationToTicks<QuarterNote>();
                addNoteOrRest (sequence, startTime, noteLength);
                return startTime + noteLength;
            }
        }

        /**
         * @brief Generates a three-beat cell (three eighths or eighth + quarter in either order)
         * @param sequence MIDI sequence to add notes to
         * @param startTime Start time for this cell
         * @return End time of the generated cell
         */
        uint64_t generateThreeBeatsCell (MidiSequence& sequence, uint64_t startTime)
        {
            using namespace MusicalTime;

            // Choose between three eighths or eighth + quarter
            bool threeEighths = std::uniform_int_distribution<> { 0, 1 }(m_rng) == 0;

            if (threeEighths)
            {
                // Generate three eighth notes
                uint64_t eighthLength = durationToTicks<EighthNote>();

                addNoteOrRest (sequence, startTime, eighthLength);
                addNoteOrRest (sequence, startTime + eighthLength, eighthLength);
                addNoteOrRest (sequence, startTime + 2 * eighthLength, eighthLength);

                return startTime + 3 * eighthLength;
            }
            else
            {
                // Generate eighth + quarter in either order
                bool eighthFirst = std::uniform_int_distribution<> { 0, 1 }(m_rng) == 0;
                uint64_t eighthLength = durationToTicks<EighthNote>();
                uint64_t quarterLength = durationToTicks<QuarterNote>();

                if (eighthFirst)
                {
                    addNoteOrRest (sequence, startTime, eighthLength);
                    addNoteOrRest (sequence, startTime + eighthLength, quarterLength);
                }
                else
                {
                    addNoteOrRest (sequence, startTime, quarterLength);
                    addNoteOrRest (sequence, startTime + quarterLength, eighthLength);
                }

                return startTime + eighthLength + quarterLength;
            }
        }

        /**
         * @brief Generates a random velocity within the specified range
         * @return Random velocity value
         */
        [[nodiscard]] uint8_t getRandomVelocity() const noexcept
        {
            return std::uniform_int_distribution<> { m_velocityMin, m_velocityMax }(m_rng);
        }

        /**
         * @brief Maybe adds a note to the sequence (50% chance of note vs rest)
         * @param sequence MIDI sequence to add the note/rest to
         * @param startTime Start time for the note/rest
         * @param duration Duration of the note/rest in ticks
         */
        void addNoteOrRest (MidiSequence& sequence, uint64_t startTime, uint64_t duration)
        {
            // 50% chance of rest vs note
            if (std::uniform_int_distribution<> { 0, 1 }(m_rng) == 0)
            {
                return; // Rest
            }

            // Choose a random scale degree
            int scaleIndex = std::uniform_int_distribution<> { 0, static_cast<int> (m_scale.size() - 1) }(m_rng);
            uint8_t note = m_rootNote + m_scale[scaleIndex];

            // Add note on
            sequence.addMessage (startTime,
                MidiMessage (MidiStatus::NoteOn, m_channel, note, getRandomVelocity()));

            // Add note off
            sequence.addMessage (startTime + duration,
                MidiMessage (MidiStatus::NoteOff, m_channel, note));
        }
    };
}
