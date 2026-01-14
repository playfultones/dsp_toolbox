/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/core/decibels.hpp"
#include "dsp_toolbox/core/frequency.hpp"
#include "dsp_toolbox/core/linear_gain.hpp"
#include "dsp_toolbox/core/midi_note_number.hpp"
#include "dsp_toolbox/core/milliseconds.hpp"
#include "dsp_toolbox/core/sample_rate.hpp"
#include "dsp_toolbox/core/samples.hpp"
#include "dsp_toolbox/core/seconds.hpp"

#include "dsp_toolbox/math/functions.hpp"

namespace PlayfulTones::DspToolbox
{

    /**
 * @brief Convert decibels to linear gain.
 *
 * gain = 10^(dB/20)
 *
 * @param dB Decibels value
 * @return Linear gain multiplier
 */
    template <Numeric T>
    [[nodiscard]] constexpr LinearGain<T> toLinearGain (Decibels<T> dB) noexcept
    {
        return LinearGain<T> { Math::pow (T (10), dB.value() / T (20)) };
    }

    /**
 * @brief Convert linear gain to decibels.
 *
 * dB = 20 * log10(gain)
 *
 * @param gain Linear gain multiplier
 * @return Decibels value
 */
    template <Numeric T>
    [[nodiscard]] constexpr Decibels<T> toDecibels (LinearGain<T> gain) noexcept
    {
        return Decibels<T> { T (20) * Math::log (gain.value()) / Math::ln10<T> };
    }

    /**
 * @brief Convert seconds to samples.
 *
 * @param duration Time in seconds
 * @param rate Sample rate
 * @return Sample count
 */
    template <Numeric T, Numeric U>
    [[nodiscard]] constexpr Samples<int64_t> toSamples (Seconds<T> duration, SampleRate<U> rate) noexcept
    {
        return Samples<int64_t> { static_cast<int64_t> (duration.value() * rate.value) };
    }

    /**
 * @brief Convert milliseconds to samples.
 *
 * @param duration Time in milliseconds
 * @param rate Sample rate
 * @return Sample count
 */
    template <Numeric T, Numeric U>
    [[nodiscard]] constexpr Samples<int64_t> toSamples (Milliseconds<T> duration, SampleRate<U> rate) noexcept
    {
        return Samples<int64_t> { static_cast<int64_t> (duration.value() * rate.value / T (1000)) };
    }

    /**
 * @brief Convert samples to seconds.
 *
 * @param count Sample count
 * @param rate Sample rate
 * @return Time in seconds
 */
    template <Numeric T, Numeric U>
    [[nodiscard]] constexpr Seconds<U> toSeconds (Samples<T> count, SampleRate<U> rate) noexcept
    {
        return Seconds<U> { static_cast<U> (count.value) / rate.value };
    }

    /**
 * @brief Convert samples to milliseconds.
 *
 * @param count Sample count
 * @param rate Sample rate
 * @return Time in milliseconds
 */
    template <Numeric T, Numeric U>
    [[nodiscard]] constexpr Milliseconds<U> toMilliseconds (Samples<T> count, SampleRate<U> rate) noexcept
    {
        return Milliseconds<U> { static_cast<U> (count.value) * U (1000) / rate.value };
    }

    /**
 * @brief Convert seconds to milliseconds.
 */
    template <Numeric T>
    [[nodiscard]] constexpr Milliseconds<T> toMilliseconds (Seconds<T> s) noexcept
    {
        return Milliseconds<T> { s.value() * T (1000) };
    }

    /**
 * @brief Convert milliseconds to seconds.
 */
    template <Numeric T>
    [[nodiscard]] constexpr Seconds<T> toSeconds (Milliseconds<T> ms) noexcept
    {
        return Seconds<T> { ms.value() / T (1000) };
    }

    /**
 * @brief Convert MIDI note number to frequency.
 *
 * freq = a4Reference * 2^((note - 69) / 12)
 *
 * @param note MIDI note number (0-127)
 * @param a4Reference Tuning reference for A4 (default: 440 Hz)
 * @return Frequency in Hz
 */
    template <Numeric T = float>
    [[nodiscard]] constexpr Frequency<T> toFrequency (
        MIDINoteNumber note,
        Frequency<T> a4Reference = Frequency<T> { T (440) }) noexcept
    {
        T const semitones = static_cast<T> (note.value()) - T (69);
        return Frequency<T> { a4Reference.value() * Math::pow (T (2), semitones / T (12)) };
    }

    /**
 * @brief Convert frequency to MIDI note number (nearest note).
 *
 * note = 69 + 12 * log2(freq / a4Reference)
 *
 * @param freq Frequency in Hz
 * @param a4Reference Tuning reference for A4 (default: 440 Hz)
 * @return Nearest MIDI note number
 */
    template <Numeric T>
    [[nodiscard]] constexpr MIDINoteNumber toMIDINoteNumber (
        Frequency<T> freq,
        Frequency<T> a4Reference = Frequency<T> { T (440) }) noexcept
    {
        T const semitones = T (12) * Math::log (freq.value() / a4Reference.value()) / Math::ln2<T>;
        T const noteNumber = T (69) + semitones;
        auto const rounded = static_cast<int> (noteNumber + T (0.5));
        return MIDINoteNumber { static_cast<uint8_t> (rounded < 0 ? 0 : (rounded > 127 ? 127 : rounded)) };
    }

    /**
     * @brief Convert 1V/Oct CV to frequency.
     *
     * freq = baseFreq * 2^cv
     *
     * The 1V/Oct (one volt per octave) standard means:
     * - cv = 0.0 → base frequency
     * - cv = 1.0 → one octave up (2x frequency)
     * - cv = -1.0 → one octave down (0.5x frequency)
     *
     * @param cv Control voltage in 1V/Oct format
     * @param baseFreq Reference frequency at cv=0 (default: 261.63 Hz = middle C)
     * @return Frequency in Hz
     */
    template <Numeric T>
    [[nodiscard]] constexpr Frequency<T> cvToFrequency (
        T cv,
        Frequency<T> baseFreq = Frequency<T> { T (261.63) }) noexcept
    {
        return Frequency<T> { baseFreq.value() * Math::pow (T (2), cv) };
    }

    /**
     * @brief Convert frequency to 1V/Oct CV.
     *
     * cv = log2(freq / baseFreq)
     *
     * @param freq Frequency in Hz
     * @param baseFreq Reference frequency at cv=0 (default: 261.63 Hz = middle C)
     * @return Control voltage in 1V/Oct format
     */
    template <Numeric T>
    [[nodiscard]] constexpr T frequencyToCv (
        Frequency<T> freq,
        Frequency<T> baseFreq = Frequency<T> { T (261.63) }) noexcept
    {
        return Math::log (freq.value() / baseFreq.value()) / Math::ln2<T>;
    }

    /**
     * @brief Convert semitones to 1V/Oct CV.
     *
     * cv = semitones / 12
     *
     * Since 1V/Oct means 1 volt = 1 octave = 12 semitones,
     * each semitone equals 1/12 volt.
     *
     * @param semitones Number of semitones (positive = up, negative = down)
     * @return Control voltage in 1V/Oct format
     */
    template <Numeric T = float>
    [[nodiscard]] constexpr T semitonesToCv (int semitones) noexcept
    {
        return static_cast<T> (semitones) / T (12);
    }

    /**
     * @brief Convert 1V/Oct CV to semitones.
     *
     * semitones = cv * 12
     *
     * @param cv Control voltage in 1V/Oct format
     * @return Number of semitones (may be fractional)
     */
    template <Numeric T>
    [[nodiscard]] constexpr T cvToSemitones (T cv) noexcept
    {
        return cv * T (12);
    }

} // namespace PlayfulTones::DspToolbox
