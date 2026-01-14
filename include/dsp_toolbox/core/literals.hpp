/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/core/decibels.hpp"
#include "dsp_toolbox/core/decibels_full_scale.hpp"
#include "dsp_toolbox/core/frequency.hpp"
#include "dsp_toolbox/core/linear_gain.hpp"
#include "dsp_toolbox/core/milliseconds.hpp"
#include "dsp_toolbox/core/sample_rate.hpp"
#include "dsp_toolbox/core/samples.hpp"
#include "dsp_toolbox/core/seconds.hpp"

namespace PlayfulTones::DspToolbox::Literals
{

    constexpr Frequency<float> operator""_Hz (long double hz)
    {
        return Frequency<float> { static_cast<float> (hz) };
    }

    constexpr Frequency<float> operator""_Hz (unsigned long long hz)
    {
        return Frequency<float> { static_cast<float> (hz) };
    }

    constexpr Frequency<float> operator""_kHz (long double khz)
    {
        return Frequency<float> { static_cast<float> (khz * 1000.0L) };
    }

    constexpr Frequency<float> operator""_kHz (unsigned long long khz)
    {
        return Frequency<float> { static_cast<float> (khz * 1000) };
    }

    constexpr Decibels<float> operator""_dB (long double dB)
    {
        return Decibels<float> { static_cast<float> (dB) };
    }

    constexpr Decibels<float> operator""_dB (unsigned long long dB)
    {
        return Decibels<float> { static_cast<float> (dB) };
    }

    constexpr DecibelsFullScale<float> operator""_dBFS (long double dBFS)
    {
        return DecibelsFullScale<float> { static_cast<float> (dBFS) };
    }

    constexpr DecibelsFullScale<float> operator""_dBFS (unsigned long long dBFS)
    {
        return DecibelsFullScale<float> { static_cast<float> (dBFS) };
    }

    constexpr Seconds<float> operator""_s (long double s)
    {
        return Seconds<float> { static_cast<float> (s) };
    }

    constexpr Seconds<float> operator""_s (unsigned long long s)
    {
        return Seconds<float> { static_cast<float> (s) };
    }

    constexpr Milliseconds<float> operator""_ms (long double ms)
    {
        return Milliseconds<float> { static_cast<float> (ms) };
    }

    constexpr Milliseconds<float> operator""_ms (unsigned long long ms)
    {
        return Milliseconds<float> { static_cast<float> (ms) };
    }

    constexpr Samples<int64_t> operator""_samp (unsigned long long n)
    {
        return Samples<int64_t> { static_cast<int64_t> (n) };
    }

    constexpr SampleRate<double> operator""_sps (long double sps)
    {
        return SampleRate<double> { static_cast<double> (sps) };
    }

    constexpr SampleRate<double> operator""_sps (unsigned long long sps)
    {
        return SampleRate<double> { static_cast<double> (sps) };
    }

} // namespace PlayfulTones::DspToolbox::Literals
