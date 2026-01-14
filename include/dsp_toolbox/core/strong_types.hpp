/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

/**
 * @file strong_types.hpp
 * @brief Umbrella header for all strong value types.
 *
 * Includes all type-safe DSP value wrappers:
 * - Decibels, DecibelsFullScale, LinearGain
 * - Frequency, SampleRate
 * - Seconds, Milliseconds, Samples
 * - MIDINoteNumber and predefined note constants
 * - CV/RV (ConstantValue/RuntimeValue) for compile-time optimization
 *
 * @code
 * #include <dsp_toolbox/core/strong_types.hpp>
 *
 * using namespace PlayfulTones::DspToolbox;
 * using namespace PlayfulTones::DspToolbox::Notes;
 *
 * auto freq = toFrequency(A4);  // 440 Hz
 * auto gain = toLinearGain(Decibels{-6.0f});  // 0.5
 *
 * // CV/RV for compile-time optimization
 * auto unity = CV<1.0f>{};  // Enables compile-time bypass
 * auto param = RV{userValue};  // Runtime value
 * @endcode
 */

#include "dsp_toolbox/core/concepts.hpp"
#include "dsp_toolbox/core/conversions.hpp"
#include "dsp_toolbox/core/crv.hpp"
#include "dsp_toolbox/core/decibels.hpp"
#include "dsp_toolbox/core/decibels_full_scale.hpp"
#include "dsp_toolbox/core/frequency.hpp"
#include "dsp_toolbox/core/linear_gain.hpp"
#include "dsp_toolbox/core/literals.hpp"
#include "dsp_toolbox/core/midi_note_number.hpp"
#include "dsp_toolbox/core/milliseconds.hpp"
#include "dsp_toolbox/core/notes.hpp"
#include "dsp_toolbox/core/sample_rate.hpp"
#include "dsp_toolbox/core/samples.hpp"
#include "dsp_toolbox/core/seconds.hpp"
