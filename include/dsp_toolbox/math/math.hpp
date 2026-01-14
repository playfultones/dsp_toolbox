/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

/**
 * @file math.hpp
 * @brief Umbrella header for the constexpr math library.
 *
 * Includes all math-related headers:
 * - constants.hpp: Mathematical constants (pi, e, ln2, etc.)
 * - functions.hpp: Transcendental functions (sin, cos, exp, log, etc.)
 *
 * All functions are constexpr and can be evaluated at compile time.
 *
 * @code
 * #include <dsp_toolbox/math/math.hpp>
 *
 * using namespace PlayfulTones::DspToolbox::Math;
 *
 * // Compile-time evaluation
 * constexpr double sineValue = sin(pi<double> / 4.0);
 * static_assert(sineValue > 0.707 && sineValue < 0.708);
 * @endcode
 */

#include "dsp_toolbox/math/constants.hpp"
#include "dsp_toolbox/math/functions.hpp"
