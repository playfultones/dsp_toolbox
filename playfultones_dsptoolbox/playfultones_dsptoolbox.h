/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

/** BEGIN_JUCE_MODULE_DECLARATION

    ID:               playfultones_dsptoolbox
    vendor:           Playful Tones
    version:          1.0.0
    name:             DSP Toolbox JUCE Adapter
    description:      JUCE integration for the DSP Toolbox library. Provides zero-copy buffer
                      adapters and generic processor wrappers for seamless integration with
                      JUCE audio processors. Supports runtime sample rate configuration via
                      RuntimeSpec fallback.
    website:          https://playfultones.com
    license:          MIT
    dependencies:     juce_audio_processors

END_JUCE_MODULE_DECLARATION
*/

#pragma once
#define PLAYFULTONES_DSPTOOLBOX_H_INCLUDED

// Core DSP Toolbox headers (relative path to sibling include/ directory)
#include "../include/dsp_toolbox/core/buffer_view.hpp"
#include "../include/dsp_toolbox/core/param.hpp"
#include "../include/dsp_toolbox/core/process_spec.hpp"
#include "../include/dsp_toolbox/processors/core/iprocessor.hpp"
#include "../include/dsp_toolbox/processors/core/processor_base.hpp"

// JUCE headers
#include <juce_audio_processors/juce_audio_processors.h>

// Module components
#include "src/juce_buffer_view.h"
#include "src/juce_parameter_helpers.h"
#include "src/juce_processor_adapter.h"
