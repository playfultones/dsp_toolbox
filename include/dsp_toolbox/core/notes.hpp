/*
 * DSP Toolbox | Playful Tones
 *
 * 2026
 */

#pragma once

#include "dsp_toolbox/core/midi_note_number.hpp"

namespace PlayfulTones::DspToolbox::Notes
{

    // Octave -1 (MIDI 0-11)
    inline constexpr MIDINoteNumber Cn1 { 0 };
    inline constexpr MIDINoteNumber Csn1 { 1 };
    inline constexpr MIDINoteNumber Dfn1 { 1 };
    inline constexpr MIDINoteNumber Dn1 { 2 };
    inline constexpr MIDINoteNumber Dsn1 { 3 };
    inline constexpr MIDINoteNumber Efn1 { 3 };
    inline constexpr MIDINoteNumber En1 { 4 };
    inline constexpr MIDINoteNumber Fn1 { 5 };
    inline constexpr MIDINoteNumber Fsn1 { 6 };
    inline constexpr MIDINoteNumber Gfn1 { 6 };
    inline constexpr MIDINoteNumber Gn1 { 7 };
    inline constexpr MIDINoteNumber Gsn1 { 8 };
    inline constexpr MIDINoteNumber Afn1 { 8 };
    inline constexpr MIDINoteNumber An1 { 9 };
    inline constexpr MIDINoteNumber Asn1 { 10 };
    inline constexpr MIDINoteNumber Bfn1 { 10 };
    inline constexpr MIDINoteNumber Bn1 { 11 };

    // Octave 0 (MIDI 12-23)
    inline constexpr MIDINoteNumber C0 { 12 };
    inline constexpr MIDINoteNumber Cs0 { 13 };
    inline constexpr MIDINoteNumber Df0 { 13 };
    inline constexpr MIDINoteNumber D0 { 14 };
    inline constexpr MIDINoteNumber Ds0 { 15 };
    inline constexpr MIDINoteNumber Ef0 { 15 };
    inline constexpr MIDINoteNumber E0 { 16 };
    inline constexpr MIDINoteNumber F0 { 17 };
    inline constexpr MIDINoteNumber Fs0 { 18 };
    inline constexpr MIDINoteNumber Gf0 { 18 };
    inline constexpr MIDINoteNumber G0 { 19 };
    inline constexpr MIDINoteNumber Gs0 { 20 };
    inline constexpr MIDINoteNumber Af0 { 20 };
    inline constexpr MIDINoteNumber A0 { 21 };
    inline constexpr MIDINoteNumber As0 { 22 };
    inline constexpr MIDINoteNumber Bf0 { 22 };
    inline constexpr MIDINoteNumber B0 { 23 };

    // Octave 1 (MIDI 24-35)
    inline constexpr MIDINoteNumber C1 { 24 };
    inline constexpr MIDINoteNumber Cs1 { 25 };
    inline constexpr MIDINoteNumber Df1 { 25 };
    inline constexpr MIDINoteNumber D1 { 26 };
    inline constexpr MIDINoteNumber Ds1 { 27 };
    inline constexpr MIDINoteNumber Ef1 { 27 };
    inline constexpr MIDINoteNumber E1 { 28 };
    inline constexpr MIDINoteNumber F1 { 29 };
    inline constexpr MIDINoteNumber Fs1 { 30 };
    inline constexpr MIDINoteNumber Gf1 { 30 };
    inline constexpr MIDINoteNumber G1 { 31 };
    inline constexpr MIDINoteNumber Gs1 { 32 };
    inline constexpr MIDINoteNumber Af1 { 32 };
    inline constexpr MIDINoteNumber A1 { 33 };
    inline constexpr MIDINoteNumber As1 { 34 };
    inline constexpr MIDINoteNumber Bf1 { 34 };
    inline constexpr MIDINoteNumber B1 { 35 };

    // Octave 2 (MIDI 36-47)
    inline constexpr MIDINoteNumber C2 { 36 };
    inline constexpr MIDINoteNumber Cs2 { 37 };
    inline constexpr MIDINoteNumber Df2 { 37 };
    inline constexpr MIDINoteNumber D2 { 38 };
    inline constexpr MIDINoteNumber Ds2 { 39 };
    inline constexpr MIDINoteNumber Ef2 { 39 };
    inline constexpr MIDINoteNumber E2 { 40 };
    inline constexpr MIDINoteNumber F2 { 41 };
    inline constexpr MIDINoteNumber Fs2 { 42 };
    inline constexpr MIDINoteNumber Gf2 { 42 };
    inline constexpr MIDINoteNumber G2 { 43 };
    inline constexpr MIDINoteNumber Gs2 { 44 };
    inline constexpr MIDINoteNumber Af2 { 44 };
    inline constexpr MIDINoteNumber A2 { 45 };
    inline constexpr MIDINoteNumber As2 { 46 };
    inline constexpr MIDINoteNumber Bf2 { 46 };
    inline constexpr MIDINoteNumber B2 { 47 };

    // Octave 3 (MIDI 48-59)
    inline constexpr MIDINoteNumber C3 { 48 };
    inline constexpr MIDINoteNumber Cs3 { 49 };
    inline constexpr MIDINoteNumber Df3 { 49 };
    inline constexpr MIDINoteNumber D3 { 50 };
    inline constexpr MIDINoteNumber Ds3 { 51 };
    inline constexpr MIDINoteNumber Ef3 { 51 };
    inline constexpr MIDINoteNumber E3 { 52 };
    inline constexpr MIDINoteNumber F3 { 53 };
    inline constexpr MIDINoteNumber Fs3 { 54 };
    inline constexpr MIDINoteNumber Gf3 { 54 };
    inline constexpr MIDINoteNumber G3 { 55 };
    inline constexpr MIDINoteNumber Gs3 { 56 };
    inline constexpr MIDINoteNumber Af3 { 56 };
    inline constexpr MIDINoteNumber A3 { 57 };
    inline constexpr MIDINoteNumber As3 { 58 };
    inline constexpr MIDINoteNumber Bf3 { 58 };
    inline constexpr MIDINoteNumber B3 { 59 };

    // Octave 4 (MIDI 60-71) - Middle C octave
    inline constexpr MIDINoteNumber C4 { 60 }; // Middle C
    inline constexpr MIDINoteNumber Cs4 { 61 };
    inline constexpr MIDINoteNumber Df4 { 61 };
    inline constexpr MIDINoteNumber D4 { 62 };
    inline constexpr MIDINoteNumber Ds4 { 63 };
    inline constexpr MIDINoteNumber Ef4 { 63 };
    inline constexpr MIDINoteNumber E4 { 64 };
    inline constexpr MIDINoteNumber F4 { 65 };
    inline constexpr MIDINoteNumber Fs4 { 66 };
    inline constexpr MIDINoteNumber Gf4 { 66 };
    inline constexpr MIDINoteNumber G4 { 67 };
    inline constexpr MIDINoteNumber Gs4 { 68 };
    inline constexpr MIDINoteNumber Af4 { 68 };
    inline constexpr MIDINoteNumber A4 { 69 }; // Concert pitch (440 Hz)
    inline constexpr MIDINoteNumber As4 { 70 };
    inline constexpr MIDINoteNumber Bf4 { 70 };
    inline constexpr MIDINoteNumber B4 { 71 };

    // Octave 5 (MIDI 72-83)
    inline constexpr MIDINoteNumber C5 { 72 };
    inline constexpr MIDINoteNumber Cs5 { 73 };
    inline constexpr MIDINoteNumber Df5 { 73 };
    inline constexpr MIDINoteNumber D5 { 74 };
    inline constexpr MIDINoteNumber Ds5 { 75 };
    inline constexpr MIDINoteNumber Ef5 { 75 };
    inline constexpr MIDINoteNumber E5 { 76 };
    inline constexpr MIDINoteNumber F5 { 77 };
    inline constexpr MIDINoteNumber Fs5 { 78 };
    inline constexpr MIDINoteNumber Gf5 { 78 };
    inline constexpr MIDINoteNumber G5 { 79 };
    inline constexpr MIDINoteNumber Gs5 { 80 };
    inline constexpr MIDINoteNumber Af5 { 80 };
    inline constexpr MIDINoteNumber A5 { 81 };
    inline constexpr MIDINoteNumber As5 { 82 };
    inline constexpr MIDINoteNumber Bf5 { 82 };
    inline constexpr MIDINoteNumber B5 { 83 };

    // Octave 6 (MIDI 84-95)
    inline constexpr MIDINoteNumber C6 { 84 };
    inline constexpr MIDINoteNumber Cs6 { 85 };
    inline constexpr MIDINoteNumber Df6 { 85 };
    inline constexpr MIDINoteNumber D6 { 86 };
    inline constexpr MIDINoteNumber Ds6 { 87 };
    inline constexpr MIDINoteNumber Ef6 { 87 };
    inline constexpr MIDINoteNumber E6 { 88 };
    inline constexpr MIDINoteNumber F6 { 89 };
    inline constexpr MIDINoteNumber Fs6 { 90 };
    inline constexpr MIDINoteNumber Gf6 { 90 };
    inline constexpr MIDINoteNumber G6 { 91 };
    inline constexpr MIDINoteNumber Gs6 { 92 };
    inline constexpr MIDINoteNumber Af6 { 92 };
    inline constexpr MIDINoteNumber A6 { 93 };
    inline constexpr MIDINoteNumber As6 { 94 };
    inline constexpr MIDINoteNumber Bf6 { 94 };
    inline constexpr MIDINoteNumber B6 { 95 };

    // Octave 7 (MIDI 96-107)
    inline constexpr MIDINoteNumber C7 { 96 };
    inline constexpr MIDINoteNumber Cs7 { 97 };
    inline constexpr MIDINoteNumber Df7 { 97 };
    inline constexpr MIDINoteNumber D7 { 98 };
    inline constexpr MIDINoteNumber Ds7 { 99 };
    inline constexpr MIDINoteNumber Ef7 { 99 };
    inline constexpr MIDINoteNumber E7 { 100 };
    inline constexpr MIDINoteNumber F7 { 101 };
    inline constexpr MIDINoteNumber Fs7 { 102 };
    inline constexpr MIDINoteNumber Gf7 { 102 };
    inline constexpr MIDINoteNumber G7 { 103 };
    inline constexpr MIDINoteNumber Gs7 { 104 };
    inline constexpr MIDINoteNumber Af7 { 104 };
    inline constexpr MIDINoteNumber A7 { 105 };
    inline constexpr MIDINoteNumber As7 { 106 };
    inline constexpr MIDINoteNumber Bf7 { 106 };
    inline constexpr MIDINoteNumber B7 { 107 };

    // Octave 8 (MIDI 108-119)
    inline constexpr MIDINoteNumber C8 { 108 }; // Highest C on piano
    inline constexpr MIDINoteNumber Cs8 { 109 };
    inline constexpr MIDINoteNumber Df8 { 109 };
    inline constexpr MIDINoteNumber D8 { 110 };
    inline constexpr MIDINoteNumber Ds8 { 111 };
    inline constexpr MIDINoteNumber Ef8 { 111 };
    inline constexpr MIDINoteNumber E8 { 112 };
    inline constexpr MIDINoteNumber F8 { 113 };
    inline constexpr MIDINoteNumber Fs8 { 114 };
    inline constexpr MIDINoteNumber Gf8 { 114 };
    inline constexpr MIDINoteNumber G8 { 115 };
    inline constexpr MIDINoteNumber Gs8 { 116 };
    inline constexpr MIDINoteNumber Af8 { 116 };
    inline constexpr MIDINoteNumber A8 { 117 };
    inline constexpr MIDINoteNumber As8 { 118 };
    inline constexpr MIDINoteNumber Bf8 { 118 };
    inline constexpr MIDINoteNumber B8 { 119 };

    // Octave 9 (MIDI 120-127) - Partial octave
    inline constexpr MIDINoteNumber C9 { 120 };
    inline constexpr MIDINoteNumber Cs9 { 121 };
    inline constexpr MIDINoteNumber Df9 { 121 };
    inline constexpr MIDINoteNumber D9 { 122 };
    inline constexpr MIDINoteNumber Ds9 { 123 };
    inline constexpr MIDINoteNumber Ef9 { 123 };
    inline constexpr MIDINoteNumber E9 { 124 };
    inline constexpr MIDINoteNumber F9 { 125 };
    inline constexpr MIDINoteNumber Fs9 { 126 };
    inline constexpr MIDINoteNumber Gf9 { 126 };
    inline constexpr MIDINoteNumber G9 { 127 }; // Highest MIDI note

} // namespace PlayfulTones::DspToolbox::Notes
