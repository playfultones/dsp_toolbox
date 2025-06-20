/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#include "musicality/musical_time.h"
#include <cassert>

using namespace PlayfulTones::DspToolBox::MusicalTime;

int main()
{
    // Test BPM to microseconds conversion
    assert (bpmToMicroseconds (120.0) == 500'000); // 120 BPM = 500,000 microseconds per beat
    assert (bpmToMicroseconds (60.0) == 1'000'000); // 60 BPM = 1 second per beat

    // Test duration to samples conversion
    constexpr double sampleRate = 44100.0;
    constexpr double bpm = 120.0;

    // At 120 BPM, a quarter note is 0.5 seconds
    // At 44100 Hz, 0.5 seconds is 22050 samples
    assert (durationToSamples<QuarterNote> (bpm, sampleRate) == 22050);

    // A whole note at 120 BPM should be 4x longer than a quarter note
    assert (durationToSamples<WholeNote> (bpm, sampleRate) == 88200);

    // Test duration to ticks conversion
    // Quarter note should be PPQN (960) ticks
    assert (durationToTicks<QuarterNote>() == PPQN);

    // Half note should be 2x PPQN
    assert (durationToTicks<HalfNote>() == PPQN * 2);

    // Dotted quarter note should be 1.5x PPQN
    assert (durationToTicks<DottedQuarterNote>() == PPQN * 3 / 2);

    // Test Duration class
    auto quarterDuration = Duration::fromMusicalTime<QuarterNote> (bpm, sampleRate);
    assert (quarterDuration.length() == 22050);

    auto quarterTickDuration = Duration::fromTicks<QuarterNote>();
    assert (quarterTickDuration.length() == PPQN);

    return 0;
}
