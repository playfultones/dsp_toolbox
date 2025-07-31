/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#include "midi/midi_message.h"
#include <cassert>
#include <iostream>

using namespace PlayfulTones::DspToolBox;

void testMidiMessageConstructionAndBasicProperties()
{
    // Channel Voice Message Construction with strong types
    {
        MidiMessage noteOn(MidiStatus::NoteOn, MidiChannel(0), NoteNumber(60), Velocity(100));
        assert(noteOn.getStatus() == 0x90);
        assert(static_cast<uint8_t>(noteOn.getChannel()) == 0);
        assert(static_cast<uint8_t>(noteOn.getNoteNumber()) == 60);
        assert(static_cast<uint8_t>(noteOn.getVelocity()) == 100);
        assert(noteOn.getData1() == 60);
        assert(noteOn.getData2() == 100);
    }

    // Control Change Construction with strong types
    {
        MidiMessage cc(MidiChannel(1), ControllerNumber(7), ControllerValue(127));
        assert(cc.getStatus() == 0xB1);
        assert(static_cast<uint8_t>(cc.getChannel()) == 1);
        assert(static_cast<uint8_t>(cc.getControllerNumber()) == 7);
        assert(static_cast<uint8_t>(cc.getControllerValue()) == 127);
    }

    // System Message Construction
    {
        MidiMessage sysEx(0xF0, 0x7E, 0x7F);
        assert(sysEx.getStatus() == 0xF0);
        assert(sysEx.getData1() == 0x7E);
        assert(sysEx.getData2() == 0x7F);
    }
}

void testMidiMessageChannelHandling()
{
    // Test all 16 MIDI channels with strong types
    for (uint8_t channel = 0; channel < 16; ++channel)
    {
        MidiMessage msg(MidiStatus::NoteOn, MidiChannel(channel), NoteNumber(60), Velocity(100));
        assert(static_cast<uint8_t>(msg.getChannel()) == channel);
        assert((msg.getStatus() & 0x0F) == channel);
        assert((msg.getStatus() & 0xF0) == 0x90);
    }

    // Test that channel values > 15 are properly masked by MidiChannel
    MidiMessage msg(MidiStatus::NoteOn, MidiChannel(20), NoteNumber(60), Velocity(100));
    assert(static_cast<uint8_t>(msg.getChannel()) == 4); // 20 & 0x0F = 4
}

void testMidiMessageTypeDetection()
{
    // Note On Detection
    {
        MidiMessage noteOn(MidiStatus::NoteOn, MidiChannel(0), NoteNumber(60), Velocity(100));
        assert(noteOn.isNoteOn());
        assert(!noteOn.isNoteOff());

        // Note-on with velocity 0 should not be detected as note-on
        MidiMessage noteOnZeroVel(MidiStatus::NoteOn, MidiChannel(0), NoteNumber(60), Velocity(0));
        assert(!noteOnZeroVel.isNoteOn());
        assert(noteOnZeroVel.isNoteOff());
    }

    // Note Off Detection
    {
        MidiMessage noteOff(MidiStatus::NoteOff, MidiChannel(0), NoteNumber(60), Velocity(0));
        assert(noteOff.isNoteOff());
        assert(!noteOff.isNoteOn());

        // Note-on with velocity 0 should be detected as note-off
        MidiMessage noteOnZeroVel(MidiStatus::NoteOn, MidiChannel(0), NoteNumber(60), Velocity(0));
        assert(noteOnZeroVel.isNoteOff());
    }

    // Control Change Detection
    {
        MidiMessage cc(MidiChannel(0), ControllerNumber(7), ControllerValue(100));
        assert(cc.isControlChange());
        assert(!cc.isNoteOn());
        assert(!cc.isNoteOff());
    }

    // Pitch Bend Detection
    {
        MidiMessage pb(MidiStatus::PitchBend, MidiChannel(0), 0, 64);
        assert(pb.isPitchBend());
        assert(!pb.isControlChange());
        assert(!pb.isNoteOn());
        assert(!pb.isNoteOff());
    }
}

void testMidiMessageStatusByteOperations()
{
    // Test status byte composition for each message type
    struct TestCase
    {
        MidiStatus status;
        uint8_t channel;
        uint8_t expectedStatus;
    };

    TestCase testCases[] = {
        { MidiStatus::NoteOff, 0, 0x80 },
        { MidiStatus::NoteOn, 1, 0x91 },
        { MidiStatus::PolyphonicPressure, 2, 0xA2 },
        { MidiStatus::ControlChange, 3, 0xB3 },
        { MidiStatus::ProgramChange, 4, 0xC4 },
        { MidiStatus::ChannelPressure, 5, 0xD5 },
        { MidiStatus::PitchBend, 6, 0xE6 }
    };

    for (const auto& tc : testCases)
    {
        MidiMessage msg(tc.status, MidiChannel(tc.channel), 0, 0);
        assert(msg.getStatus() == tc.expectedStatus);
    }

    // Test that status byte can be correctly decomposed into type and channel
    MidiMessage msg(MidiStatus::NoteOn, MidiChannel(5), NoteNumber(60), Velocity(100));
    assert((msg.getStatus() & 0xF0) == static_cast<uint8_t>(MidiStatus::NoteOn));
    assert((msg.getStatus() & 0x0F) == 5);

    // Test pack/unpack functionality
    auto packed = msg.pack();
    auto unpacked = MidiMessage::unpack(packed);
    assert(unpacked == msg);
}

int main()
{
    try
    {
        testMidiMessageConstructionAndBasicProperties();
        std::cout << "MidiMessage Construction and Basic Properties: PASSED" << std::endl;

        testMidiMessageChannelHandling();
        std::cout << "MidiMessage Channel Handling: PASSED" << std::endl;

        testMidiMessageTypeDetection();
        std::cout << "MidiMessage Type Detection: PASSED" << std::endl;

        testMidiMessageStatusByteOperations();
        std::cout << "MidiMessage Status Byte Operations: PASSED" << std::endl;

        return 0;
    } catch (const std::exception& e)
    {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
}
