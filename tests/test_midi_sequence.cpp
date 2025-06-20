/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#include "midi/midi_sequence.h"
#include <cassert>
#include <iostream>

using namespace PlayfulTones::DspToolBox;

void testMidiSequenceBasicOperations()
{
    MidiSequence sequence;

    // Test initial state
    assert (sequence.isEmpty());
    assert (sequence.size() == 0);

    // Add some messages
    MidiMessage noteOn (MidiStatus::NoteOn, 0, 60, 100);
    MidiMessage noteOff (MidiStatus::NoteOff, 0, 60, 0);

    sequence.addMessage (100, noteOn);
    assert (sequence.size() == 1);
    assert (!sequence.isEmpty());

    sequence.addMessage (200, noteOff);
    assert (sequence.size() == 2);

    // Test clear
    sequence.clear();
    assert (sequence.isEmpty());
    assert (sequence.size() == 0);
}

void testMidiSequenceTimeOrdering()
{
    MidiSequence sequence;

    // Add messages in non-chronological order
    MidiMessage msg1 (MidiStatus::NoteOn, 0, 60, 100);
    MidiMessage msg2 (MidiStatus::NoteOff, 0, 60, 0);
    MidiMessage msg3 (MidiStatus::ControlChange, 0, 7, 100);

    sequence.addMessage (300, msg3);
    sequence.addMessage (100, msg1);
    sequence.addMessage (200, msg2);

    // Verify messages are sorted by timestamp
    const auto& messages = sequence.getAllMessages();
    assert (messages[0].timestamp == 100);
    assert (messages[1].timestamp == 200);
    assert (messages[2].timestamp == 300);
}

void testMidiSequenceTimeRangeQueries()
{
    MidiSequence sequence;

    MidiMessage msg1 (MidiStatus::NoteOn, 0, 60, 100);
    MidiMessage msg2 (MidiStatus::NoteOff, 0, 60, 0);
    MidiMessage msg3 (MidiStatus::ControlChange, 0, 7, 100);
    MidiMessage msg4 (MidiStatus::PitchBend, 0, 0, 64);

    sequence.addMessage (100, msg1);
    sequence.addMessage (200, msg2);
    sequence.addMessage (300, msg3);
    sequence.addMessage (400, msg4);

    // Test getMessagesUpTo
    auto upTo250 = sequence.getMessagesUpTo (250);
    assert (upTo250.size() == 2);
    assert (upTo250[0].isNoteOn());
    assert (upTo250[1].isNoteOff());

    // Test getMessagesBetween
    auto between150And350 = sequence.getMessagesBetween (150, 350);
    assert (between150And350.size() == 2);
    assert (between150And350[0].isNoteOff());
    assert (between150And350[1].isControlChange());

    // Test getNextMessage
    const auto* nextMsg = sequence.getNextMessage (250);
    assert (nextMsg != nullptr);
    assert (nextMsg->timestamp == 300);
    assert (nextMsg->message.isControlChange());

    // Test getNextMessage at end
    const auto* noNext = sequence.getNextMessage (400);
    assert (noNext == nullptr);
}

int main()
{
    try
    {
        testMidiSequenceBasicOperations();
        std::cout << "MidiSequence Basic Operations: PASSED" << std::endl;

        testMidiSequenceTimeOrdering();
        std::cout << "MidiSequence Time Ordering: PASSED" << std::endl;

        testMidiSequenceTimeRangeQueries();
        std::cout << "MidiSequence Time Range Queries: PASSED" << std::endl;

        return 0;
    } catch (const std::exception& e)
    {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
}
