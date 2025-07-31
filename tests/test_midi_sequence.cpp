/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#include "helpers/compilationhelpers.h"
#include "midi/midi_sequence.h"
#include <cassert>
#include <iostream>
#include <vector>

using namespace PlayfulTones::DspToolBox;

void testMidiSequenceBasicOperations()
{
    MidiSequence64 sequence;

    // Test initial state
    assert(sequence.isEmpty());
    assert(sequence.size() == 0);
    assert(sequence.capacity() == 1024);

    // Add some messages with strong types
    MidiMessage noteOn(MidiStatus::NoteOn, MidiChannel(0), NoteNumber(60), Velocity(100));
    MidiMessage noteOff(MidiStatus::NoteOff, MidiChannel(0), NoteNumber(60), Velocity(0));

    assert(sequence.addMessage(Timestamp<uint64_t>(100), noteOn));
    assert(sequence.size() == 1);
    assert(!sequence.isEmpty());

    assert(sequence.addMessage(Timestamp<uint64_t>(200), noteOff));
    assert(sequence.size() == 2);

    // Test clear
    sequence.clear();
    assert(sequence.isEmpty());
    assert(sequence.size() == 0);
}

void testMidiSequenceTimeOrdering()
{
    MidiSequence64 sequence;

    // Add messages in non-chronological order with strong types
    MidiMessage msg1(MidiStatus::NoteOn, MidiChannel(0), NoteNumber(60), Velocity(100));
    MidiMessage msg2(MidiStatus::NoteOff, MidiChannel(0), NoteNumber(60), Velocity(0));
    MidiMessage msg3(MidiChannel(0), ControllerNumber(7), ControllerValue(100));

    assert(sequence.addMessage(Timestamp<uint64_t>(300), msg3));
    assert(sequence.addMessage(Timestamp<uint64_t>(100), msg1));
    assert(sequence.addMessage(Timestamp<uint64_t>(200), msg2));

    // Verify messages are sorted by timestamp
    const auto messages = sequence.getAllMessages();
    assert(static_cast<uint64_t>(messages[0].timestamp) == 100);
    assert(static_cast<uint64_t>(messages[1].timestamp) == 200);
    assert(static_cast<uint64_t>(messages[2].timestamp) == 300);
    markUsed(messages);
}

void testMidiSequenceTimeRangeQueries()
{
    MidiSequence64 sequence;

    MidiMessage msg1(MidiStatus::NoteOn, MidiChannel(0), NoteNumber(60), Velocity(100));
    MidiMessage msg2(MidiStatus::NoteOff, MidiChannel(0), NoteNumber(60), Velocity(0));
    MidiMessage msg3(MidiChannel(0), ControllerNumber(7), ControllerValue(100));
    MidiMessage msg4(MidiStatus::PitchBend, MidiChannel(0), 0, 64);

    assert(sequence.addMessage(Timestamp<uint64_t>(100), msg1));
    assert(sequence.addMessage(Timestamp<uint64_t>(200), msg2));
    assert(sequence.addMessage(Timestamp<uint64_t>(300), msg3));
    assert(sequence.addMessage(Timestamp<uint64_t>(400), msg4));

    // Test processMessagesUpTo (real-time safe)
    std::vector<MidiMessage> upTo250;
    sequence.processMessagesUpTo(Timestamp<uint64_t>(250), [&](const MidiMessage& msg) {
        upTo250.push_back(msg);
    });
    assert(upTo250.size() == 2);
    assert(upTo250[0].isNoteOn());
    assert(upTo250[1].isNoteOff());

    // Test processMessagesBetween (real-time safe)
    std::vector<MidiMessage> between150And350;
    sequence.processMessagesBetween(Timestamp<uint64_t>(150), Timestamp<uint64_t>(350), 
                                  [&](const MidiMessage& msg) {
        between150And350.push_back(msg);
    });
    assert(between150And350.size() == 2);
    assert(between150And350[0].isNoteOff());
    assert(between150And350[1].isControlChange());

    // Test getNextMessage
    const auto* nextMsg = sequence.getNextMessage(Timestamp<uint64_t>(250));
    assert(nextMsg != nullptr);
    assert(static_cast<uint64_t>(nextMsg->timestamp) == 300);
    assert(nextMsg->message.isControlChange());
    markUsed(nextMsg);

    // Test getNextMessage at end
    const auto* noNext = sequence.getNextMessage(Timestamp<uint64_t>(400));
    assert(noNext == nullptr);
    markUsed(noNext);
}

void testMidiSequenceCapacityAndRemoval()
{
    // Test with small capacity
    MidiSequence<uint64_t, 3> smallSequence;
    
    MidiMessage msg1(MidiStatus::NoteOn, MidiChannel(0), NoteNumber(60), Velocity(100));
    MidiMessage msg2(MidiStatus::NoteOff, MidiChannel(0), NoteNumber(60), Velocity(0));
    MidiMessage msg3(MidiChannel(0), ControllerNumber(7), ControllerValue(100));
    MidiMessage msg4(MidiStatus::PitchBend, MidiChannel(0), 0, 64);

    // Fill to capacity
    assert(smallSequence.addMessage(Timestamp<uint64_t>(100), msg1));
    assert(smallSequence.addMessage(Timestamp<uint64_t>(200), msg2));
    assert(smallSequence.addMessage(Timestamp<uint64_t>(300), msg3));
    assert(smallSequence.isFull());
    
    // Should fail to add when full
    assert(!smallSequence.addMessage(Timestamp<uint64_t>(400), msg4));
    assert(smallSequence.size() == 3);

    // Test removeMessagesOlderThan
    auto removed = smallSequence.removeMessagesOlderThan(Timestamp<uint64_t>(250));
    assert(removed == 2);
    assert(smallSequence.size() == 1);
    assert(static_cast<uint64_t>(smallSequence[0].timestamp) == 300);
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

        testMidiSequenceCapacityAndRemoval();
        std::cout << "MidiSequence Capacity and Removal: PASSED" << std::endl;

        return 0;
    } catch (const std::exception& e)
    {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
}
