/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author           : Bence Kovács
* License          : GNU General Public License v3.0
*******************************************************************/

#include "midi/midi_file.h"
#include "musicality/constants/scales.h"
#include "musicality/generators/melody_generator.h"
#include <filesystem>
#include <iostream>

using namespace PlayfulTones::DspToolBox;

int main()
{
    // Create melody generator
    SimpleMelodyGenerator generator;

    // Set up generator parameters
    // C4 (MIDI note 60) as root note
    generator.setRootNote (60);

    // Use C major scale
    generator.setScale (Scales::Utils::toVector (Scales::Intervals::Major));

    // Set velocity range for some dynamic variation
    generator.setVelocityRange (64, 96);

    // Generate the melody
    auto sequence = generator.generatePattern();

    // Create output filename in current directory
    std::string filename = "generated_melody.mid";

    try
    {
        // Write the MIDI file
        MidiFile::writeToFile (sequence, filename);
        std::cout << "Successfully wrote MIDI file: " << std::filesystem::absolute (filename).string() << std::endl;
    } catch (const std::exception& e)
    {
        std::cerr << "Error writing MIDI file: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
