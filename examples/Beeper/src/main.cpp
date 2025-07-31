/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author           : Bence Kovács
* License          : GNU General Public License v3.0
*******************************************************************/

#include "core/audio_buffer.h"
#include "generators/adsr.h"
#include "generators/wavegenerators.h"
#include "musicality/musical_time.h"
#include "processors/graph/audio_processor_graph.h"

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace PlayfulTones::DspToolBox;

/**
 * @brief Simple WAV file writer for stereo 16-bit audio
 */
class SimpleWavWriter
{
public:
    static bool writeWavFile (const std::string& filename,
        const std::vector<float>& leftChannel,
        const std::vector<float>& rightChannel,
        uint32_t sampleRate)
    {
        if (leftChannel.size() != rightChannel.size())
            return false;

        std::ofstream file (filename, std::ios::binary);
        if (!file.is_open())
            return false;

        uint32_t numSamples = static_cast<uint32_t> (leftChannel.size());
        uint32_t byteRate = sampleRate * 2 * 2; // 2 channels * 2 bytes per sample
        uint32_t dataSize = numSamples * 2 * 2; // 2 channels * 2 bytes per sample
        uint32_t fileSize = 36 + dataSize;

        // WAV header
        file.write ("RIFF", 4);
        file.write (reinterpret_cast<const char*> (&fileSize), 4);
        file.write ("WAVE", 4);

        // Format chunk
        file.write ("fmt ", 4);
        uint32_t fmtChunkSize = 16;
        file.write (reinterpret_cast<const char*> (&fmtChunkSize), 4);
        uint16_t audioFormat = 1; // PCM
        file.write (reinterpret_cast<const char*> (&audioFormat), 2);
        uint16_t numChannels = 2;
        file.write (reinterpret_cast<const char*> (&numChannels), 2);
        file.write (reinterpret_cast<const char*> (&sampleRate), 4);
        file.write (reinterpret_cast<const char*> (&byteRate), 4);
        uint16_t blockAlign = 4;
        file.write (reinterpret_cast<const char*> (&blockAlign), 2);
        uint16_t bitsPerSample = 16;
        file.write (reinterpret_cast<const char*> (&bitsPerSample), 2);

        // Data chunk
        file.write ("data", 4);
        file.write (reinterpret_cast<const char*> (&dataSize), 4);

        // Write interleaved stereo samples
        for (size_t i = 0; i < numSamples; ++i)
        {
            // Convert float to 16-bit PCM
            int16_t leftSample = static_cast<int16_t> (std::clamp (leftChannel[i], -1.0f, 1.0f) * 32767.0f);
            int16_t rightSample = static_cast<int16_t> (std::clamp (rightChannel[i], -1.0f, 1.0f) * 32767.0f);

            file.write (reinterpret_cast<const char*> (&leftSample), 2);
            file.write (reinterpret_cast<const char*> (&rightSample), 2);
        }

        return file.good();
    }
};

int main (int argc, char* argv[])
{
    // Parse command line arguments
    if (argc != 3)
    {
        std::cout << "Usage: " << argv[0] << " <BPM> <frequency_hz>\n";
        std::cout << "Example: " << argv[0] << " 120 440\n";
        return 1;
    }

    double bpm = std::stod (argv[1]);
    float frequency = std::stof (argv[2]);

    if (bpm <= 0 || frequency <= 0)
    {
        std::cout << "Error: BPM and frequency must be positive numbers\n";
        return 1;
    }

    std::cout << "Generating 10 beeps at " << bpm << " BPM, " << frequency << " Hz\n";

    // Audio configuration
    constexpr size_t sampleRate = 44100;
    constexpr size_t blockSize = 512;
    constexpr size_t numChannels = 2;

    // Calculate timing for eighth notes
    const uint64_t eighthNoteSamples = MusicalTime::durationToSamples<MusicalTime::EighthNote> (bpm, sampleRate);
    const uint64_t totalSamples = eighthNoteSamples * 20; // 10 beeps + 10 silences

    std::cout << "Eighth note duration: " << eighthNoteSamples << " samples\n";
    std::cout << "Total duration: " << totalSamples / sampleRate << " seconds\n";

    // Create audio graph
    AudioProcessorGraph<float, blockSize, sampleRate, numChannels> graph;

    // Add processors to the graph
    auto sineNodeId = graph.addProcessor<SineWaveGenerator<float, blockSize, sampleRate, numChannels>> (frequency, 0.5f);
    auto adsrNodeId = graph.addProcessor<ADSR<float, blockSize, sampleRate, numChannels>>();

    // Connect sine generator to ADSR envelope
    graph.connect (sineNodeId, adsrNodeId);

    // Connect ADSR to output
    auto outputNode = graph.getOutputNode();
    if (outputNode)
    {
        graph.connect (adsrNodeId, outputNode->getId());
    }

    // Get processor references for parameter control
    auto* sineProcessor = graph.getProcessor<SineWaveGenerator<float, blockSize, sampleRate, numChannels>> (sineNodeId);
    auto* adsrProcessor = graph.getProcessor<ADSR<float, blockSize, sampleRate, numChannels>> (adsrNodeId);

    if (!sineProcessor || !adsrProcessor)
    {
        std::cout << "Error: Failed to get processor references\n";
        return 1;
    }

    // Configure ADSR envelope (quick attack/release for beep effect)
    adsrProcessor->setAttack (0.01); // 10ms attack
    adsrProcessor->setDecay (0.0); // No decay
    adsrProcessor->setSustain (1.0); // Full sustain
    adsrProcessor->setRelease (0.05); // 50ms release

    // Configure sine wave
    sineProcessor->setFrequency (frequency);
    sineProcessor->setGain (0.5f);

    // Prepare the graph
    graph.prepare();

    // Create output buffers
    std::vector<float> leftChannel, rightChannel;
    leftChannel.reserve (totalSamples);
    rightChannel.reserve (totalSamples);

    // Create audio buffer for processing
    AudioBufferStorage<float, blockSize, numChannels> bufferStorage;

    // Generate the sequence
    uint64_t samplesProcessed = 0;
    int beepCount = 0;
    bool inBeep = true;
    uint64_t nextEventSample = eighthNoteSamples;

    std::cout << "Generating audio...\n";

    while (samplesProcessed < totalSamples)
    {
        // Calculate how many samples to process in this block
        size_t samplesToProcess = std::min (blockSize, static_cast<size_t> (totalSamples - samplesProcessed));

        // Check if we need to trigger events in this block
        if (samplesProcessed + samplesToProcess >= nextEventSample)
        {
            if (inBeep)
            {
                // End of beep - trigger note off
                adsrProcessor->noteOff();
                inBeep = false;
                nextEventSample += eighthNoteSamples;
                std::cout << "Beep " << (beepCount + 1) << " ended\n";
            }
            else
            {
                // End of silence - trigger next beep
                beepCount++;
                if (beepCount < 10)
                {
                    adsrProcessor->noteOn();
                    inBeep = true;
                    nextEventSample += eighthNoteSamples;
                    std::cout << "Beep " << beepCount << " started\n";
                }
            }
        }

        // Process audio block
        auto& buffer = bufferStorage.getBuffer();
        bufferStorage.clear();

        // Start first beep
        if (samplesProcessed == 0)
        {
            adsrProcessor->noteOn();
            std::cout << "Beep 1 started\n";
        }

        graph.process_audio (buffer);

        // Copy processed samples to output buffers
        for (size_t i = 0; i < samplesToProcess; ++i)
        {
            leftChannel.push_back (buffer[0][i]);
            rightChannel.push_back (buffer[1][i]);
        }

        samplesProcessed += samplesToProcess;
    }

    // Write to WAV file
    std::string filename = "beeps_" + std::to_string (static_cast<int> (bpm)) + "bpm_" + std::to_string (static_cast<int> (frequency)) + "hz.wav";

    std::cout << "Writing to file: " << filename << "\n";

    if (SimpleWavWriter::writeWavFile (filename, leftChannel, rightChannel, sampleRate))
    {
        std::cout << "Successfully wrote " << filename << "\n";
        std::cout << "File size: " << leftChannel.size() << " samples (" << static_cast<double> (leftChannel.size()) / sampleRate << " seconds)\n";
    }
    else
    {
        std::cout << "Error: Failed to write WAV file\n";
        return 1;
    }

    return 0;
}
