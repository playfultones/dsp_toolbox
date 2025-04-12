#include "core/processor_chain.h"
#include "dynamics/gain.h"
#include "generators/noisegenerators.h"
#include <cassert>
#include <iostream>
#include <memory>

using namespace PlayfulTones::DspToolBox;

constexpr auto kGain = 0.5f;

void testBasicSignalFlow()
{
    const int numChannels = 2;
    const double sampleRate = 44100.0;
    const int numFrames = static_cast<int> (sampleRate); // 1 second of audio

    // Create audio buffer
    std::vector<std::vector<float>> audioData (numChannels, std::vector<float> (numFrames, 0.0f));
    std::vector<float*> buffer (numChannels);
    for (int ch = 0; ch < numChannels; ++ch)
        buffer[ch] = audioData[ch].data();

    // Create processing chain
    ProcessorChain chain;

    // Create a gain processor node
    auto gainNode = chain.createNode<Gain>();
    if (auto* gain = gainNode->getProcessor<Gain>())
        gain->setGain (kGain);
    else
        throw std::runtime_error ("Failed to get Gain processor");

    // Set as output node
    chain.setOutputNode (gainNode);

    // Prepare chain
    chain.prepare (sampleRate, numFrames);

    // Fill buffer with white noise
    generateWhiteNoise (buffer.data(), numChannels, numFrames, 1.0f);

    // Store original buffer
    std::vector<std::vector<float>> originalData (numChannels);
    for (int ch = 0; ch < numChannels; ++ch)
    {
        originalData[ch].resize (numFrames);
        std::copy (audioData[ch].begin(), audioData[ch].end(), originalData[ch].begin());
    }

    // Process
    chain.process (buffer.data(), numChannels, numFrames);

    // Verify gain was applied correctly to all samples
    bool allSamplesCorrect = true;
    for (int ch = 0; ch < numChannels; ++ch)
    {
        for (int i = 0; i < numFrames; ++i)
        {
            float expectedValue = originalData[ch][i] * kGain;
            if (std::abs (audioData[ch][i] - expectedValue) > 0.000001f)
            {
                std::cout << "Mismatch at channel " << ch << ", frame " << i
                          << ". Expected: " << expectedValue
                          << ", Got: " << audioData[ch][i] << std::endl;
                allSamplesCorrect = false;
                break;
            }
        }
        if (!allSamplesCorrect)
            break;
    }

    assert (allSamplesCorrect && "All samples should be exactly half of the original values");
    std::cout << "Basic signal flow test passed for 1 second of audio!" << std::endl;
}

int main()
{
    testBasicSignalFlow();
    return 0;
}