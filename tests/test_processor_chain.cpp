#include "core/processor_chain.h"
#include "dynamics/gain.h"
#include "generators/noisegenerators.h"
#include "validators/buffer_validators.h"
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

    // Create expected buffer with gain applied
    std::vector<std::vector<float>> expectedData (numChannels);
    std::vector<float*> expectedBuffer (numChannels);
    for (int ch = 0; ch < numChannels; ++ch)
    {
        expectedData[ch].resize (numFrames);
        expectedBuffer[ch] = expectedData[ch].data();
        // Copy and apply gain
        for (int i = 0; i < numFrames; ++i)
            expectedData[ch][i] = audioData[ch][i] * kGain;
    }

    // Process
    chain.process (buffer.data(), numChannels, numFrames);

    // Verify gain was applied correctly using the buffer validator
    bool allSamplesCorrect = compareAudioBuffers (buffer.data(), expectedBuffer.data(), numChannels, numFrames);
    assert (allSamplesCorrect && "All samples should be exactly half of the original values");
    std::cout << "Basic signal flow test passed for 1 second of audio!" << std::endl;
}

int main()
{
    testBasicSignalFlow();
    return 0;
}