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

    // Create audio buffers
    AudioBuffer audioBuffer (numChannels, numFrames);
    AudioBuffer expectedBuffer (numChannels, numFrames);

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
    generateWhiteNoise (audioBuffer.getArrayOfChannels(), numChannels, numFrames, 1.0f);

    // Copy to expected buffer and apply gain
    for (int ch = 0; ch < numChannels; ++ch)
    {
        const auto* srcData = audioBuffer.getChannelPointer (ch);
        auto* dstData = expectedBuffer.getChannelPointer (ch);
        for (int i = 0; i < numFrames; ++i)
            dstData[i] = srcData[i] * kGain;
    }

    // Process using BufferView
    BufferView view;
    view.setData (audioBuffer.getArrayOfChannels(), numChannels, numFrames);
    chain.process (view);

    // Verify gain was applied correctly using the buffer validator
    bool allSamplesCorrect = compareAudioBuffers (audioBuffer, expectedBuffer);
    assert (allSamplesCorrect && "All samples should be exactly half of the original values");
    std::cout << "Basic signal flow test passed for 1 second of audio!" << std::endl;
}

int main()
{
    testBasicSignalFlow();
    return 0;
}