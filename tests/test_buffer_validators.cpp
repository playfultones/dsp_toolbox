#include "validators/buffer_validators.h"
#include <cassert>
#include <iostream>
#include <memory>
#include <vector>

using namespace PlayfulTones::DspToolBox;

void testAudioBufferValidation()
{
    const int numChannels = 2;
    const int numFrames = 512;

    // Create valid buffers
    AudioBuffer bufferA (numChannels, numFrames);
    AudioBuffer bufferB (numChannels, numFrames);

    // Test valid case
    assert (validateBufferDimensions (bufferA, bufferB) && "Should validate matching AudioBuffers");
    std::cout << "Valid AudioBuffer dimensions test passed!" << std::endl;

    // Test mismatched channels
    AudioBuffer bufferC (numChannels + 1, numFrames);
    assert (!validateBufferDimensions (bufferA, bufferC) && "Should fail with mismatched channel counts");
    std::cout << "Mismatched channel count test passed!" << std::endl;

    // Test mismatched frames
    AudioBuffer bufferD (numChannels, numFrames * 2);
    assert (!validateBufferDimensions (bufferA, bufferD) && "Should fail with mismatched frame counts");
    std::cout << "Mismatched frame count test passed!" << std::endl;

    // Test buffer comparison
    for (int ch = 0; ch < numChannels; ++ch)
    {
        for (int i = 0; i < numFrames; ++i)
        {
            auto* channelA = bufferA.getChannelPointer (ch);
            channelA[i] = static_cast<float> (i);
            auto* channelB = bufferB.getChannelPointer (ch);
            channelB[i] = static_cast<float> (i);
        }
    }

    assert (compareAudioBuffers (bufferA, bufferB) && "Should match with identical content");
    std::cout << "Identical content comparison test passed!" << std::endl;

    // Test mismatch detection
    bufferB.getChannelPointer (0)[0] += 0.1f;
    assert (!compareAudioBuffers (bufferA, bufferB) && "Should detect sample mismatch");
    std::cout << "Content mismatch detection test passed!" << std::endl;
}

void testRawPointerValidation()
{
    const int numChannels = 2;
    const int numFrames = 512;

    // Create two valid buffers
    std::vector<std::vector<float>> bufferDataA (numChannels, std::vector<float> (numFrames, 0.0f));
    std::vector<std::vector<float>> bufferDataB (numChannels, std::vector<float> (numFrames, 0.0f));
    std::vector<float*> bufferA (numChannels);
    std::vector<float*> bufferB (numChannels);

    for (int ch = 0; ch < numChannels; ++ch)
    {
        bufferA[ch] = bufferDataA[ch].data();
        bufferB[ch] = bufferDataB[ch].data();
    }

    // Test valid case
    assert (validateBufferDimensions (bufferA.data(), bufferB.data(), numChannels, numFrames) && "Should validate valid buffer pointers");
    std::cout << "Valid buffer pointers test passed!" << std::endl;

    // Test null buffer pointers
    assert (!validateBufferDimensions (nullptr, bufferB.data(), numChannels, numFrames) && "Should fail with null first buffer");
    assert (!validateBufferDimensions (bufferA.data(), nullptr, numChannels, numFrames) && "Should fail with null second buffer");
    std::cout << "Null buffer pointer tests passed!" << std::endl;

    // Test invalid channel count
    assert (!validateBufferDimensions (bufferA.data(), bufferB.data(), 0, numFrames) && "Should fail with zero channels");
    assert (!validateBufferDimensions (bufferA.data(), bufferB.data(), -1, numFrames) && "Should fail with negative channels");
    std::cout << "Invalid channel count tests passed!" << std::endl;

    // Test invalid frame count
    assert (!validateBufferDimensions (bufferA.data(), bufferB.data(), numChannels, 0) && "Should fail with zero frames");
    assert (!validateBufferDimensions (bufferA.data(), bufferB.data(), numChannels, -1) && "Should fail with negative frames");
    std::cout << "Invalid frame count tests passed!" << std::endl;

    // Test null channel pointers
    bufferA[0] = nullptr;
    assert (!validateBufferDimensions (bufferA.data(), bufferB.data(), numChannels, numFrames) && "Should fail with null channel pointer");
    std::cout << "Null channel pointer test passed!" << std::endl;
}

int main()
{
    testAudioBufferValidation();
    testRawPointerValidation();
    return 0;
}