/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#include "../validators/buffer_validators.h"
#include <cassert>
#include <iostream>
#include <limits>

using namespace PlayfulTones::DspToolBox;

void testBasicBufferValidation()
{
    std::cout << "Testing basic buffer validation..." << std::endl;
    
    // Create buffers with matching dimensions
    constexpr size_t BlockSize = 512;
    constexpr size_t NumChannels = 2;
    using SampleType = float;
    
    AudioBufferStorage<SampleType, BlockSize, NumChannels> storageA;
    AudioBufferStorage<SampleType, BlockSize, NumChannels> storageB;
    
    auto& bufferA = storageA.getBuffer();
    auto& bufferB = storageB.getBuffer();

    // Test valid case
    assert(validateBufferDimensions(bufferA, bufferB) && "Should validate matching AudioBuffers");
    std::cout << "✓ Valid buffer dimensions test passed!" << std::endl;

    // Test single buffer validation
    assert(validateSingleBuffer(bufferA) && "Should validate single buffer");
    std::cout << "✓ Single buffer validation test passed!" << std::endl;
}

void testBufferComparison()
{
    std::cout << "Testing buffer comparison..." << std::endl;
    
    constexpr size_t BlockSize = 128;
    constexpr size_t NumChannels = 2;
    using SampleType = float;
    
    AudioBufferStorage<SampleType, BlockSize, NumChannels> storageA;
    AudioBufferStorage<SampleType, BlockSize, NumChannels> storageB;
    
    auto& bufferA = storageA.getBuffer();
    auto& bufferB = storageB.getBuffer();

    // Fill buffers with identical data
    for (size_t ch = 0; ch < NumChannels; ++ch)
    {
        for (size_t i = 0; i < BlockSize; ++i)
        {
            bufferA[ch][i] = static_cast<float>(i * 0.01f);
            bufferB[ch][i] = static_cast<float>(i * 0.01f);
        }
    }

    // Test exact comparison
    assert(compareAudioBuffersExact(bufferA, bufferB) && "Should match with identical content");
    std::cout << "✓ Exact buffer comparison test passed!" << std::endl;

    // Test threshold comparison
    assert(compareAudioBuffers(bufferA, bufferB, 0.00001f, false) && "Should match within threshold");
    std::cout << "✓ Threshold buffer comparison test passed!" << std::endl;

    // Test mismatch detection
    bufferB[0][0] += 0.1f;
    assert(!compareAudioBuffersExact(bufferA, bufferB) && "Should detect exact mismatch");
    assert(!compareAudioBuffers(bufferA, bufferB, 0.00001f, false) && "Should detect sample mismatch");
    std::cout << "✓ Mismatch detection test passed!" << std::endl;

    // Test within threshold
    bufferB[0][0] = bufferA[0][0] + 0.00001f;
    assert(compareAudioBuffers(bufferA, bufferB, 0.0001f, false) && "Should match within larger threshold");
    std::cout << "✓ Threshold tolerance test passed!" << std::endl;
}

void testDifferentBufferSizes()
{
    std::cout << "Testing different buffer sizes..." << std::endl;
    
    // Different block sizes
    AudioBufferStorage<float, 256, 2> storage256;
    AudioBufferStorage<float, 512, 2> storage512;
    
    auto& buffer256 = storage256.getBuffer();
    auto& buffer512 = storage512.getBuffer();

    // Should fail dimension validation due to different frame counts
    assert(!validateBufferDimensions(buffer256, buffer512) && "Should fail with different block sizes");
    std::cout << "✓ Different block size validation test passed!" << std::endl;

    // Different channel counts
    AudioBufferStorage<float, 256, 2> storage2ch;
    AudioBufferStorage<float, 256, 4> storage4ch;
    
    auto& buffer2ch = storage2ch.getBuffer();
    auto& buffer4ch = storage4ch.getBuffer();

    assert(!validateBufferDimensions(buffer2ch, buffer4ch) && "Should fail with different channel counts");
    std::cout << "✓ Different channel count validation test passed!" << std::endl;
}

void testFiniteValueValidation()
{
    std::cout << "Testing finite value validation..." << std::endl;
    
    constexpr size_t BlockSize = 64;
    constexpr size_t NumChannels = 1;
    
    AudioBufferStorage<float, BlockSize, NumChannels> storage;
    auto& buffer = storage.getBuffer();

    // Fill with valid values
    for (size_t i = 0; i < BlockSize; ++i)
    {
        buffer[0][i] = static_cast<float>(i * 0.1f);
    }

    assert(validateFiniteValues(buffer, false) && "Should validate finite values");
    std::cout << "✓ Finite values validation test passed!" << std::endl;

    // Test NaN detection
    buffer[0][10] = std::numeric_limits<float>::quiet_NaN();
    assert(!validateFiniteValues(buffer, false) && "Should detect NaN values");
    std::cout << "✓ NaN detection test passed!" << std::endl;

    // Test infinity detection
    buffer[0][10] = std::numeric_limits<float>::infinity();
    assert(!validateFiniteValues(buffer, false) && "Should detect infinite values");
    std::cout << "✓ Infinity detection test passed!" << std::endl;
}

void testTemplateSpecialization()
{
    std::cout << "Testing template specialization..." << std::endl;
    
    // Test with double precision
    AudioBufferStorage<double, 128, 2> doubleStorage;
    auto& doubleBuffer = doubleStorage.getBuffer();
    
    assert(validateSingleBuffer(doubleBuffer) && "Should work with double precision");
    std::cout << "✓ Double precision test passed!" << std::endl;

    // Test with different sizes
    AudioBufferStorage<float, 1024, 8> largeStorage;
    auto& largeBuffer = largeStorage.getBuffer();
    
    assert(validateSingleBuffer(largeBuffer) && "Should work with large buffers");
    std::cout << "✓ Large buffer test passed!" << std::endl;

    // Test mono buffer
    AudioBufferStorage<float, 512, 1> monoStorage;
    auto& monoBuffer = monoStorage.getBuffer();
    
    assert(validateSingleBuffer(monoBuffer) && "Should work with mono buffers");
    std::cout << "✓ Mono buffer test passed!" << std::endl;
}

void testPerformanceCharacteristics()
{
    std::cout << "Testing performance characteristics..." << std::endl;
    
    // Test with larger buffers to ensure it scales
    constexpr size_t LargeBlockSize = 4096;
    constexpr size_t ManyChannels = 8;
    
    AudioBufferStorage<float, LargeBlockSize, ManyChannels> storageA;
    AudioBufferStorage<float, LargeBlockSize, ManyChannels> storageB;
    
    auto& bufferA = storageA.getBuffer();
    auto& bufferB = storageB.getBuffer();

    // Fill with identical data
    for (size_t ch = 0; ch < ManyChannels; ++ch)
    {
        for (size_t i = 0; i < LargeBlockSize; ++i)
        {
            float value = static_cast<float>(std::sin(i * 0.001));
            bufferA[ch][i] = value;
            bufferB[ch][i] = value;
        }
    }

    // This should complete quickly due to optimized implementation
    assert(compareAudioBuffersExact(bufferA, bufferB) && "Should handle large buffers efficiently");
    assert(validateFiniteValues(bufferA, false) && "Should validate large buffers efficiently");
    
    std::cout << "✓ Large buffer performance test passed!" << std::endl;
}

int main()
{
    std::cout << "Running AudioBuffer Validator Tests..." << std::endl;
    std::cout << "=====================================" << std::endl;
    
    testBasicBufferValidation();
    testBufferComparison();
    testDifferentBufferSizes();
    testFiniteValueValidation();
    testTemplateSpecialization();
    testPerformanceCharacteristics();
    
    std::cout << "=====================================" << std::endl;
    std::cout << "All tests passed! ✓" << std::endl;
    
    return 0;
}
