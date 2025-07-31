/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#include "analysis/zerocrossing.h"
#include "core/constants.h"
#include "helpers/compilationhelpers.h"
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <span>

using namespace PlayfulTones::DspToolBox;

// Helper function to create a sine wave in a fixed-size buffer
template <size_t Size>
void generateSineWave (std::array<float, Size>& buffer, float frequency, float sampleRate) noexcept
{
    for (size_t i = 0; i < Size; ++i)
    {
        buffer[i] = std::sin (Constants::twoPi * frequency * static_cast<float> (i) / sampleRate);
    }
}

// Test basic zero crossing detection with compile-time sized buffers
void testBasicZeroCrossing()
{
    constexpr size_t BlockSize = 5;
    std::array<float, BlockSize> data = { 1.0f, -1.0f, 1.0f, -1.0f, 1.0f }; // 4 zero crossings
    std::span<const float, BlockSize> buffer { data };

    const size_t crossings = ZeroCrossingAnalyzer::countZeroCrossings (buffer);
    constexpr size_t expectedCrossings = 4; // We expect 4 zero crossings in our test data
    assert (crossings == expectedCrossings && "Basic zero crossing test failed");
    markUsed (crossings, expectedCrossings);
}

// Test zero crossing with a sine wave
void testSineWaveZeroCrossing()
{
    constexpr size_t BlockSize = 1000;
    constexpr float sampleRate = 44100.0f;
    constexpr float frequency = 440.0f; // A4 note

    std::array<float, BlockSize> sineWave;
    generateSineWave (sineWave, frequency, sampleRate);

    std::span<const float, BlockSize> buffer { sineWave };
    const size_t crossings = ZeroCrossingAnalyzer::countZeroCrossings (buffer);

    // For a sine wave, we expect approximately 2 * frequency * (BlockSize/sampleRate) zero crossings
    const float expectedCrossings = 2.0f * frequency * (static_cast<float> (BlockSize) / sampleRate);
    constexpr float tolerance = 2.0f; // Allow for some numerical imprecision

    const float difference = std::abs (static_cast<float> (crossings) - expectedCrossings);
    assert (difference <= tolerance && "Sine wave zero crossing count is not within expected range");

    markUsed (difference, tolerance);
}

// Test frequency detection using zero crossings
void testFrequencyDetection()
{
    constexpr size_t BlockSize = 44100; // 1 second at 44.1kHz for better accuracy
    constexpr float sampleRate = 44100.0f;
    constexpr float testFrequency = 440.0f; // A4 note
    constexpr float hysteresis = 0.0001f; // Very small hysteresis for clean sine wave

    std::array<float, BlockSize> sineWave;
    generateSineWave (sineWave, testFrequency, sampleRate);

    std::span<const float, BlockSize> buffer { sineWave };
    const size_t crossings = ZeroCrossingAnalyzer::countZeroCrossings (buffer, ZeroCrossingDirection::All, hysteresis);

    // Calculate frequency from zero crossings using the built-in method
    const float zcr = static_cast<float> (crossings) / static_cast<float> (BlockSize);
    const float detectedFrequency = ZeroCrossingAnalyzer::estimateFrequencyFromZCR (zcr, sampleRate);

    // Debug output
    printf ("Expected frequency: %.3f Hz\n", testFrequency);
    printf ("Detected frequency: %.3f Hz\n", detectedFrequency);
    printf ("Number of crossings: %zu\n", crossings);
    printf ("Expected crossings: %.1f\n", 2.0f * testFrequency); // For 1 second of audio

    constexpr float tolerance = 0.51f; // Allow for floating point precision issues
    const float diff = std::abs (detectedFrequency - testFrequency);
    printf ("Frequency difference: %.3f Hz (tolerance: %.3f Hz)\n", diff, tolerance);
    assert (diff <= tolerance && "Frequency detection is not within expected range");
}

// Test multi-channel zero crossing detection using a simple multi-channel buffer
void testMultiChannelZeroCrossing()
{
    constexpr size_t BlockSize = 5;
    constexpr size_t NumChannels = 2;

    std::array<float, BlockSize> channel1 = { 1.0f, -1.0f, 1.0f, -1.0f, 1.0f }; // 4 crossings
    std::array<float, BlockSize> channel2 = { 1.0f, 1.0f, -1.0f, -1.0f, 1.0f }; // 2 crossings

    std::array<std::span<const float, BlockSize>, NumChannels> buffer = {
        std::span<const float, BlockSize> { channel1 },
        std::span<const float, BlockSize> { channel2 }
    };

    const size_t crossings = ZeroCrossingAnalyzer::countZeroCrossings (buffer);
    constexpr size_t expectedCrossings = 6; // Total should be 6

    markUsed (crossings);
    assert (crossings == expectedCrossings && "Multi-channel zero crossing test failed");
}

// Test zero crossing rate calculation
void testZeroCrossingRate()
{
    // Test with basic signal
    {
        constexpr size_t BlockSize = 5;
        std::array<float, BlockSize> data = { 1.0f, -1.0f, 1.0f, -1.0f, 1.0f }; // 4 zero crossings
        std::span<const float, BlockSize> buffer { data };

        const float zcr = ZeroCrossingAnalyzer::calculateZeroCrossingRate (buffer);
        constexpr float expectedZCR = 4.0f / 5.0f; // 4 crossings / 5 samples
        markUsed (zcr, expectedZCR);
        assert (std::abs (zcr - expectedZCR) < 0.0001f && "Basic ZCR test failed");
    }

    // Test with sine wave
    {
        constexpr size_t BlockSize = 44100; // 1 second at 44.1kHz
        constexpr float sampleRate = 44100.0f;
        constexpr float frequency = 440.0f; // A4 note

        std::array<float, BlockSize> sineWave;
        generateSineWave (sineWave, frequency, sampleRate);

        std::span<const float, BlockSize> buffer { sineWave };
        const float zcr = ZeroCrossingAnalyzer::calculateZeroCrossingRate (buffer);

        // Expected ZCR for a sine wave is 2 * frequency / sampleRate
        const float expectedZCR = 2.0f * frequency / sampleRate;
        constexpr float tolerance = 0.0001f;

        markUsed (tolerance);
        printf ("Expected ZCR: %.6f\n", expectedZCR);
        printf ("Measured ZCR: %.6f\n", zcr);
        assert (std::abs (zcr - expectedZCR) <= tolerance && "Sine wave ZCR test failed");
    }

    // Test multi-channel ZCR
    {
        constexpr size_t BlockSize = 5;
        constexpr size_t NumChannels = 2;

        std::array<float, BlockSize> channel1 = { 1.0f, -1.0f, 1.0f, -1.0f, 1.0f }; // 4 crossings
        std::array<float, BlockSize> channel2 = { 1.0f, 1.0f, -1.0f, -1.0f, 1.0f }; // 2 crossings

        std::array<std::span<const float, BlockSize>, NumChannels> buffer = {
            std::span<const float, BlockSize> { channel1 },
            std::span<const float, BlockSize> { channel2 }
        };

        const float zcr = ZeroCrossingAnalyzer::calculateZeroCrossingRate (buffer);
        constexpr float expectedZCR = 6.0f / (BlockSize * NumChannels); // 6 total crossings / 10 total samples

        markUsed (zcr, expectedZCR);
        assert (std::abs (zcr - expectedZCR) < 0.0001f && "Multi-channel ZCR test failed");
    }
}

// Test directional zero crossing detection
void testDirectionalZeroCrossing()
{
    constexpr size_t BlockSize = 5;
    std::array<float, BlockSize> data = { 1.0f, -1.0f, 1.0f, -1.0f, 1.0f }; // 2 positive and 2 negative crossings
    std::span<const float, BlockSize> buffer { data };

    // Test positive-going crossings
    const size_t positiveCrossings = ZeroCrossingAnalyzer::countZeroCrossings (buffer, ZeroCrossingDirection::Positive, 0.0f);
    assert (positiveCrossings == 2 && "Positive-going zero crossing count incorrect");

    // Test negative-going crossings
    const size_t negativeCrossings = ZeroCrossingAnalyzer::countZeroCrossings (buffer, ZeroCrossingDirection::Negative, 0.0f);
    assert (negativeCrossings == 2 && "Negative-going zero crossing count incorrect");

    // Verify that total equals sum of positive and negative
    const size_t totalCrossings = ZeroCrossingAnalyzer::countZeroCrossings (buffer, ZeroCrossingDirection::All, 0.0f);
    assert (totalCrossings == positiveCrossings + negativeCrossings && "Total crossings should equal sum of directional crossings");

    markUsed (positiveCrossings, negativeCrossings, totalCrossings);

    // Test with sine wave
    constexpr float sampleRate = 44100.0f;
    constexpr float frequency = 440.0f;
    constexpr size_t SineBlockSize = 1000;

    std::array<float, SineBlockSize> sineWave;
    generateSineWave (sineWave, frequency, sampleRate);
    std::span<const float, SineBlockSize> sineBuffer { sineWave };

    const size_t sinePositive = ZeroCrossingAnalyzer::countZeroCrossings (sineBuffer, ZeroCrossingDirection::Positive, 0.0f);
    const size_t sineNegative = ZeroCrossingAnalyzer::countZeroCrossings (sineBuffer, ZeroCrossingDirection::Negative, 0.0f);
    const size_t sineTotal = ZeroCrossingAnalyzer::countZeroCrossings (sineBuffer, ZeroCrossingDirection::All, 0.0f);

    markUsed (sinePositive, sineNegative, sineTotal);

    // For a sine wave, positive and negative crossings should be approximately equal
    assert (std::abs (static_cast<int> (sinePositive) - static_cast<int> (sineNegative)) <= 1 && "Positive and negative crossings should be equal for sine wave");
    assert (sineTotal == sinePositive + sineNegative && "Total crossings should equal sum of directional crossings for sine wave");

    // Test zero crossing rate with directional counting
    const float zcrPositive = ZeroCrossingAnalyzer::calculateZeroCrossingRate (sineBuffer, ZeroCrossingDirection::Positive, 0.0f);
    const float zcrNegative = ZeroCrossingAnalyzer::calculateZeroCrossingRate (sineBuffer, ZeroCrossingDirection::Negative, 0.0f);
    const float zcrTotal = ZeroCrossingAnalyzer::calculateZeroCrossingRate (sineBuffer, ZeroCrossingDirection::All, 0.0f);

    // The ZCR for positive or negative only should be approximately half of the total ZCR
    // For frequency detection, we allow 0.5 Hz difference, which means about 1 crossing difference per second
    // This translates to a ZCR difference of about (1.0 / SineBlockSize) for our buffer length
    const float toleranceZCR = 2.0f / static_cast<float> (SineBlockSize); // Allow 2 crossings difference

    // Verify that positive + negative = total (this should be exact)
    const float sumDiff = std::abs (zcrTotal - (zcrPositive + zcrNegative));
    printf ("ZCR Sum difference: %.9f\n", sumDiff);
    assert (sumDiff < 0.000001f && "Total ZCR should equal sum of directional ZCRs");

    // Verify that positive ≈ negative (allowing for quantization effects)
    const float zcrDiff = std::abs (zcrPositive - zcrNegative);
    printf ("ZCR Difference: %.9f (tolerance: %.9f)\n", zcrDiff, toleranceZCR);
    assert (zcrDiff < toleranceZCR && "Positive and negative ZCRs should be approximately equal for sine wave");
}

int main()
{
    testBasicZeroCrossing();
    testSineWaveZeroCrossing();
    testFrequencyDetection();
    testMultiChannelZeroCrossing();
    testZeroCrossingRate();
    testDirectionalZeroCrossing();

    printf ("All zero crossing tests passed!\n");
    return 0;
}
