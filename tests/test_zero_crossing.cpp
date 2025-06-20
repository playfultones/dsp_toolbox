/*******************************************************************
* Copyright         : 2025 Playful Tones
* Author            : Bence Kovács
* License           : GNU General Public License v3.0
*******************************************************************/

#include "analysis/zerocrossing.h"
#include "core/constants.h"
#include <cassert>
#include <cmath>
#include <cstdio>

using namespace PlayfulTones::DspToolBox;

// Helper function to create a sine wave
void generateSineWave (float* buffer, int numSamples, float frequency, float sampleRate)
{
    for (int i = 0; i < numSamples; i++)
    {
        buffer[i] = std::sin (Constants::twoPi * frequency * i / sampleRate);
    }
}

// Test basic zero crossing detection
void testBasicZeroCrossing()
{
    const int numFrames = 5;
    float data[] = { 1.0f, -1.0f, 1.0f, -1.0f, 1.0f }; // 4 zero crossings
    float* buffer[1] = { data };

    int crossings = countZeroCrossings (buffer, 1, numFrames);
    assert (crossings == 4 && "Basic zero crossing test failed");
}

// Test zero crossing with a sine wave
void testSineWaveZeroCrossing()
{
    const int numFrames = 1000;
    const float sampleRate = 44100.0f;
    const float frequency = 440.0f; // A4 note

    float* sineWave = new float[numFrames];
    generateSineWave (sineWave, numFrames, frequency, sampleRate);

    float* buffer[1] = { sineWave };
    int crossings = countZeroCrossings (buffer, 1, numFrames);

    // For a sine wave, we expect approximately 2 * frequency * (numFrames/sampleRate) zero crossings
    float expectedCrossings = 2.0f * frequency * (numFrames / sampleRate);
    float tolerance = 2.0f; // Allow for some numerical imprecision

    assert (std::abs (crossings - expectedCrossings) <= tolerance && "Sine wave zero crossing count is not within expected range");

    delete[] sineWave;
}

// Test frequency detection using zero crossings
void testFrequencyDetection()
{
    const int numFrames = 44100; // 1 second at 44.1kHz for better accuracy
    const float sampleRate = 44100.0f;
    const float testFrequency = 440.0f; // A4 note
    const float hysteresis = 0.0001f; // Very small hysteresis for clean sine wave

    float* sineWave = new float[numFrames];
    generateSineWave (sineWave, numFrames, testFrequency, sampleRate);

    float* buffer[1] = { sineWave };
    int crossings = countZeroCrossings (buffer, 1, numFrames, ZeroCrossingDirection::All, hysteresis);

    // Calculate frequency from zero crossings
    // frequency = (number of zero crossings * sampleRate) / (2 * numFrames)
    float detectedFrequency = (crossings * sampleRate) / (2.0f * numFrames);

    // Debug output
    printf ("Expected frequency: %.3f Hz\n", testFrequency);
    printf ("Detected frequency: %.3f Hz\n", detectedFrequency);
    printf ("Number of crossings: %d\n", crossings);
    printf ("Expected crossings: %.1f\n", 2.0f * testFrequency); // For 1 second of audio

    float tolerance = 0.5f; // Tighter tolerance with improved accuracy
    float diff = std::abs (detectedFrequency - testFrequency);
    printf ("Frequency difference: %.3f Hz (tolerance: %.3f Hz)\n", diff, tolerance);
    assert (diff <= tolerance && "Frequency detection is not within expected range");

    delete[] sineWave;
}

// Test multi-channel zero crossing detection
void testMultiChannelZeroCrossing()
{
    const int numFrames = 5;
    const int numChannels = 2;

    float channel1[] = { 1.0f, -1.0f, 1.0f, -1.0f, 1.0f }; // 4 crossings
    float channel2[] = { 1.0f, 1.0f, -1.0f, -1.0f, 1.0f }; // 2 crossings
    float* buffer[2] = { channel1, channel2 };

    int crossings = countZeroCrossings (buffer, numChannels, numFrames);
    assert (crossings == 6 && "Multi-channel zero crossing test failed"); // Total should be 6
}

// Test zero crossing rate calculation
void testZeroCrossingRate()
{
    // Test with basic signal
    {
        const int numFrames = 5;
        float data[] = { 1.0f, -1.0f, 1.0f, -1.0f, 1.0f }; // 4 zero crossings
        float* buffer[1] = { data };

        float zcr = calculateZeroCrossingRate (buffer, 1, numFrames);
        float expectedZCR = 4.0f / 5.0f; // 4 crossings / 5 samples
        assert (std::abs (zcr - expectedZCR) < 0.0001f && "Basic ZCR test failed");
    }

    // Test with sine wave
    {
        const int numFrames = 44100; // 1 second at 44.1kHz
        const float sampleRate = 44100.0f;
        const float frequency = 440.0f; // A4 note

        float* sineWave = new float[numFrames];
        generateSineWave (sineWave, numFrames, frequency, sampleRate);

        float* buffer[1] = { sineWave };
        float zcr = calculateZeroCrossingRate (buffer, 1, numFrames);

        // Expected ZCR for a sine wave is 2 * frequency / sampleRate
        float expectedZCR = 2.0f * frequency / sampleRate;
        float tolerance = 0.0001f;

        printf ("Expected ZCR: %.6f\n", expectedZCR);
        printf ("Measured ZCR: %.6f\n", zcr);
        assert (std::abs (zcr - expectedZCR) <= tolerance && "Sine wave ZCR test failed");

        delete[] sineWave;
    }

    // Test multi-channel ZCR
    {
        const int numFrames = 5;
        const int numChannels = 2;

        float channel1[] = { 1.0f, -1.0f, 1.0f, -1.0f, 1.0f }; // 4 crossings
        float channel2[] = { 1.0f, 1.0f, -1.0f, -1.0f, 1.0f }; // 2 crossings
        float* buffer[2] = { channel1, channel2 };

        float zcr = calculateZeroCrossingRate (buffer, numChannels, numFrames);
        float expectedZCR = 6.0f / (numFrames * numChannels); // 6 total crossings / 10 total samples
        assert (std::abs (zcr - expectedZCR) < 0.0001f && "Multi-channel ZCR test failed");
    }
}

// Test directional zero crossing detection
void testDirectionalZeroCrossing()
{
    const int numFrames = 5;
    float data[] = { 1.0f, -1.0f, 1.0f, -1.0f, 1.0f }; // 2 positive and 2 negative crossings
    float* buffer[1] = { data };

    // Test positive-going crossings
    int positiveCrossings = countZeroCrossings (buffer, 1, numFrames, ZeroCrossingDirection::Positive, 0.0f);
    assert (positiveCrossings == 2 && "Positive-going zero crossing count incorrect");

    // Test negative-going crossings
    int negativeCrossings = countZeroCrossings (buffer, 1, numFrames, ZeroCrossingDirection::Negative, 0.0f);
    assert (negativeCrossings == 2 && "Negative-going zero crossing count incorrect");

    // Verify that total equals sum of positive and negative
    int totalCrossings = countZeroCrossings (buffer, 1, numFrames, ZeroCrossingDirection::All, 0.0f);
    assert (totalCrossings == positiveCrossings + negativeCrossings && "Total crossings should equal sum of directional crossings");

    // Test with sine wave
    const float sampleRate = 44100.0f;
    const float frequency = 440.0f;
    const int sineFrames = 1000;

    float* sineWave = new float[sineFrames];
    generateSineWave (sineWave, sineFrames, frequency, sampleRate);
    float* sineBuffer[1] = { sineWave };

    int sinePositive = countZeroCrossings (sineBuffer, 1, sineFrames, ZeroCrossingDirection::Positive, 0.0f);
    int sineNegative = countZeroCrossings (sineBuffer, 1, sineFrames, ZeroCrossingDirection::Negative, 0.0f);
    int sineTotal = countZeroCrossings (sineBuffer, 1, sineFrames, ZeroCrossingDirection::All, 0.0f);

    // For a sine wave, positive and negative crossings should be approximately equal
    assert (std::abs (sinePositive - sineNegative) <= 1 && "Positive and negative crossings should be equal for sine wave");
    assert (sineTotal == sinePositive + sineNegative && "Total crossings should equal sum of directional crossings for sine wave");

    // Test zero crossing rate with directional counting
    float zcrPositive = calculateZeroCrossingRate (sineBuffer, 1, sineFrames, ZeroCrossingDirection::Positive, 0.0f);
    float zcrNegative = calculateZeroCrossingRate (sineBuffer, 1, sineFrames, ZeroCrossingDirection::Negative, 0.0f);
    float zcrTotal = calculateZeroCrossingRate (sineBuffer, 1, sineFrames, ZeroCrossingDirection::All, 0.0f);

    // The ZCR for positive or negative only should be approximately half of the total ZCR
    // For frequency detection, we allow 0.5 Hz difference, which means about 1 crossing difference per second
    // This translates to a ZCR difference of about (1.0 / sineFrames) for our buffer length
    float toleranceZCR = 2.0f / static_cast<float> (sineFrames); // Allow 2 crossings difference

    // Verify that positive + negative = total (this should be exact)
    float sumDiff = std::abs (zcrTotal - (zcrPositive + zcrNegative));
    printf ("ZCR Sum difference: %.9f\n", sumDiff);
    assert (sumDiff < 0.000001f && "Total ZCR should equal sum of directional ZCRs");

    // Verify that positive ≈ negative (allowing for quantization effects)
    float zcrDiff = std::abs (zcrPositive - zcrNegative);
    printf ("ZCR Difference: %.9f (tolerance: %.9f)\n", zcrDiff, toleranceZCR);
    assert (zcrDiff < toleranceZCR && "Positive and negative ZCRs should be approximately equal for sine wave");

    delete[] sineWave;
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
