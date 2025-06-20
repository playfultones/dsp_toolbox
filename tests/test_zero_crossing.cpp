#include "../analysis/zerocrossing.h"
#include <cassert>
#include <cmath>
#include <cstdio>

using namespace PlayfulTones::DspToolBox;

// Helper function to create a sine wave
void generateSineWave (float* buffer, int numSamples, float frequency, float sampleRate)
{
    for (int i = 0; i < numSamples; i++)
    {
        buffer[i] = std::sin (2.0f * M_PI * frequency * i / sampleRate);
    }
}

// Test basic zero crossing detection
void testBasicZeroCrossing()
{
    const int numFrames = 5;
    float data[] = { 1.0f, -1.0f, 1.0f, -1.0f, 1.0f }; // 4 zero crossings
    float* buffer[1] = { data };

    int crossings = countZeroCrossings (buffer, 1, numFrames, 0.0f);
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
    int crossings = countZeroCrossings (buffer, 1, numFrames, 0.0f);

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
    int crossings = countZeroCrossings (buffer, 1, numFrames, hysteresis);

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

    int crossings = countZeroCrossings (buffer, numChannels, numFrames, 0.0f);
    assert (crossings == 6 && "Multi-channel zero crossing test failed"); // Total should be 6
}

int main()
{
    testBasicZeroCrossing();
    testSineWaveZeroCrossing();
    testFrequencyDetection();
    testMultiChannelZeroCrossing();

    printf ("All zero crossing tests passed!\n");
    return 0;
}
