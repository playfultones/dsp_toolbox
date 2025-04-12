#pragma once
#include "../core/processor.h"

namespace PlayfulTones::DspToolBox
{
    /**
     * @brief A simple ADSR envelope generator.
     *
     * This class implements a simple ADSR (Attack, Decay, Sustain, Release) envelope generator.
     * It can be used to shape the amplitude of audio signals over time.
     */
    class ADSR : public Processor
    {
    public:
        ADSR() = default;

        ~ADSR() = default;

        void prepare (double sampleRate, int /*maxFramesPerBlock*/) override
        {
            setSampleRate (sampleRate);
            reset();
        }

        void reset() override
        {
            currentValue = 0.0;
            currentState = State::Idle;
            sampleCounter = 0;
        }

        void setAttack (double attackTimeInSeconds)
        {
            attackSamples = attackTimeInSeconds * getSampleRate();
        }

        void setDecay (double decayTimeInSeconds)
        {
            decaySamples = decayTimeInSeconds * getSampleRate();
        }

        void setSustain (double sustainLevel)
        {
            sustainValue = sustainLevel;
        }

        void setRelease (double releaseTimeInSeconds)
        {
            releaseSamples = releaseTimeInSeconds * getSampleRate();
        }

        void noteOn()
        {
            currentState = State::Attack;
            sampleCounter = 0;
            releaseStartValue = currentValue;
        }

        void noteOff()
        {
            currentState = State::Release;
            sampleCounter = 0;
            releaseStartValue = currentValue;
        }

        void process (float** buffer, int numChannels, int numFrames) override
        {
            for (int i = 0; i < numFrames; i++)
            {
                double envValue = processNextValue();

                for (int ch = 0; ch < numChannels; ch++)
                {
                    buffer[ch][i] *= static_cast<float> (envValue);
                }
            }
        }

    private:
        enum class State {
            Idle,
            Attack,
            Decay,
            Sustain,
            Release
        };

        double processNextValue()
        {
            switch (currentState)
            {
                case State::Attack:
                    if (sampleCounter >= attackSamples)
                    {
                        currentState = State::Decay;
                        sampleCounter = 0;
                        currentValue = 1.0;
                    }
                    else
                    {
                        currentValue = sampleCounter / attackSamples;
                    }
                    break;

                case State::Decay:
                    if (sampleCounter >= decaySamples)
                    {
                        currentState = State::Sustain;
                        currentValue = sustainValue;
                    }
                    else
                    {
                        double decayProgress = sampleCounter / decaySamples;
                        currentValue = 1.0 - (1.0 - sustainValue) * decayProgress;
                    }
                    break;

                case State::Sustain:
                    currentValue = sustainValue;
                    break;

                case State::Release:
                    if (sampleCounter >= releaseSamples)
                    {
                        currentState = State::Idle;
                        currentValue = 0.0;
                    }
                    else
                    {
                        double releaseProgress = sampleCounter / releaseSamples;
                        currentValue = releaseStartValue * (1.0 - releaseProgress);
                    }
                    break;

                case State::Idle:
                    currentValue = 0.0;
                    break;
            }

            sampleCounter++;
            return currentValue;
        }

        double attackSamples = 0.0;
        double decaySamples = 0.0;
        double sustainValue = 1.0;
        double releaseSamples = 0.0;
        double currentValue = 0.0;
        double releaseStartValue = 0.0;
        double sampleCounter = 0;
        State currentState = State::Idle;
    };
} // namespace PlayfulTones::DspToolBox