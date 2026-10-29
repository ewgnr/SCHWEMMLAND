#pragma once

#include <vector>
#include "ModalFilter.h"
#include "ofMain.h"
#include "PinkNoiseGenerator.h"
#include "LinearEnvelope.h"
#include "ParamRegistry.h"

struct EnvVoice
{
    LinearEnvelope env;
};

class ModalBank2D
{
public:
    // Setup with N sources (boids) and M modes per source
    void setup(size_t numSources, size_t numModesPerSource, double sR)
    {
        sampleRate = sR;

        sources = numSources;
        modesPerSource = numModesPerSource;

        nodes.resize(numSources);
        weights.resize(numSources);
        excitation.resize(numSources);
        output.resize(numSources);
        noiseState.assign(numSources, 0.0);
        envEnergy.assign(numSources, 0.0);
        noise.resize(numSources);
        voices.resize(numSources);

        for (auto& n : noise)
            n.setAmplitude(1.0);

        for (size_t i = 0; i < numSources; i++)
        {
            nodes[i].resize(numModesPerSource);
            weights[i].assign(numModesPerSource, 1.0);
            excitation[i].assign(numModesPerSource, 0.0);
            output[i].assign(numModesPerSource, 0.0);

            for (auto& n : nodes[i])
                n.setSampleRate(sampleRate);
        }
    }

    // Initialize all modes with random parameters
    void initRandom(double baseFreq = 200.0, double freqRange = 800.0)
    {
        for (size_t i = 0; i < nodes.size(); i++)
        {
            for (auto& n : nodes[i])
            {
                double frq = baseFreq + freqRange * ofRandom(1.0);
                double q = 5.0 + 10.0 * ofRandom(1.0);
                n.init(frq, q, 0.2);
            }
        }
    }

    void setParams(size_t sourceIdx, size_t modeIdx, double frq, double q, double amp)
    {
        if (sourceIdx < nodes.size() && modeIdx < nodes[sourceIdx].size())
            nodes[sourceIdx][modeIdx].setParams(frq, q, amp);
    }

    void excite(size_t sourceIdx, size_t modeIdx, double value)
    {
        if (sourceIdx < excitation.size() && modeIdx < excitation[sourceIdx].size())
            excitation[sourceIdx][modeIdx] += value;
    }

    inline void trigger(size_t i)
    {
        if (i >= voices.size()) return;

        constexpr std::size_t maxVoices = 1;

        if (voices[i].size() >= maxVoices) return;

        const double attack = ENV_ATTACK_MS.load(std::memory_order_relaxed) / 1000.0;
        const double decay = ENV_DECAY_MS.load(std::memory_order_relaxed) / 1000.0;
        const double sustain = std::clamp(ENV_SUSTAIN_LEVEL.load(std::memory_order_relaxed), 0.0, 1.0);
        const double sustainTime = ENV_SUSTAIN_MS.load(std::memory_order_relaxed) / 1000.0;
        const double release = ENV_RELEASE_MS.load(std::memory_order_relaxed) / 1000.0;

        const std::vector<std::pair<double, double>> shape = {
            {attack, 1.0},
            {decay, sustain},
            {sustainTime, sustain},
            {release, 0.0}
        };

        EnvVoice v;
        v.env.set(shape);
        v.env.reset();

        voices[i].push_back(std::move(v));
    }


    void playMulti(std::vector<double>& sums)
    {
        const size_t numSources = nodes.size();
        const double dt = 1.0 / sampleRate;

        for (size_t i = 0; i < numSources; ++i)
        {
            auto& voices_i = voices[i];
            double env = 0.0;

            for (std::size_t k = 0; k < voices_i.size(); ++k)
            {
                env += voices_i[k].env.process(dt);
            }

            for (std::size_t k = voices_i.size(); k-- > 0; )
            {
                if (!voices_i[k].env.isActive())
                {
                    voices_i.erase(voices_i.begin() + k);
                }
            }

            const double noiseSample = noise[i].process();
            const double excitationSignal = env * noiseSample;

            double sum = 0.0;

            auto& modes = nodes[i];
            auto& w = weights[i];

            for (size_t j = 0; j < modes.size(); ++j)
            {
                const double modeScale = 1.0 / (1.0 + 0.35 * j * j);
                const double input = excitationSignal * modeScale;

                sum += modes[j].process(input) * w[j];
            }

            sums[i] = sum;
        }
    }

    std::size_t getNumModes(size_t sourceIdx) const
    {
        if (sourceIdx >= nodes.size()) return 0;
        return nodes[sourceIdx].size();
    }

    const std::vector<std::vector<double>>& getOutputs() const
    {
        return output;
    }

private:
    double sampleRate = 48000.0;
    std::size_t sources = 0;
    std::size_t modesPerSource = 0;

    std::vector<std::vector<ModalFilter>> nodes;
    std::vector<std::vector<double>> weights;
    std::vector<std::vector<double>> excitation;
    std::vector<std::vector<double>> output;

    std::vector<std::vector<EnvVoice>> voices;
    std::vector<PinkNoiseGenerator> noise;
    std::vector<double> noiseState;
    std::vector<double> envEnergy;
};