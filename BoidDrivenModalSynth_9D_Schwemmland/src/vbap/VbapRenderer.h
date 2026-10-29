
#pragma once

#include "BoidSpatialSystem.h"
#include "Space.h"
#include "Vbap2.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <cstddef>

class VBAPRenderer
{
public:

    void setup(std::size_t numSpeakers, int maxBoids)
    {
        speakerAz.resize(numSpeakers);

        if (numSpeakers == 1)
        {
            speakerAz[0] = 0.0;
        }
        else if (numSpeakers == 2)
        {
            // Stereo: left and right speakers at +/- 30 degrees.
            // Avoids the singular matrix produced by antipodal speakers.
            speakerAz[0] = -M_PI / 6.0;
            speakerAz[1] = M_PI / 6.0;
        }
        else if (numSpeakers == 4)
        {
            speakerAz[0] = -3.0 * M_PI / 4.0;
            speakerAz[1] = -1.0 * M_PI / 4.0;
            speakerAz[2] = 1.0 * M_PI / 4.0;
            speakerAz[3] = 3.0 * M_PI / 4.0;
        }
        else
        {
            // Circular 2D loudspeaker layout.
            for (std::size_t i = 0; i < numSpeakers; ++i)
            {
                speakerAz[i] = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(numSpeakers);
            }
        }

        vbap.setup(speakerAz);

        preparedGains.assign(std::max(0, maxBoids), std::vector<double>(numSpeakers, 0.0));

        currentGains.assign(std::max(0, maxBoids), std::vector<double>(numSpeakers, 0.0));

        tempGains.resize(numSpeakers);
    }

    // Prepare spatial gains once per audio block.
    void prepareParams(const std::vector<SpatialParams>& params, int N)
    {
        const int count = std::min({
            N,
            static_cast<int>(params.size()),
            static_cast<int>(preparedGains.size())
            });

        for (int j = 0; j < count; ++j)
        {
            const auto& p = params[j];
            auto& out = preparedGains[j];

            std::fill(out.begin(), out.end(), 0.0);

            const int steps = std::max(1, p.spreadSteps);

            for (int s = 0; s < steps; ++s)
            {
                const double t = (steps == 1) ? 0.0 : 2.0 * static_cast<double>(s) / static_cast<double>(steps - 1) - 1.0;

                const double azOffset = p.az + t * (0.25 * p.spread);

                vbap.process(azOffset, tempGains);

                const double weight = 1.0 / steps;

                for (std::size_t spk = 0; spk < out.size(); ++spk)
                {
                    out[spk] += tempGains[spk] * weight;
                }
            }
        }
    }

    // Same rendering interface as AmbisonicRenderer.
    void renderSpeakers(
        std::vector<double>& speakerBuffer,
        const std::vector<double>& boidOutputs,
        int N,
        const std::vector<SpatialParams>& params)
    {
        std::fill(speakerBuffer.begin(), speakerBuffer.end(), 0.0);

        const int count = std::min({
            N,
            static_cast<int>(boidOutputs.size()),
            static_cast<int>(params.size()),
            static_cast<int>(preparedGains.size())
            });

        const std::size_t speakers = std::min(speakerBuffer.size(), speakerAz.size());

        for (int j = 0; j < count; ++j)
        {
            const double audio = boidOutputs[j] * params[j].distGain;

            auto& current = currentGains[j];
            const auto& target = preparedGains[j];

            for (std::size_t spk = 0; spk < speakers; ++spk)
            {
                current[spk] += smoothingAlpha * (target[spk] - current[spk]);
                speakerBuffer[spk] += audio * current[spk];
            }
        }
    }

private:

    VBAP2 vbap;

    std::vector<double> speakerAz;
    std::vector<std::vector<double>> preparedGains;
    std::vector<std::vector<double>> currentGains;
    std::vector<double> tempGains;

    double smoothingAlpha = 0.0;

public:
    void setSmoothing(double sampleRate, double timeMs = 10.0)
    {
        const double tau = std::max(0.001, timeMs * 0.001);

        smoothingAlpha = 1.0 - std::exp(-1.0 / (sampleRate * tau));
    }
};
