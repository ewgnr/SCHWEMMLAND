#pragma once

#include "ambiEncode2DThirdOrder.h"
#include "ambiDecode2DThirdOrder.h"
#include "BoidSpatialSystem.h"
#include "Space.h"

#include <array>
#include <vector>
#include <glm/glm.hpp>
#include <algorithm>
#include <cmath>

// ============================================================
// AMBISONIC RENDERER (ENC + DEC ONLY)
// ============================================================
class AmbisonicRenderer
{
public:

    void setup(std::size_t numSpeakers, int maxBoids)
    {
        speakerAz.resize(numSpeakers);

        for (std::size_t i = 0; i < numSpeakers; ++i)
        {
            speakerAz[i] = 2.0 * M_PI * double(i) / double(numSpeakers);
        }
    }

    void prepareParams(
        std::vector<SpatialParams>& params,
        int N)
    {
        const int count = std::min(N, static_cast<int>(params.size()));

        for (int j = 0; j < count; ++j)
        {
            auto& p = params[j];
            p.coeff.fill(0.0);

            const int steps = std::max(1, p.spreadSteps);

            for (int s = 0; s < steps; ++s)
            {
                const double t = (steps == 1) ? 0.0 : 2.0 * static_cast<double>(s) / static_cast<double>(steps - 1) - 1.0;

                const double azOffset = p.az + t * (0.25 * p.spread);

                const auto encoded = ambiEnc.process(1.0, azOffset, p.dist);

                const double weight = 1.0 / steps;

                for (int k = 0; k < 7; ++k)
                    p.coeff[k] += encoded[k] * weight;
            }
        }
    }

    void renderSpeakers(
        std::vector<double>& speakerBuffer,
        const std::vector<double>& boidOutputs,
        int N,
        const std::vector<SpatialParams>& params)
    {

        const int count = std::min({
            N,
            static_cast<int>(boidOutputs.size()),
            static_cast<int>(params.size())
        });

        std::fill(speakerBuffer.begin(), speakerBuffer.end(), 0.0);

        // ------------------------------------------------
        // Position Encoding (7 is hardcoded for 3rd Order!)
        // ------------------------------------------------        
        std::array<double, 7> ambiFrame{};
        
        for (int j = 0; j < count; ++j)
        {
            const double audio = boidOutputs[j] * params[j].distGain;

            for (int k = 0; k < 7; ++k)
            {
                ambiFrame[k] += audio * params[j].coeff[k];
            }
        }

        for (std::size_t spk = 0; spk < speakerAz.size(); ++spk)
        {
            speakerBuffer[spk] = ambiDec.process(ambiFrame, speakerAz[spk]);
        }
    }


private:

    AmbiEncode2D ambiEnc;
    AmbiDecode2D ambiDec;

    std::vector<double> speakerAz;
};