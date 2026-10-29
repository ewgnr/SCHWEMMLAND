#pragma once

#include <vector>
#include <cmath>
#include <algorithm>


// ============================================================
// OUTPUT STAGE (SATURATION + ROUTING)
// ============================================================
class OutputStage
{
public:

    // --------------------------------------------------------
    // SETUP ROUTING
    // --------------------------------------------------------
    void setup(const std::vector<int>& speakers, const std::vector<int>& outputs)
    {
        routedSpeakers = speakers;
        outMap = outputs;
    }


    // --------------------------------------------------------
    // PROCESS SINGLE SAMPLE FRAME
    // --------------------------------------------------------
    void processSample(float* frame, int CH, const std::vector<double>& speakerBuffer, double drive, double master)
    {
        const int R = std::min(static_cast<int>(routedSpeakers.size()), static_cast<int>(outMap.size()));

        for (int idx = 0; idx < R; idx++)
        {
            if (idx >= static_cast<int>(outMap.size()))
                continue;

            int spk = routedSpeakers[idx];

            if (spk < 0 || spk >= static_cast<int>(speakerBuffer.size()))
                continue;

            double sOut = speakerBuffer[spk];


            // ------------------------------------------------
            // Soft Saturation
            // ------------------------------------------------
            float processed = static_cast<float>(std::tanh(sOut * drive) * master);


            // ------------------------------------------------
            // Output Routing
            // ------------------------------------------------
            int outCh = outMap[idx];

            if (outCh >= 0 && outCh < CH)
            {
                frame[outCh] += processed;
            }
        }
    }


private:

    std::vector<int> routedSpeakers;
    std::vector<int> outMap;
};