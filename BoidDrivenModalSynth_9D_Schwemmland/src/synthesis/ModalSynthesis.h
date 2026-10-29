#pragma once

#include "ModalBank2D.h"
#include "ModalParams.h"
#include <vector>
#include <atomic>

class ModalSynthesis
{
public:

    void setup(int maxBoids, double samplerate)
    {
        Nmax = maxBoids;

        modalBank.setup(Nmax, 8, samplerate);
        modalBank.initRandom();

        boidOutputs.resize(Nmax);
    }

    // ----------------------------------------------------
    // PARAM UPDATE (CONTROL RATE)
    // ----------------------------------------------------
    void applyParams(const ModalParamsBuffer* params, int N)
    {
        if (!params || N <= 0)
            return;

        const int step = 2;

        for (int j = 0; j < N; j += step)
        {
            for (int n = 0; n < step && (j + n) < N; n++)
            {
                int idx = j + n;

                for (size_t m = 0; m < params->perBoid[idx].freq.size(); m++)
                {
                    double freq = params->perBoid[idx].freq[m];
                    double bw = params->perBoid[idx].bw[m];
                    double amp = params->perBoid[idx].amp[m];

                    modalBank.setParams(idx, m, freq, bw, amp);
                }
            }
        }
    }

    // ----------------------------------------------------
    // AUDIO RATE RENDER
    // ----------------------------------------------------
    void process()
    {
        modalBank.playMulti(boidOutputs);
    }

    // ----------------------------------------------------
    // TRIGGER ENVELOPE
    // ----------------------------------------------------
    void updateTriggers(int N)
    {
        modalBank.trigger(N);
    }

    // ----------------------------------------------------
    // OUTPUT ACCESS
    // ----------------------------------------------------
    std::vector<double>& getOutputs()
    {
        return boidOutputs;
    }

private:

    ModalBank2D modalBank;

    std::vector<double> boidOutputs;

    int Nmax = 0;
    int updateOffset = 0;
};