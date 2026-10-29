#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <iostream>

#include "ParamRegistry.h"
#include "ModalParams.h"


class TriggerEngine
{
public:

    // =========================================================
    // SETUP
    // =========================================================
    void setup(int maxBoids, double sr)
    {
        Nmax = maxBoids;
        sampleRate = sr;

        rate.resize(Nmax, 10.0);
        smoothedRate.resize(Nmax, 10.0);
    }

    // =========================================================
    // PHASE 1 — CONTINUOUS RATE FIELD
    // Input: ControlMapping excitationField (0..1)
    // Output: smoothed rate + normalized rate
    // =========================================================
    void computeRate(const std::vector<double>& excitationField, int N, double dt)
    {
        const double minRate = TRIGGER_MIN_RATE.load(std::memory_order_relaxed);
        const double maxRate = TRIGGER_MAX_RATE.load(std::memory_order_relaxed);
        const double activityGain = TRIGGER_ACTIVITY_GAIN.load(std::memory_order_relaxed);
        const double curveExp = TRIGGER_CURVE_EXPONENT.load(std::memory_order_relaxed);

        const double tau = 0.01;
        const double alpha = 1.0 - std::exp(-dt / tau);

        for (int i = 0; i < N; i++)
        {
            double a = excitationField[i] * activityGain;

            a = std::clamp(a / 0.05, 0.0, 1.0);

            double shaped = std::pow(a, curveExp);

            double r = minRate * std::pow(maxRate / minRate, shaped);

            smoothedRate[i] += alpha * (r - smoothedRate[i]);
            rate[i] = smoothedRate[i];
        }
    }

    // =========================================================
    // OUTPUT
    // =========================================================
    const std::vector<double>& getRate() const
    {
        return rate;
    }

private:

    int Nmax = 0;
    double sampleRate;

    // Phase 1
    std::vector<double> rate;
    std::vector<double> smoothedRate;
};