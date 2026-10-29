#pragma once
#include <random>
#include <algorithm>

class PinkNoiseGenerator
{
public:
    explicit PinkNoiseGenerator(double pAmplitude = 1.0) : amp(pAmplitude)
    {
        reset();
    }

    inline double process()
    {
        const double white = dist(rng) * amp;
        const double pink = 0.5 * white + 0.5 * last;
        last = pink;
        return pink;
    }

    inline void reset()
    {
        last = 0.0;
    }

    inline void setAmplitude(double pAmplitude)
    {
        amp = pAmplitude;
    }

private:
    std::mt19937 rng{ std::random_device{}() };
    std::uniform_real_distribution<double> dist{ -1.0, 1.0 };
    double last = 0.0;
    double amp;
};