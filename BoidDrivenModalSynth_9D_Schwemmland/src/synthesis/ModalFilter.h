#pragma once
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class ModalFilter
{
public:
    explicit ModalFilter(double sR = 44100.0)
        : sampleRate(sR)
    {
    }

    void init(double f, double Q, double a)
    {
        setParams(f, Q, a);
        reset();
    }

    void setParams(double f, double Q, double a)
    {
        frq = std::clamp(f, 20.0, 20000.0);
        q = std::clamp(Q, 0.1, 1000.0);
        amp = std::clamp(a, 0.0, 1.0);

        updateCoeffs();
    }

    inline double process(double x)
    {
        // Direct Form I
        double y = b0 * x + b1 * x1 + b2 * x2
            - a1 * y1 - a2 * y2;

        x2 = x1;
        x1 = x;

        y2 = y1;
        y1 = y;

        return y * amp;
    }

    void reset()
    {
        x1 = x2 = 0.0;
        y1 = y2 = 0.0;
    }

    void setSampleRate(double sr)
    {
        sampleRate = sr;
        updateCoeffs();
    }

private:
    void updateCoeffs()
    {
        const double w0 = 2.0 * M_PI * frq / sampleRate;
        const double alpha = std::sin(w0) / (2.0 * q);

        const double cosw = std::cos(w0);

        // bandpass (constant skirt gain, peak gain = Q)
        double _b0 = alpha;
        double _b1 = 0.0;
        double _b2 = -alpha;
        double _a0 = 1.0 + alpha;
        double _a1 = -2.0 * cosw;
        double _a2 = 1.0 - alpha;

        // normalize
        b0 = _b0 / _a0;
        b1 = _b1 / _a0;
        b2 = _b2 / _a0;
        a1 = _a1 / _a0;
        a2 = _a2 / _a0;
    }

    double sampleRate = 44100.0;

    double frq = 440.0;
    double q = 1.0;
    double amp = 1.0;

    double b0 = 0, b1 = 0, b2 = 0;
    double a1 = 0, a2 = 0;

    double x1 = 0, x2 = 0;
    double y1 = 0, y2 = 0;
};