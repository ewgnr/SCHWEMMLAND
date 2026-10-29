#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class VBAP2
{
public:
    void setup(const std::vector<double>& azimuths)
    {
        speakerAz = azimuths;
    }

    // Add the panning gains for one azimuth to an existing buffer.
    void process(double azimuth, std::vector<double>& gains) const
    {
        const std::size_t count = speakerAz.size();

        gains.assign(count, 0.0);

        if (count == 0)
            return;

        if (count == 1)
        {
            gains[0] = 1.0;
            return;
        }

        // Find an adjacent speaker pair whose angular sector
        // contains the source direction.
        for (std::size_t i = 0; i < count; ++i)
        {
            const std::size_t j = (i + 1) % count;

            const double a = speakerAz[i];
            const double b = speakerAz[j];

            const double gap = wrapPositive(b - a);
            const double offset = wrapPositive(azimuth - a);

            // A valid 2D VBAP pair must span at most 180 degrees.
            if (gap > 1e-9 && gap <= M_PI + 1e-9 && offset <= gap + 1e-9)
            {
                const double det = std::sin(gap);

                if (std::abs(det) > 1e-9)
                {
                    gains[i] = std::sin(gap - offset) / det;
                    gains[j] = std::sin(offset) / det;

                    normalize(gains);
                    return;
                }
            }
        }

        // Outside all valid sectors: route to the nearest speaker.
        std::size_t nearest = 0;
        double bestDistance = std::numeric_limits<double>::max();

        for (std::size_t i = 0; i < count; ++i)
        {
            const double distance = std::abs(std::remainder(azimuth - speakerAz[i], 2.0 * M_PI));

            if (distance < bestDistance)
            {
                bestDistance = distance;
                nearest = i;
            }
        }

        gains[nearest] = 1.0;
    }

private:
    std::vector<double> speakerAz;

    static double wrapPositive(double angle)
    {
        angle = std::fmod(angle, 2.0 * M_PI);

        if (angle < 0.0)
            angle += 2.0 * M_PI;

        return angle;
    }

    static void normalize(std::vector<double>& gains)
    {
        for (double& g : gains)
            g = std::max(0.0, g);

        double norm = 0.0;

        for (double g : gains)
            norm += g * g;

        norm = std::sqrt(norm);

        if (norm > 1e-12)
        {
            for (double& g : gains)
                g /= norm;
        }
    }
};
