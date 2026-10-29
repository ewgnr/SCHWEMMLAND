#pragma once

#include "Space.h"

#include <array>
#include <cmath>


class AmbiEncode2D
{
public:

    inline std::array<double, 7> process(double in, double azRad, double dist)
    {
        
        std::array<double, 7> out;

        // ----------------------------
        // preprocess (reused scalars)
        // ----------------------------
        const double az = azRad;
        const double d = std::abs(dist) + 1e-4;

        const double x = d * glm::pi<double>() * 0.5;
        const double gainW = std::atan(x) / x;

        const double base = in * gainW;
        const double sh = base * Space::radialGain(dist);

        // ----------------------------
        // spherical harmonics (2D, order 3)
        // ----------------------------
        const double c1 = std::cos(az);
        const double s1 = std::sin(az);
        const double c2 = std::cos(2.0 * az);
        const double s2 = std::sin(2.0 * az);
        const double c3 = std::cos(3.0 * az);
        const double s3 = std::sin(3.0 * az);

        out[0] = base;
        out[1] = c1 * sh;
        out[2] = s1 * sh;
        out[3] = c2 * sh;
        out[4] = s2 * sh;
        out[5] = c3 * sh;
        out[6] = s3 * sh;

        return out;
    }
};