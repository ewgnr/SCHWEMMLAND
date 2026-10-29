#pragma once

#include "Space.h"

#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct BoidSpatial2D
{
    double azimuthRad = 0.0;  // [-π, π]
    double distance = 1.0;  // normalized [0, 1]
};

class BoidToAmbi2D
{
public:
    explicit BoidToAmbi2D(double pMaxRadius = 300.0) : maxRadius(pMaxRadius) {}

    inline BoidSpatial2D map(double x, double y, double z)
    {
        BoidSpatial2D out;

        // --- toroidal wrap
        glm::dvec3 v{ x, y, z };
        glm::dvec3 w = Space::wrap(v);

        // --- azimuth
        out.azimuthRad = Space::azimuth(w);

        // --- normalised distance
        out.distance = Space::radialNorm(w);

        return out;
    }

    inline void setMaxRadius(double r)
    {
        maxRadius = std::max(r, 1.0);
    }

private:
    double maxRadius;
};