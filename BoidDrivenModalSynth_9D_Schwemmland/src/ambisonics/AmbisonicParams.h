#pragma once

struct SpatialParams
{
    double az;
    double dist;
    double distGain;
    double spread;
    int spreadSteps;

    std::array<double, 7> coeff{};
};