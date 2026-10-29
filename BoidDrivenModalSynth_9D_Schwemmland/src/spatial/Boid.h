#pragma once

#include "BoidDefs.h"

struct Boid
{
    int index = 0;
    float position[DimsPerBoid] = { 0 };
    float velocity[DimsPerBoid] = { 0 };
};