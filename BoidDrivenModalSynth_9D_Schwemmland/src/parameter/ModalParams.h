#pragma once
#include <vector>

struct ModalParams
{
    std::vector<double> freq;
    std::vector<double> bw;
    std::vector<double> amp;
};

struct ModalParamsBuffer
{
    std::vector<ModalParams> perBoid;
};